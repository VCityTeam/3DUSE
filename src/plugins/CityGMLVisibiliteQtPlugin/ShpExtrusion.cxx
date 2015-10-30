#include "ShpExtrusion.h"

#include <qstring.h>
#include <qfiledialog.h>

#include "gui/osg/osgGDAL.hpp"
#include "gui/osg/osgScene.hpp"
#include "export/exportCityGML.hpp"

#include <unordered_map>

#include "data/AABB.hpp"
#include "Visibilite.hpp"

std::map<std::string,TriangleList*> tileTriangles;

/**
*	@brief Get a linear ring modify by a height
*/
std::vector<TVec3d>  GetLRingWidthHeight(LRing poly, double zmax)
{
	std::vector<TVec3d> result;
	result.resize(poly.size());

	for(unsigned int i = 0; i < poly.size(); i++)
	{
		result[i] = TVec3d(poly[i].x,poly[i].y,poly[i].z + zmax);
	}

	return result;
}

/**
*	@brief Convert our linear ring to a citygml linear ring
*/
citygml::LinearRing* LRingToCityRing(LRing polyvec, std::string name, bool exterior = true)
{
	citygml::LinearRing* ring = new citygml::LinearRing(name+"_ring",exterior);
	for(unsigned int j = 0; j < polyvec.size(); j++)
	{
		ring->addVertex(polyvec[j]);
	}
	return ring;
}

/**
*	@brief Build a citygml polygon from an extern linear ring and several intern linear ring
*/
citygml::Polygon* BuildPolygon(LRing ringextern, std::vector<LRing> ringintern, std::string name)
{
	citygml::Polygon* poly = new citygml::Polygon(name+"_poly");

	poly->addRing(LRingToCityRing(ringextern,name));
	for(std::vector<TVec3d> vec : ringintern)
		poly->addRing(LRingToCityRing(vec,name,false));

	return poly;
}

/**
*	@brief Build the wall polygon from the points of the ground and the roof
*/
std::vector<LRing> GetWall(LRing sol, LRing toit)
{
	std::vector<LRing> result;
	if(sol.size() == toit.size())
	{
		for(unsigned int i = 0; i < sol.size()-1; i++)
		{
			std::vector<TVec3d> poly;

			poly.push_back(sol[i]);
			poly.push_back(sol[i+1]);
			poly.push_back(toit[i+1]);
			poly.push_back(toit[i]);

			result.push_back(poly);
		}

		LRing polyTemp;

		polyTemp.push_back(sol[sol.size()-1]);
		polyTemp.push_back(sol[0]);
		polyTemp.push_back(toit[0]);
		polyTemp.push_back(toit[sol.size()-1]);

		result.push_back(polyTemp);

	}

	return result;
}

/**
*	@brief Convert and OGR linear ring to ours
*/
LRing OGRLinearRingToLRing(OGRLinearRing* poLR)
{
	OGRPoint p;
    TVec3d v;
	LRing ptsSol;
    for(int i=0; i<poLR->getNumPoints(); ++i)
    {
        poLR->getPoint(i, &p);
        v = TVec3d(p.getX(), p.getY(), 0);
        ptsSol.push_back(v);
    }

	return ptsSol;
}


LRing PutLRingOnTerrain(LRing ring, std::string dir)
{
	//Load all terrain bounding box that are abox the points
	AABBCollection boxes = LoadAABB(dir);

	std::vector<AABB> ptAABB;
	LRing ptResult;

	for(TVec3d vec : ring)
	{
		for(AABB box : boxes.terrain)
		{
			TVec3d min = box.min;
			TVec3d max = box.max;
			if(vec.x >= min.x && vec.x <= max.x && vec.y >= min.y && vec.y <= max.y)
			{
				ptAABB.push_back(box);
				ptResult.push_back(vec);
				break;
			}
		}
	}

	if(ptResult.size() != ring.size())
	{
		//std::cout << "Error some point are out of range of tiles" << std::endl;
		ptResult.clear();
		return ptResult;
	}

	for(unsigned int i = 0; i < ptResult.size(); i++)
	{
		TriangleList* trianglesTemp;
		if(tileTriangles.find(ptAABB[i].name) != tileTriangles.end())
		{
			trianglesTemp = tileTriangles[ptAABB[i].name];
		}
		else
		{
			std::string path = dir + ptAABB[i].name;
			//Get the triangle list
			trianglesTemp = BuildTriangleList(path,citygml::CityObjectsType::COT_TINRelief);
			tileTriangles.insert(std::make_pair(ptAABB[i].name,trianglesTemp));
		}

		Ray ray(ptResult[i],TVec3d(0.0,0.0,1.0));
		bool Intesect = false;

		for(unsigned int j = 0; j < trianglesTemp->triangles.size(); j++)
		{
			Hit hit;
			if(ray.Intersect(trianglesTemp->triangles[j], &hit)) //Check if the ray hit the triangle
			{
				ptResult[i].z = hit.point.z;
				Intesect = true;
				break;
			}
		}
		if(!Intesect) //Si un point ne peut pas se projeter sur le terrain, alors on enlève tout le bâtiment pour ne pas avoir de modèles diformes.
		{
			ptResult.clear();
			return ptResult;
		}
	}

	return ptResult;
}


void ShpExtruction(std::string dir)
{
	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load shp file");

	QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

	if(ext == "shp")
    {
		citygml::CityModel* ModelOut = new citygml::CityModel;

		OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), TRUE/*FALSE*/);
		std::cout << "Shp loaded" << std::endl;
		std::cout << "Processing..." << std::endl;


		OGRLayer *poLayer;
        int nbLayers = poDS->GetLayerCount();
        if(nbLayers > 0)
        {
			poLayer = poDS->GetLayer(0);

			OGRFeature *poFeature;
            poLayer->ResetReading();

			unsigned int cpt = 0;

			while( (poFeature = poLayer->GetNextFeature()) != NULL )
            {
				std::string name = "test_"+std::to_string(cpt++);

				OGRGeometry* poGeometry = poFeature->GetGeometryRef();

				if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
                {
					citygml::CityObject* BuildingCO = new citygml::Building(name);
					
					citygml::CityObject* RoofCO = new citygml::RoofSurface(name+"_Roof");
					citygml::Geometry* Roof = new citygml::Geometry(name+"_RoofGeometry", citygml::GT_Roof, 2);
					citygml::CityObject* WallCO = new citygml::WallSurface(name+"_Wall");
					citygml::Geometry* Wall = new citygml::Geometry(name+"_WallGeometry", citygml::GT_Wall, 2);

					//Emprise au sol
					OGRPolygon* poPG = (OGRPolygon*) poGeometry;

                    LRing ptsSol = PutLRingOnTerrain(OGRLinearRingToLRing(poPG->getExteriorRing()), dir);

					if(ptsSol.size() == 0) //La génération a posé problème, probablement parce que cette emprise au sol n'est pas complètement sur le terrain
						continue;

					double H = 20;
					double Zmin = ptsSol.front().z;
					if(poFeature->GetFieldIndex("HAUTEUR") != -1)
						H = poFeature->GetFieldAsDouble("HAUTEUR");
					if(poFeature->GetFieldIndex("Z_MIN") != -1)
						Zmin = poFeature->GetFieldAsDouble("Z_MIN");
					double Zmax = H;
					Zmax = Zmax > 5000 ? ptsSol.front().z + 20 : Zmax;


					LRing ptsToit = GetLRingWidthHeight(ptsSol, Zmax);

					std::vector<LRing> ptsSolIntern;
					std::vector<LRing> ptsToitIntern;

					for(unsigned int i = 0; i < (unsigned int)poPG->getNumInteriorRings();i++)
					{
						LRing ptsSolTemp = PutLRingOnTerrain(OGRLinearRingToLRing(poPG->getInteriorRing(i)), dir);
						if(ptsSolTemp.size() == 0)
							continue;
						LRing ptsToitTemp = GetLRingWidthHeight(ptsSolTemp, Zmax);
						ptsSolIntern.push_back(ptsSolTemp);
						ptsToitIntern.push_back(ptsToitTemp);
					}
					
					Roof->addPolygon(BuildPolygon(ptsSol,ptsSolIntern,name));
					Roof->addPolygon(BuildPolygon(ptsToit,ptsToitIntern,name));

					std::vector<std::vector<TVec3d>> walls = GetWall(ptsSol, ptsToit);
					
					for(unsigned int i = 0; i < walls.size(); i++)
					{
						Wall->addPolygon(BuildPolygon(walls[i],std::vector<std::vector<TVec3d>>(),name));
					}

					for(unsigned int i = 0; i < ptsSolIntern.size(); i++)
					{
						std::vector<std::vector<TVec3d>> wallsTemp = GetWall(ptsSolIntern[i], ptsToitIntern[i]);

						for(unsigned int j = 0; j < wallsTemp.size(); j++)
						{
							Wall->addPolygon(BuildPolygon(wallsTemp[j],std::vector<std::vector<TVec3d>>(),name));
						}
					}

					RoofCO->addGeometry(Roof);
					ModelOut->addCityObject(RoofCO);
					BuildingCO->insertNode(RoofCO);
					WallCO->addGeometry(Wall);
					ModelOut->addCityObject(WallCO);
					BuildingCO->insertNode(WallCO);



					ModelOut->addCityObject(BuildingCO);
					ModelOut->addCityObjectAsRoot(BuildingCO);
				}
			}
		}

		for(auto it = tileTriangles.begin(); it != tileTriangles.end(); it++)
			delete it->second;

		std::cout << "Exporting citygml" << std::endl;
		ModelOut->computeEnvelope();

		QDir dir;
		dir.mkdir("./ShpExtruded/");

		citygml::ExporterCityGML exporter("./ShpExtruded/"+file.baseName().toStdString()+".gml");
		exporter.exportCityModel(*ModelOut);
		std::cout << "Done exporting" << std::endl;
	}

	

}