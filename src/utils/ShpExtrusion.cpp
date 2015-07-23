#include "utils/ShpExtrusion.h"

#include <qstring.h>
#include <qfiledialog.h>

#include "core/application.hpp"
#include "gui/osg/osgGDAL.hpp"
#include "gui/osg/osgScene.hpp"
#include "export/exportCityGML.hpp"


std::vector<TVec3d>  GetPolyWidthHeight(std::vector<TVec3d> poly)
{
	std::vector<TVec3d> result;
	result.resize(poly.size());

	for(unsigned int i = 0; i < poly.size(); i++)
	{
		result[i] = TVec3d(poly[i].x,poly[i].y,50.0);
	}

	return result;
}

citygml::Polygon* PolyToPoly(std::vector<TVec3d> polyvec, std::string name)
{
	citygml::Polygon* poly = new citygml::Polygon(name+"_poly");
	citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringext",true);
	for(unsigned int j = 0; j < polyvec.size(); j++)
	{
		ring->addVertex(polyvec[j]);
	}
	poly->addRing(ring);

	return poly;
}

std::vector<std::vector<TVec3d>> GetWall(std::vector<TVec3d> sol, std::vector<TVec3d> toit)
{
	std::vector<std::vector<TVec3d>> result;
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

		std::vector<TVec3d> polyTemp;

		polyTemp.push_back(sol[sol.size()-1]);
		polyTemp.push_back(sol[0]);
		polyTemp.push_back(toit[0]);
		polyTemp.push_back(toit[sol.size()-1]);

		result.push_back(polyTemp);

	}

	return result;
}

void ShpExtruction()
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

		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
		osg::Vec3d offset(offset_.x, offset_.y, offset_.z);


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


					//Emprise au sol
					OGRPolygon* poPG = (OGRPolygon*) poGeometry;
                    OGRLinearRing* poLR = poPG->getExteriorRing();

					OGRPoint p;
                    TVec3d v;
					std::vector<TVec3d> ptsSol;
                    for(int i=0; i<poLR->getNumPoints(); ++i)
                    {
                        poLR->getPoint(i, &p);
                        v = TVec3d(p.getX(), p.getY(), 0);
                        ptsSol.push_back(v);
                    }

					std::vector<TVec3d> ptsToit = GetPolyWidthHeight(ptsSol);

					std::vector<std::vector<TVec3d>> walls = GetWall(ptsSol, ptsToit);

					//Export toit
					citygml::CityObject* RoofCO = new citygml::RoofSurface(name+"_Roof");
					citygml::Geometry* Roof = new citygml::Geometry(name+"_RoofGeometry", citygml::GT_Roof, 2);

					Roof->addPolygon(PolyToPoly(ptsSol,name));
					Roof->addPolygon(PolyToPoly(ptsToit,name));


					RoofCO->addGeometry(Roof);
					ModelOut->addCityObject(RoofCO);
					BuildingCO->insertNode(RoofCO);


					//Export Wall

					citygml::CityObject* WallCO = new citygml::WallSurface(name+"_Wall");
					citygml::Geometry* Wall = new citygml::Geometry(name+"_WallGeometry", citygml::GT_Wall, 2);

					for(unsigned int i = 0; i < walls.size(); i++)
					{
						Wall->addPolygon(PolyToPoly(walls[i],name));
					}


					WallCO->addGeometry(Wall);
					ModelOut->addCityObject(WallCO);
					BuildingCO->insertNode(WallCO);



					ModelOut->addCityObject(BuildingCO);
					ModelOut->addCityObjectAsRoot(BuildingCO);
				}
			}
		}

		std::cout << "Exporting citygml" << std::endl;
		ModelOut->computeEnvelope();

		citygml::ExporterCityGML exporter("IchBinEinTest.gml");
		exporter.exportCityModel(*ModelOut);
		std::cout << "Done exporting" << std::endl;
	}

	

}