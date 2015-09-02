#include "visibilite/AlignementTree.hpp"

#include <qstring.h>
#include <qfiledialog.h>

#include "core/application.hpp"
#include "gui/osg/osgGDAL.hpp"
#include "gui/osg/osgScene.hpp"
#include "export/exportCityGML.hpp"

#include "visibilite/ShpExtrusion.h"

/**
*	@brief Convert a degree float to radian
*/
inline float DegToRad(float x)
{
	return x / 180 * M_PI;
}

/**
*	@brief Convert a radian float to degree
*/
inline float RadToDeg(float x)
{
	return x / M_PI * 180;
}

citygml::Geometry* BuildCube(std::string name,TVec3d pos, float radius,float height)
{
	float offsetPoint = radius / sqrt(2);

	citygml::Geometry* cityGeom = new citygml::Geometry(name+"_Geometry", citygml::GT_Unknown, 2);

	{citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringFutBot",true);
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y+offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y+offsetPoint,pos.z));
	citygml::Polygon* poly = new citygml::Polygon(name+"_polyFutBot");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}

	{citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringFutTop",true);
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y+offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y+offsetPoint,pos.z+height));
	citygml::Polygon* poly = new citygml::Polygon(name+"_polyFutTop");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}

	{citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringFutFront",true);
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z+height));
	citygml::Polygon* poly = new citygml::Polygon(name+"_polyFutFront");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}

	{citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringFutBack",true);
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y+offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y+offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z+height));
	citygml::Polygon* poly = new citygml::Polygon(name+"_polyFutBack");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}

	{citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringFutLeft",true);
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y+offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y-offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x-offsetPoint,pos.y+offsetPoint,pos.z+height));
	citygml::Polygon* poly = new citygml::Polygon(name+"_polyFutLeft");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}

	{citygml::LinearRing* ring = new citygml::LinearRing(name+"_ringFutRight",true);
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y+offsetPoint,pos.z));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y+offsetPoint,pos.z+height));
	ring->addVertex(TVec3d(pos.x+offsetPoint,pos.y-offsetPoint,pos.z+height));
	citygml::Polygon* poly = new citygml::Polygon(name+"_polyFutRight");
	poly->addRing(ring);
	cityGeom->addPolygon(poly);}

	return cityGeom;
}

/**
*	Create a cylinder in a citymodel
*	@param pos 3D position of the cylinder
*	@param ModelOut CityModel where to put the cylinder
*/
void GenCylindre(TVec3d pos,OGRFeature* feature, citygml::CityModel* ModelOut)
{
	static int cpt = 0;
	float radiusFut = 1.0;
	float heightFut = 6.0;
	float heightLeaf = 6.0;
	float radiusLeaf = 3.0;


	std::string name = "noid";
	if(feature->GetFieldIndex("identifian") != -1)
		name = feature->GetFieldAsString("identifian");// "codefuv"/"gid"

	name += "_tree";

	if(feature->GetFieldIndex("circonfere") != -1)
		radiusFut = ((feature->GetFieldAsDouble("circonfere") / M_PI)/100)/2;//On a une ciconference en cm...

	if(feature->GetFieldIndex("hauteurfut") != -1)
		heightFut = feature->GetFieldAsDouble("hauteurfut");

	if(feature->GetFieldIndex("hauteurtot") != -1)
		heightLeaf = feature->GetFieldAsDouble("hauteurtot") - heightFut;

	if(feature->GetFieldIndex("rayoncouro") != -1)
		radiusLeaf = feature->GetFieldAsDouble("rayoncouro");

	
	std::cout << name << " " << radiusFut<< " "<< heightFut<< " "<< heightLeaf<< " "<< radiusLeaf<< std::endl;
	
	citygml::CityObject* BuildingCO = new citygml::SolitaryVegetationObject(name);

	BuildingCO->addGeometry(BuildCube(name,pos,radiusFut,heightFut));
	BuildingCO->addGeometry(BuildCube(name,TVec3d(pos.x,pos.y,pos.z + heightFut),radiusLeaf,heightLeaf));

	ModelOut->addCityObject(BuildingCO);
	ModelOut->addCityObjectAsRoot(BuildingCO);

	cpt++;
}

void ExtrudeAlignementTree()
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

		TVec3d offset = vcity::app().getSettings().getDataProfile().m_offset;


		OGRLayer *poLayer;
        int nbLayers = poDS->GetLayerCount();

		LRing points;
		std::vector<OGRFeature*> pointsFeatures;

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

				if(poGeometry != NULL && poGeometry->getGeometryType() == wkbPoint)
				{
                    OGRPoint* poP = (OGRPoint*) poGeometry;
                    
					points.push_back(TVec3d(poP->getX(),poP->getY(),0.0));
					pointsFeatures.push_back(poFeature);

				}
			}
		}

		points = PutLRingOnTerrain(points,offset);

		for(unsigned int i = 0; i < points.size();i++)
		{
			GenCylindre(points[i],pointsFeatures[i],ModelOut);
		}


		std::cout << "Exporting citygml" << std::endl;
		ModelOut->computeEnvelope();

		QDir dir;
		dir.mkdir("./ShpExtruded/");

		citygml::ExporterCityGML exporter("./ShpExtruded/"+file.baseName().toStdString()+".gml");
		exporter.exportCityModel(*ModelOut);
		std::cout << "Done exporting" << std::endl;
	}
}