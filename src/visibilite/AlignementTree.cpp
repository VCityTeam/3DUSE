#include "visibilite/AlignementTree.hpp"

#include <qstring.h>
#include <qfiledialog.h>

#include "core/application.hpp"
#include "gui/osg/osgGDAL.hpp"
#include "gui/osg/osgScene.hpp"
#include "export/exportCityGML.hpp"

#include "visibilite/ShpExtrusion.h"

const float Pi = 3.141592654f;

/**
*	@brief Convert a degree float to radian
*/
inline float DegToRad(float x)
{
	return x / 180 * Pi;
}

/**
*	@brief Convert a radian float to degree
*/
inline float RadToDeg(float x)
{
	return x / Pi * 180;
}

/**
*	Create a cylinder in a citymodel
*	@param pos 3D position of the cylinder
*	@param ModelOut CityModel where to put the cylinder
*/
void GenCylindre(TVec3d pos, citygml::CityModel* ModelOut)
{
	static int cpt = 0;
	float radius = 6.0;
	float height = 20.0;

	int slice = 4;
	float anglePerSlice = 360.f / (float(slice));

	std::vector<TVec3d> circle;

	float currentAngle = 0;
	for(unsigned int i = 0; i < slice; i++)
	{
		float x = radius * cos( DegToRad(currentAngle) );
		float y = radius * sin( DegToRad(currentAngle) );
		float z = 0;

		circle.push_back(TVec3d(x,y,z));

		currentAngle += anglePerSlice;
	}

	std::vector<TVec3d> circleTop;

	for(unsigned int i = 0; i < circle.size(); i++)
	{
		circleTop.push_back(TVec3d(circle[i].x,circle[i].y,height));
	}

	std::string name = "test_"+cpt;

	citygml::CityObject* BuildingCO = new citygml::SolitaryVegetationObject(name);

	citygml::CityObject* RoofCO = new citygml::RoofSurface(name+"_Roof");
	citygml::Geometry* Roof = new citygml::Geometry(name+"_RoofGeometry");
	citygml::CityObject* WallCO = new citygml::WallSurface(name+"_Wall");
	citygml::Geometry* Wall = new citygml::Geometry(name+"_WallGeometry");

	{
		citygml::LinearRing* ring = new citygml::LinearRing(name+"_ring",true);

		for(unsigned int i = 0; i < circle.size(); i++)
		{
			ring->addVertex(circle[i]+pos);
		}

		citygml::Polygon* poly = new citygml::Polygon(name+"_poly");
		poly->addRing(ring);
		Roof->addPolygon(poly);
	}

	{
		citygml::LinearRing* ring = new citygml::LinearRing(name+"_ring",true);

		for(unsigned int i = 0; i < circleTop.size(); i++)
		{
			ring->addVertex(circleTop[i]+pos);
		}

		citygml::Polygon* poly = new citygml::Polygon(name+"_poly");
		poly->addRing(ring);
		Roof->addPolygon(poly);
	}

	for(unsigned int i = 0; i < circle.size(); i++)
	{
		citygml::LinearRing* ring = new citygml::LinearRing(name+"_ring",true);

		ring->addVertex(circle[i]+pos);
		ring->addVertex(circle[(i+1)%circle.size()]+pos);
		ring->addVertex(circleTop[(i+1)%circle.size()]+pos);
		ring->addVertex(circleTop[i]+pos);

		citygml::Polygon* poly = new citygml::Polygon(name+"_poly");
		poly->addRing(ring);
		Wall->addPolygon(poly);
	}

	RoofCO->addGeometry(Roof);
	ModelOut->addCityObject(RoofCO);
	BuildingCO->insertNode(RoofCO);
	WallCO->addGeometry(Wall);
	ModelOut->addCityObject(WallCO);
	BuildingCO->insertNode(WallCO);

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

		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
		osg::Vec3d offset(offset_.x, offset_.y, offset_.z);


		OGRLayer *poLayer;
        int nbLayers = poDS->GetLayerCount();

		LRing points;

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

				}
			}
		}

		points = PutLRingOnTerrain(points,offset);

		for(unsigned int i = 0; i < points.size();i++)
		{
			GenCylindre(points[i],ModelOut);
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