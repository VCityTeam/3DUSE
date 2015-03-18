#include "ExportToShape.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////

/**
* @brief Sauvegarde la geometry dans un fichier shape
* @param name Nom du shape à enregistrer
* @param G Geometry à enregistrer (MultiPolygon)
*/
void SaveGeometrytoShape(std::string name, const OGRGeometryCollection* G)
{
	const char * DriverName = "ESRI Shapefile";
    OGRSFDriver * Driver;

    OGRRegisterAll();
    Driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(DriverName);
    if( Driver == NULL )
    {
        printf( "%s driver not available.\n", DriverName );
        return;
    }

    remove(name.c_str());
    OGRDataSource * DS = Driver->CreateDataSource(name.c_str(), NULL);

	//OGRSpatialReference * SRS = new OGRSpatialReference;
	//SRS->importFromEPSG(3946);

    //OGRLayer * Layer = DS->CreateLayer("Layer1", SRS);
	OGRLayer * Layer = DS->CreateLayer("Layer1");

    for(int i = 0; i < G->getNumGeometries(); ++i)
    {
        OGRGeometry * Geometry =  G->getGeometryRef(i)->clone();

        OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());

        Feature->SetGeometry(Geometry);
        Layer->CreateFeature(Feature);

        OGRFeature::DestroyFeature(Feature);
    }
    OGRDataSource::DestroyDataSource(DS);

    std::cout << "Fichier " << name << " cree." << std::endl;
}

/*void SaveGeometrytoShape(std::string name, const OGRMultiPolygon* G)
{
    const char * DriverName = "ESRI Shapefile";
    OGRSFDriver * Driver;

    OGRRegisterAll();
    Driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(DriverName);
    if( Driver == NULL )
    {
        printf( "%s driver not available.\n", DriverName );
        return;
    }

    remove(name.c_str());
    OGRDataSource * DS = Driver->CreateDataSource(name.c_str(), NULL);

	//OGRSpatialReference * SRS = new OGRSpatialReference;
	//SRS->importFromEPSG(3946);

    //OGRLayer * Layer = DS->CreateLayer("Layer1", SRS);
	OGRLayer * Layer = DS->CreateLayer("Layer1");

    for(int i = 0; i < G->getNumGeometries(); ++i)
    {
        if(G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon && G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon25D)
            continue;

        OGRPolygon * Polygon =  dynamic_cast<OGRPolygon*>(G->getGeometryRef(i)->clone());

        OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());

        Feature->SetGeometry(Polygon);
        Layer->CreateFeature(Feature);

        OGRFeature::DestroyFeature(Feature);
    }
    OGRDataSource::DestroyDataSource(DS);

    std::cout << "Fichier " << name << " cree." << std::endl;
}

////////////////////////////////////

void SaveGeometrytoShape(std::string name, const OGRMultiLineString* G)
{
    const char * DriverName = "ESRI Shapefile";
    OGRSFDriver * Driver;

    OGRRegisterAll();
    Driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(DriverName);
    if( Driver == NULL )
    {
        printf( "%s driver not available.\n", DriverName );
        return;
    }

    remove(name.c_str());
    OGRDataSource * DS = Driver->CreateDataSource(name.c_str(), NULL);

	//OGRSpatialReference * SRS = new OGRSpatialReference;
	//SRS->importFromEPSG(3946);

    //OGRLayer * Layer = DS->CreateLayer("Layer1", SRS);
	OGRLayer * Layer = DS->CreateLayer("Layer1");

    for(int i = 0; i < G->getNumGeometries(); ++i)
    {
		if(G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbLineString && G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbLineString25D)
            continue;

		OGRLineString * LineString =  dynamic_cast<OGRLineString*>(G->getGeometryRef(i)->clone());

        OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());

        Feature->SetGeometry(LineString);
        Layer->CreateFeature(Feature);

        OGRFeature::DestroyFeature(Feature);
    }
    OGRDataSource::DestroyDataSource(DS);

    std::cout << "Fichier " << name << " cree." << std::endl;
}*/

////////////////////////////////////

void SaveGeometrytoShape(std::string name, const OGRGeometry* G)
{
	if(G->getGeometryType() == wkbPolygon || G->getGeometryType() == wkbPolygon25D || G->getGeometryType() == wkbPoint || G->getGeometryType() == wkbPoint25D ||G->getGeometryType() == wkbLineString || G->getGeometryType() == wkbLineString25D)
	{
		OGRGeometryCollection * Temp = new OGRGeometryCollection;
		Temp->addGeometry(G);
		SaveGeometrytoShape(name, Temp);
		delete Temp;
	}
	else if(G->getGeometryType() == wkbMultiPolygon || G->getGeometryType() == wkbMultiPolygon25D)
	{
		SaveGeometrytoShape(name, (OGRGeometryCollection *)G);
	}
	else if(G->getGeometryType() == wkbMultiLineString || G->getGeometryType() == wkbMultiLineString25D)
	{
		SaveGeometrytoShape(name, (OGRGeometryCollection *)G);
	}
}

