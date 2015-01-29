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
void SaveGeometrytoShape(std::string name, const OGRMultiPolygon* G)
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

void SaveGeometrytoShape(std::string name, const OGRGeometry* G)
{
    OGRMultiPolygon * Temp = new OGRMultiPolygon;
    Temp->addGeometry(G);
    SaveGeometrytoShape(name, Temp);
    delete Temp;
}
