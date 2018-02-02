// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "OGRGDALtoShpWriter.hpp"
#include <iostream>
#include <QDir>
////////////////////////////////////////////////////////////////////////////////

/**
* @brief Sauvegarde la geometry dans un fichier shape
* @param name Nom du shape a enregistrer
* @param G Geometry a enregistrer
*/
void SaveGeometrytoShape( std::string name, const OGRGeometryCollection* G )
{
   if ( name.find( '/' ) != std::string::npos )
   {
      std::string Path = name.substr( 0, name.find_last_of( '/' ) );

      if ( !QDir( QString::fromStdString( Path ) ).exists() )
         QDir().mkdir( QString::fromStdString( Path ) );
   }

   const char * DriverName = "ESRI Shapefile";
   GDALDriver * Driver;

   GDALAllRegister();
   Driver = GetGDALDriverManager()->GetDriverByName( DriverName );
   if ( Driver == NULL )
   {
      printf( "%s driver not available.\n", DriverName );
      return;
   }

   remove( name.c_str() );
   GDALDataset * DS = Driver->Create( name.c_str(), 0, 0, 0, GDT_Unknown, NULL );

   //OGRSpatialReference * SRS = new OGRSpatialReference;
   //SRS->importFromEPSG(3946);

   //OGRLayer * Layer = DS->CreateLayer("Layer1", SRS);
   OGRLayer * Layer = DS->CreateLayer( "Layer1" );

   for ( int i = 0; i < G->getNumGeometries(); ++i )
   {
      OGRGeometry * Geometry = G->getGeometryRef( i )->clone();

      OGRFeature * Feature = OGRFeature::CreateFeature( Layer->GetLayerDefn() );

      Feature->SetGeometry( Geometry );

      if ( Layer->CreateFeature( Feature ) != OGRERR_NONE )
      {
         printf( "Failed to create feature in shapefile.\n" );
         exit( 1 );
      }

      OGRFeature::DestroyFeature( Feature );
   }
   GDALClose( DS );

   std::cout << "Fichier " << name << " cree." << std::endl;
}

////////////////////////////////////

void SaveGeometrytoShape( std::string name, const OGRGeometry* G )
{
   if ( G->getGeometryType() == wkbPolygon || G->getGeometryType() == wkbPolygon25D || G->getGeometryType() == wkbPoint || G->getGeometryType() == wkbPoint25D || G->getGeometryType() == wkbLineString || G->getGeometryType() == wkbLineString25D )
   {
      OGRGeometryCollection * Temp = new OGRGeometryCollection;
      Temp->addGeometry( G );
      SaveGeometrytoShape( name, Temp );
      delete Temp;
   }
   else if ( G->getGeometryType() == wkbMultiPolygon || G->getGeometryType() == wkbMultiPolygon25D || G->getGeometryType() == wkbMultiLineString || G->getGeometryType() == wkbMultiLineString25D || G->getGeometryType() == wkbGeometryCollection || G->getGeometryType() == wkbGeometryCollection25D )
   {
      SaveGeometrytoShape( name, (OGRGeometryCollection *)G );
   }
}

