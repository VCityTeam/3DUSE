#include "ExtractRoof.hpp"
#include "libcitygml/utils/CityGMLtools.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtools.hpp"

std::pair<OGRGeometry*, OGRGeometry*> sortRoofs( citygml::CityModel* city )
{
   std::pair<OGRGeometry*, OGRGeometry*> output;

   const citygml::CityObjects* roofs = new citygml::CityObjects();
   citygml::ParserParams params;

   roofs = city->getCityObjectsByType( citygml::CityObjectsType::COT_RoofSurface );
   //Recuperation des toits
   if ( !roofs )
      return output;
   //Creation des OGR pour ensuite creer les fichier shapefiles et pouvoirvisualiser le resultat
   OGRMultiPolygon* toitsPlats = new OGRMultiPolygon(); //OGR toits plats
   OGRMultiPolygon * autresToits = new OGRMultiPolygon(); //OGR toits penches
   for ( citygml::CityObject* obj : *( roofs ) ) //Parcours de tous les toits
   {
      for ( citygml::Geometry* geom : obj->getGeometries() ) //Parcours de toutes les geometries
      {
         for ( citygml::Polygon* poly : geom->getPolygons() ) //Parcours de tous les polygones
         {
            //Compute Normals
            TVec3d normal = poly->computeNormal();
            //New OGR Objects
            OGRPolygon * ogrPoly = new OGRPolygon(); //Nouveau Polygone OGR
            OGRLinearRing * ogrRing = new OGRLinearRing(); //Nouveau Ring OGR
                                                           //Put Vertices in new OGR Geometry
            for ( TVec3d Vertices : poly->getExteriorRing()->getVertices() )
               //Parcours de tous les points 3D du contour (ExteriorRing) du Polygone
            {
               ogrRing->addPoint( Vertices.x, Vertices.y/*, Vertices.z*/ );
               //Ajout du point courant au contour du polygone OGR
            }
            ogrRing->closeRings(); //Fermeture du contour du polygone OGR
            if ( ogrRing->getNumPoints() > 3 )//Le polygone ne sera cree qu'a partir de 4 points
            {
               ogrPoly->addRingDirectly( ogrRing ); //Ajout du contour au polygone OFR
               if ( ogrPoly->IsValid() ) //Si le polygone est valide
               {
                  //Compute scalar product to know if flat roof or not
                  if ( normal.dot( TVec3d( 0., 0., 1. ) ) >= 0.9999 )
                     toitsPlats->addGeometryDirectly( ogrPoly ); // Ajout du polygone a la geometrie Toits plats( une OGRGeometry peut etre un OGRMultiPolygon, un OGRPolygon... )
                  else
                     autresToits->addGeometryDirectly( ogrPoly );
               }
            }
         }
      }
   }
   OGRGeometry* flatRoofs = toitsPlats->UnionCascaded();
   OGRGeometry* otherRoofs = autresToits->UnionCascaded();
   //Save Geometry to shape files
   //SaveGeometrytoShape( "FlatRoofs.shp", flatRoofs );
   //SaveGeometrytoShape( "OtherRoofs.shp", otherRoofs );

   output.first = flatRoofs;
   output.second = otherRoofs;

   return output;
}
