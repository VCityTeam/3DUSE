// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )
////////////////////////////////////////////////////////////////////////////////
#include "osgGDAL.hpp"
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/PolygonMode>
#include <stdio.h>
#include <iostream>
#include "libcitygml/tesselator.hpp"
#include "core/application.hpp"
#include <vector>

//#define FILL_POLYGON

////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> buildOsgGDAL( GDALDataset* poDS, std::string name )
{
   TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
   osg::Vec3d offset( offset_.x, offset_.y, offset_.z );

   if ( poDS )
   {
      printf( "Load using Gdal / OGR\n" );
      osg::ref_ptr<osg::Geode> geode = new osg::Geode;

      geode->setName( name );

      OGRLayer *poLayer;
      int nbLayers = poDS->GetLayerCount();
      if ( nbLayers > 0 )
      {
         printf( "nb layer %d\n", nbLayers );
         poLayer = poDS->GetLayer( 0 );
         printf( "layer %s. Type : %d\n", poLayer->GetName(), poLayer->GetGeomType() );
         //poLayer = poDS->GetLayerByName( "point" );

         Tesselator tess;

         OGRFeature *poFeature;
         poLayer->ResetReading();
         printf( "nb feature %d\n", poLayer->GetFeatureCount() );
         while ( ( poFeature = poLayer->GetNextFeature() ) != NULL )
         {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            //if(poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString25D)
            if ( poGeometry != NULL && poGeometry->getGeometryType() == wkbLineString25D )
            {
               osg::Geometry* geom = new osg::Geometry;
               osg::Vec3Array* vertices = new osg::Vec3Array;
               osg::DrawElementsUInt* indices = new osg::DrawElementsUInt( osg::PrimitiveSet::LINE_STRIP, 0 );

               OGRLineString* poLS = (OGRLineString*)poGeometry;
               int nbPoints = poLS->getNumPoints();
               for ( int i = 0; i < nbPoints; ++i )
               {
                  OGRPoint p;
                  poLS->getPoint( i, &p );
                  //printf( "%f, %f; %f\n", p.getX(), p.getY(), p.getZ() );
                  osg::Vec3d pt = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                  vertices->push_back( pt );
                  indices->push_back( i );
               }

               //geom->getOrCreateStateSet();

               geom->setVertexArray( vertices );
               geom->addPrimitiveSet( indices );
               geode->addDrawable( geom );
            }
            else if ( poGeometry != NULL && ( poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon ) )
            {
#ifdef FILL_POLYGON
               osg::Geometry* geom = new osg::Geometry;
               osg::Vec3Array* vertices = new osg::Vec3Array;
               osg::DrawElementsUInt* indices = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );
               osg::Vec3Array* colors = new osg::Vec3Array;

               OGRPolygon* poPG = (OGRPolygon*)poGeometry;
               OGRLinearRing* poLR = poPG->getExteriorRing();

               // count vertices
               unsigned int verticesCount = poLR->getNumPoints();
               for ( int i = 0; i < poPG->getNumInteriorRings(); ++i )
               {
                  verticesCount += poPG->getInteriorRing( i )->getNumPoints();
               }
               tess.init( verticesCount, TVec3d( 0.0, 0.0, 1.0 ) );

               // feed data
               OGRPoint p;
               TVec3d v;
               std::vector<TVec3d> pts;
               std::vector<TVec2f> texcoords;
               for ( int i = 0; i < poLR->getNumPoints(); ++i )
               {
                  poLR->getPoint( i, &p );
                  //v = TVec3d(p.getX(), p.getY(), p.getZ());
                  v = TVec3d( p.getX(), p.getY(), 10 );
                  pts.push_back( v );
               }
               tess.addContour( pts, texcoords );

               for ( int j = 0; j < poPG->getNumInteriorRings(); ++j )
               {
                  pts.clear();
                  poLR = poPG->getInteriorRing( j );
                  for ( int i = 0; i < poLR->getNumPoints(); ++i )
                  {
                     poLR->getPoint( i, &p );
                     v = TVec3d( p.getX(), p.getY(), 10 );
                     pts.push_back( v );
                  }
                  tess.addContour( pts, texcoords );
               }
               tess.compute();

               // feed data to osg
               for ( const TVec3d& v : tess.getVertices() )
               {
                  osg::Vec3d pt = osg::Vec3d( v.x, v.y, v.z ) - offset;
                  vertices->push_back( pt );
                  colors->push_back( osg::Vec3d( 0.0, 1.0, 0.0 ) );
               }
               for ( unsigned int id : tess.getIndices() )
               {
                  indices->push_back( id );
               }

               //geom->getOrCreateStateSet();

               geom->setVertexArray( vertices );
               geom->addPrimitiveSet( indices );

               geode->addDrawable( geom );
#else
               //TODO : Mieux gerer les indices, pour l'instance c'est sale
               osg::Geometry* geom = new osg::Geometry;
               osg::Vec3Array* vertices = new osg::Vec3Array;
               osg::DrawElementsUInt* indices = new osg::DrawElementsUInt( osg::PrimitiveSet::LINES, 0 );
               osg::Vec3Array* colors = new osg::Vec3Array;

               unsigned int cpt = 0;

               OGRPolygon* poPG = (OGRPolygon*)poGeometry;

               {
                  OGRLinearRing* poLR = poPG->getExteriorRing();

                  OGRPoint p;
                  osg::Vec3d v;
                  for ( int i = 0; i < poLR->getNumPoints() - 1; ++i )
                  {
                     poLR->getPoint( i, &p );
                     v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                     vertices->push_back( v );
                     poLR->getPoint( i + 1, &p );
                     v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                     vertices->push_back( v );
                     indices->push_back( cpt++ );
                     indices->push_back( cpt++ );
                  }

                  poLR->getPoint( poLR->getNumPoints() - 1, &p );
                  v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                  vertices->push_back( v );
                  poLR->getPoint( 0, &p );
                  v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                  vertices->push_back( v );
                  indices->push_back( cpt++ );
                  indices->push_back( cpt++ );
               }

               for ( unsigned int k = 0; k < (unsigned int)poPG->getNumInteriorRings(); k++ )
               {
                  OGRLinearRing* poLR = poPG->getInteriorRing( k );

                  OGRPoint p;
                  osg::Vec3d v;
                  for ( int i = 0; i < poLR->getNumPoints() - 1; ++i )
                  {
                     poLR->getPoint( i, &p );
                     v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                     vertices->push_back( v );
                     poLR->getPoint( i + 1, &p );
                     v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                     vertices->push_back( v );
                     indices->push_back( cpt++ );
                     indices->push_back( cpt++ );
                  }

                  poLR->getPoint( poLR->getNumPoints() - 1, &p );
                  v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                  vertices->push_back( v );
                  poLR->getPoint( 0, &p );
                  v = osg::Vec3d( p.getX(), p.getY(), p.getZ() ) - offset;
                  vertices->push_back( v );
                  indices->push_back( cpt++ );
                  indices->push_back( cpt++ );
               }

               colors->push_back( osg::Vec3d( 0.0, 1.0, 0.0 ) );

               geom->setVertexArray( vertices );
               geom->addPrimitiveSet( indices );
               geom->setColorArray( colors );
               geom->setColorBinding( osg::Geometry::BIND_OVERALL );

               geode->addDrawable( geom );
#endif

            }
            else if ( poGeometry != NULL && ( poGeometry->getGeometryType() == wkbPoint || poGeometry->getGeometryType() == wkbPoint25D ) )
            {
               osg::Geometry* geom = new osg::Geometry;
               osg::Vec3Array* vertices = new osg::Vec3Array;
               osg::DrawElementsUInt* indices = new osg::DrawElementsUInt( osg::PrimitiveSet::POINTS, 0 );
               osg::Vec3Array* colors = new osg::Vec3Array;

               OGRPoint* poP = (OGRPoint*)poGeometry;

               osg::Vec3d point( poP->getX(), poP->getY(), poP->getZ() );
               vertices->push_back( point - offset );
               indices->push_back( 0 );

               colors->push_back( osg::Vec3d( 243.0 / 255.0, 214.0 / 255.0, 23.0 / 255.0 ) );

               geom->setVertexArray( vertices );
               geom->addPrimitiveSet( indices );
               geom->setColorArray( colors );
               geom->setColorBinding( osg::Geometry::BIND_OVERALL );

               geode->addDrawable( geom );

               osg::Point* pointsize = new osg::Point( 3.0f );
               geode->getOrCreateStateSet()->setAttributeAndModes( pointsize );
            }
            else
            {
               printf( "%u %s no geometry\n", poGeometry->getGeometryType(), poGeometry->getGeometryName() );
            }
            OGRFeature::DestroyFeature( poFeature );
         }
      }

      return geode;
   }

   return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
