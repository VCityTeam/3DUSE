////////////////////////////////////////////////////////////////////////////////
#include "osgGDAL.hpp"
#include <osg/Geode>
#include <osg/Geometry>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <geos/geom/Polygon.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/Dimension.h>
#include <geos/geom/Envelope.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Point.h>
#include <geos/geom/PrecisionModel.h>
#include <geos/io/WKTReader.h>
#include <geos/util/IllegalArgumentException.h>
#include <geos/operation/union/CascadedPolygonUnion.h>
#include <geos/simplify/DouglasPeuckerSimplifier.h>
#include <geos/simplify/TopologyPreservingSimplifier.h>

////////////////////////////////////////////////////////////////////////////////

osg::ref_ptr<osg::Geode> buildOsgGDAL(OGRDataSource* poDS, geos::geom::Geometry ** ShapeGeo, std::vector<std::pair<double, double>> * Hauteurs)//Hauteurs : Liste les hauteurs et Zmin des polygons de ShapeGeo
{
    if(poDS)
    {
        printf("Load using Gdal / OGR\n");
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();//
		std::vector<geos::geom::Geometry*> Polys;//
		geos::geom::LinearRing *shell;//
		geos::geom::Polygon* P;//

        OGRLayer *poLayer;
        int nbLayers = poDS->GetLayerCount();
        if(nbLayers > 0)
        {
            printf("nb layer %d\n", nbLayers);
            poLayer = poDS->GetLayer(0);
            printf("layer %s. Type : %d\n", poLayer->GetName(), poLayer->GetGeomType());
            //poLayer = poDS->GetLayerByName( "point" );

            OGRFeature *poFeature;
            poLayer->ResetReading();
            while( (poFeature = poLayer->GetNextFeature()) != NULL )
            {      
                OGRGeometry* poGeometry = poFeature->GetGeometryRef();
                //if(poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString25D)
                if(poGeometry != NULL && poGeometry->getGeometryType() == wkbLineString25D)
                {
                    osg::Geometry* geom = new osg::Geometry;
                    osg::Vec3Array* vertices = new osg::Vec3Array;
                    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_STRIP, 0);

                    OGRLineString* poLS = (OGRLineString*) poGeometry;
                    int nbPoints = poLS->getNumPoints();
                    for(int i=0; i<nbPoints; ++i)
                    {
                        OGRPoint p;
                        poLS->getPoint(i, &p);
                        //printf( "%f, %f; %f\n", p.getX(), p.getY(), p.getZ() );
                        osg::Vec3d pt = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - osg::Vec3d(643000.0, 6857000.0, 0);
                        vertices->push_back(pt);
                        indices->push_back(i);
                    }

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
                    geode->addDrawable(geom);
                }
                else if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
                {
                    osg::Geometry* geom = new osg::Geometry;
                    osg::Vec3Array* vertices = new osg::Vec3Array;
                    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);

                    OGRPolygon* poPG = (OGRPolygon*) poGeometry;
                    OGRLinearRing* poLR = poPG->getExteriorRing();
                    int nbPoints = poLR->getNumPoints();

					geos::geom::CoordinateArraySequence temp;//
					/*std::vector<geos::geom::Geometry*> * Holes = new std::vector<geos::geom::Geometry*>;//

					for(int i = 0; i < poPG->getNumInteriorRings(); i++)// //Pour récupérer les holes des polygons
					{
						OGRLinearRing* Ring = poPG->getInteriorRing(i);

						for(int j = 0; j < Ring->getNumPoints(); j++)
						{
							OGRPoint p;
							Ring->getPoint(j, &p);
							temp.add(geos::geom::Coordinate(p.getX(), p.getY()));
						}
						if(temp.size() > 3)
							Holes->push_back((geos::geom::Geometry*)factory->createLinearRing(temp));
						temp.clear();
					}*/
					
                    for(int i=0; i<nbPoints; ++i)//Pour récupérer les points de l'exterior ring
                    {
                        OGRPoint p;
                        poLR->getPoint(i, &p);
                        osg::Vec3d pt = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - osg::Vec3d(643000.0, 6857000.0, 50.0); // osg::Vec3d(1840000.0, 5170000.0, 50.0);
                        vertices->push_back(pt);
                        indices->push_back(i);

						temp.add(geos::geom::Coordinate(p.getX()/* - 1840000.0*/, p.getY()/* - 5170000.0*/));//
                    }
					if(temp.size() > 3)//
					{
						shell=factory->createLinearRing(temp);
						P = factory->createPolygon(shell, NULL/*Holes*/); //Les bâtiments du cadastre sont récupérés sans les cours intérieures. Mettre Holes à la place de NULL pour les avoir.
						if(P->isValid()/* && P->getArea() > 10*/)
						{
							Polys.push_back(P);
							double H = poFeature->GetFieldAsDouble("HAUTEUR");
							double Zmin = poFeature->GetFieldAsDouble("Z_MIN");

							if(H == 0 || Zmin > 1000)
							{
								Zmin = 0;
								H = 20;
							}

							std::pair<double, double> PairTemp(H, Zmin);
							Hauteurs->push_back(PairTemp);
						}
					}

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
                    geode->addDrawable(geom);
                }
                else
                {
                    printf( "%u %s no geometry\n", poGeometry->getGeometryType(), poGeometry->getGeometryName() );
                }
                OGRFeature::DestroyFeature( poFeature );
            }

			geos::geom::MultiPolygon *MP = factory->createMultiPolygon(Polys);
			
			*ShapeGeo = MP;
        }

        return geode;
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
