////////////////////////////////////////////////////////////////////////////////
#include "osgGDAL.hpp"
#include <osg/Geode>
#include <osg/Geometry>
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> buildOsgGDAL(OGRDataSource* poDS)
{
    if(poDS)
    {
        printf("Load using Gdal / OGR\n");
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

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
                    for(int i=0; i<nbPoints; ++i)
                    {
                        OGRPoint p;
                        poLR->getPoint(i, &p);
                        //printf( "%f, %f; %f\n", p.getX(), p.getY(), p.getZ() );
                        osg::Vec3d pt = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - osg::Vec3d(643000.0, 6857000.0, 50.0);
                        vertices->push_back(pt);
                        indices->push_back(i);
                    }

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
                    geode->addDrawable(geom);
                }
                else
                {
                    printf( "%u %s no geometry\n", poGeometry->getGeometryType(), poGeometry->getGeometryName() );
                }
                //poFeature->get
                OGRFeature::DestroyFeature( poFeature );
                //poDS->get
            }
        }

        return geode;
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
