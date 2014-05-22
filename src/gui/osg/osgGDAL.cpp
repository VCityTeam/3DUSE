////////////////////////////////////////////////////////////////////////////////
#include "osgGDAL.hpp"
#include <osg/Geode>
#include <osg/Geometry>
#include "libcitygml/tesselator.hpp"
#include "core/application.hpp"
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> buildOsgGDAL(OGRDataSource* poDS)
{
    TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
    osg::Vec3d offset(offset_.x, offset_.y, offset_.z);

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

            Tesselator tess;

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
                        osg::Vec3d pt = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
                        vertices->push_back(pt);
                        indices->push_back(i);
                    }

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
                    geode->addDrawable(geom);
                }
                else if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
                {
                    // get height test
                    /*if(poFeature->GetFieldCount() > 0)
                    {
                        std::cout << "ID : " << poFeature->GetFieldAsString("ID") << std::endl;
                        std::cout << "HAUTEUR : " << poFeature->GetFieldAsDouble("HAUTEUR") << std::endl;
                        std::cout << "Z_MIN : " << poFeature->GetFieldAsDouble("Z_MIN") << std::endl;
                        std::cout << "Z_MAX : " << poFeature->GetFieldAsDouble("Z_MAX") << std::endl;
                    }*/

                    /*
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
                    /*/
                    osg::Geometry* geom = new osg::Geometry;
                    osg::Vec3Array* vertices = new osg::Vec3Array;
                    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES , 0);

                    OGRPolygon* poPG = (OGRPolygon*) poGeometry;
                    OGRLinearRing* poLR = poPG->getExteriorRing();

                    // count vertices
                    unsigned int verticesCount = poLR->getNumPoints();
                    for(int i=0; i<poPG->getNumInteriorRings(); ++i)
                    {
                        verticesCount += poPG->getInteriorRing(i)->getNumPoints();
                    }
                    tess.init(verticesCount, TVec3d(0.0, 0.0, 1.0));

                    // feed data
                    OGRPoint p;
                    TVec3d v;
                    std::vector<TVec3d> pts;
                    std::vector<TVec2f> texcoords;
                    for(int i=0; i<poLR->getNumPoints(); ++i)
                    {
                        poLR->getPoint(i, &p);
                        v = TVec3d(p.getX(), p.getY(), p.getZ());
                        pts.push_back(v);
                    }
                    tess.addContour(pts, texcoords);

                    for(int j=0; j<poPG->getNumInteriorRings(); ++j)
                    {
                        pts.clear();
                        poLR = poPG->getInteriorRing(j);
                        for(int i=0; i<poLR->getNumPoints(); ++i)
                        {
                            poLR->getPoint(i, &p);
                            v = TVec3d(p.getX(), p.getY(), p.getZ());
                            pts.push_back(v);
                        }
                        tess.addContour(pts, texcoords);
                    }
                    tess.compute();

                    // feed data to osg
                    for(const TVec3d& v : tess.getVertices())
                    {
                        osg::Vec3d pt = osg::Vec3d(v.x, v.y, v.z) - offset;
                        vertices->push_back(pt);
                    }
                    for(unsigned int id : tess.getIndices())
                    {
                        indices->push_back(id);
                    }

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
                    geode->addDrawable(geom);
                    //*/
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
