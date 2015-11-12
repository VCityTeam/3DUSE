// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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

#include "geos/geom/Polygon.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/CoordinateArraySequenceFactory.h"

//#define FILL_POLYGON

////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> buildOsgGDAL(OGRDataSource* poDS)
{
    TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
    osg::Vec3d offset(offset_.x, offset_.y, offset_.z);

    if(poDS)
    {
        printf("Load using Gdal / OGR\n");
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

        std::string name = poDS->GetName();
        name = name.substr(name.rfind('/')+1);

        geode->setName(name);

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
			printf("nb feature %d\n", poLayer->GetFeatureCount());
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

                    //geom->getOrCreateStateSet();

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
                    geode->addDrawable(geom);
                }
                else if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
                {
#ifdef FILL_POLYGON
                    osg::Geometry* geom = new osg::Geometry;
                    osg::Vec3Array* vertices = new osg::Vec3Array;
                    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES , 0);
					osg::Vec3Array* colors = new osg::Vec3Array;

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
                        //v = TVec3d(p.getX(), p.getY(), p.getZ());
                        v = TVec3d(p.getX(), p.getY(), 10);
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
                            v = TVec3d(p.getX(), p.getY(), 10);
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
                        colors->push_back(osg::Vec3d(0.0,1.0,0.0));
                    }
                    for(unsigned int id : tess.getIndices())
                    {
                        indices->push_back(id);
                    }

                    //geom->getOrCreateStateSet();

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);

                    geode->addDrawable(geom);
#else
					//TODO : Mieux gérer les indices, pour l'instance c'est sale
					osg::Geometry* geom = new osg::Geometry;
                    osg::Vec3Array* vertices = new osg::Vec3Array;
                    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES , 0);
					osg::Vec3Array* colors = new osg::Vec3Array;

					unsigned int cpt = 0;

                    OGRPolygon* poPG = (OGRPolygon*) poGeometry;

					{
						OGRLinearRing* poLR = poPG->getExteriorRing();

						OGRPoint p;
						osg::Vec3d v;
						for(int i=0; i<poLR->getNumPoints()-1; ++i)
						{
							poLR->getPoint(i, &p);
							v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
							vertices->push_back(v);
							poLR->getPoint(i+1, &p);
							v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
							vertices->push_back(v);
							indices->push_back(cpt++);
							indices->push_back(cpt++);
						}

						poLR->getPoint(poLR->getNumPoints()-1, &p);
						v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
						vertices->push_back(v);
						poLR->getPoint(0, &p);
						v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
						vertices->push_back(v);
						indices->push_back(cpt++);
						indices->push_back(cpt++);
					}

					for(unsigned int k = 0; k < (unsigned int)poPG->getNumInteriorRings(); k++)
					{
						OGRLinearRing* poLR = poPG->getInteriorRing(k);

						OGRPoint p;
						osg::Vec3d v;
						for(int i=0; i<poLR->getNumPoints()-1; ++i)
						{
							poLR->getPoint(i, &p);
							v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
							vertices->push_back(v);
							poLR->getPoint(i+1, &p);
							v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
							vertices->push_back(v);
							indices->push_back(cpt++);
							indices->push_back(cpt++);
						}

						poLR->getPoint(poLR->getNumPoints()-1, &p);
						v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
						vertices->push_back(v);
						poLR->getPoint(0, &p);
						v = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - offset;
						vertices->push_back(v);
						indices->push_back(cpt++);
						indices->push_back(cpt++);
					}

					colors->push_back(osg::Vec3d(0.0,1.0,0.0));

					geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
					geom->setColorArray(colors);
					geom->setColorBinding(osg::Geometry::BIND_OVERALL);

                    geode->addDrawable(geom);
#endif

                }
				else if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPoint || poGeometry->getGeometryType() == wkbPoint25D))
				{
					osg::Geometry* geom = new osg::Geometry;
                    osg::Vec3Array* vertices = new osg::Vec3Array;
                    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS, 0);
					osg::Vec3Array* colors = new osg::Vec3Array;

                    OGRPoint* poP = (OGRPoint*) poGeometry;
                    
					osg::Vec3d point(poP->getX(),poP->getY(),poP->getZ());
					vertices->push_back(point - offset);
                    indices->push_back(0);

					colors->push_back(osg::Vec3d(243.0/255.0, 214.0/255.0, 23.0/255.0));

                    geom->setVertexArray(vertices);
                    geom->addPrimitiveSet(indices);
					geom->setColorArray(colors);
					geom->setColorBinding(osg::Geometry::BIND_OVERALL);

                    geode->addDrawable(geom);

					osg::Point* pointsize = new osg::Point(3.0f); 
					geode->getOrCreateStateSet()->setAttributeAndModes(pointsize); 
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

void buildGeosShape(OGRDataSource* poDS, geos::geom::Geometry ** ShapeGeo, std::vector<std::pair<double, double>> * Hauteurs, std::vector<BatimentShape> * InfoBatiments)
{
	if(!poDS)
    {
		std::cout << "Erreur chargement fichier .shp \n";
		return;
	}

	const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
    const geos::geom::CoordinateSequenceFactory * coordFactory = geos::geom::CoordinateArraySequenceFactory::instance();
    std::vector<geos::geom::Geometry*>* Polys = new std::vector<geos::geom::Geometry*>();
	geos::geom::LinearRing *shell;
	geos::geom::Polygon* P;
	
	OGRLayer *poLayer;
	//poLayer->FindFieldIndex("a", 1);

    int nbLayers = poDS->GetLayerCount();
    if(nbLayers > 0)
    {
        poLayer = poDS->GetLayer(0);
		OGRFeature *poFeature;
        poLayer->ResetReading();

		//Ajouter un attribut
		//if(poLayer->FindFieldIndex("Horaire", 1) == -1)
		//	poLayer->CreateField(new OGRFieldDefn("Horaire", OGRFieldType::OFTInteger));
        while( (poFeature = poLayer->GetNextFeature()) != NULL )
        {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();

			//poFeature->SetField("Horaire", 55);
			//std::cout << poFeature->GetFieldAsInteger("Horaire") << std::endl;
			//int a;
			//std::cin >> a;

			//std::cout << poGeometry->getGeometryName() << std::endl;

			if(poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
            {
                OGRPolygon* poPG = (OGRPolygon*) poGeometry;
                OGRLinearRing* poLR = poPG->getExteriorRing();

                int nbPoints = poLR->getNumPoints();

				if(nbPoints > 3)
				{
					std::size_t size = 0;
					std::size_t dimension=2;
					geos::geom::CoordinateSequence* temp = static_cast<geos::geom::CoordinateArraySequence*>(coordFactory->create(size, dimension));
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

                        temp->add(geos::geom::Coordinate(p.getX(), p.getY()));
					}

					shell=factory->createLinearRing(temp);
					P = factory->createPolygon(shell, NULL/*Holes*/); //Les bâtiments du cadastre sont récupérés sans les cours intérieures. Mettre Holes à la place de NULL pour les avoir.
					if(P->isValid()/* && P->getArea() > 10*/)
					{
						if(poFeature->GetFieldIndex("ID") != -1)
							InfoBatiments->push_back(BatimentShape(P, poFeature->GetFieldAsString("ID")));
						else if(poFeature->GetFieldIndex("OBJECTID") != -1)
							InfoBatiments->push_back(BatimentShape(P, poFeature->GetFieldAsString("OBJECTID")));
						else
							InfoBatiments->push_back(BatimentShape(P, "ID NULL"));

                        Polys->push_back(P);
						double H = 20;
						double Zmin = 0;
						if(poFeature->GetFieldIndex("HAUTEUR") != -1)
							H = poFeature->GetFieldAsDouble("HAUTEUR");
						if(poFeature->GetFieldIndex("Z_MIN") != -1)
							Zmin = poFeature->GetFieldAsDouble("Z_MIN");

						if((H == 0 || Zmin > 1000) && Hauteurs->size()>1)
						{
							H = Hauteurs->at(Hauteurs->size() - 1).first;
							Zmin = Hauteurs->at(Hauteurs->size() - 1).second;
						}

						std::pair<double, double> PairTemp(H, Zmin);
						Hauteurs->push_back(PairTemp);
					}
				}
			}
			OGRFeature::DestroyFeature( poFeature );
		}
		geos::geom::MultiPolygon *MP = factory->createMultiPolygon(Polys);
			
		*ShapeGeo = MP;
	}
}
////////////////////////////////////////////////////////////////////////////////
