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

/**
    * @brief Sauvegarde une image 8bits dans un fichier pgm
    */
void SaveImage(std::string name, int* Im, int width, int height)
{
	FILE *out;
	errno_t err;
	name = name + ".pgm";
	char* nameC = (char*)name.c_str();
	err = fopen_s(&out, nameC,"w"); 

	fprintf(out,"P2\n%d %d\1\n", width, height); 

	for(int h = height - 1; h >= 0; h--)
	{
		fprintf(out, "\n");
		for(int w = 0; w < width; w++)
		{
			int pos = w + h*width;

			fprintf(out,"%d ",Im[pos]);
		}
	}

	fclose(out); 

	std::cout<<"Image " << name << " creee !"<<std::endl;
}

/**
    * @brief Trace la ligne entre deux points de coordonnées (x1,y1) et (x2,y2) par Bresenham (http://fr.wikipedia.org/wiki/Algorithme_de_trac%C3%A9_de_segment_de_Bresenham)
    */
std::vector<std::pair<int,int>> TracerSegment(int x1, int y1, int x2, int y2)
{
	std::vector<std::pair<int,int>> Points;
	int dx, dy;

	if((dx = x2 - x1) != 0)
	{
		if(dx > 0)
		{
			if((dy = y2 - y1) != 0)
			{
				if(dy > 0)
				{
					if(dx >= dy)
					{
						int e = dx;
						dx = e * 2;
						dy = dy * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((x1 = x1 + 1) == x2)
								break;
							if((e = e - dy) < 0)
							{
								y1 = y1 + 1;
								e = e + dx;
							}
						}
					}
					else
					{
						int e = dy;
						dy = e * 2;
						dx = dx * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((y1 = y1 + 1) == y2)
								break;
							if((e = e - dx) < 0)
							{
								x1 = x1 + 1;
								e = e + dy;
							}
						}
					}
				}
				else
				{
					if(dx >= -dy)
					{
						int e = dx;
						dx = e * 2;
						dy = dy * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((x1 = x1 + 1) == x2)
								break;
							if((e = e + dy) < 0)
							{
								y1 = y1 - 1;
								e = e + dx;
							}
						}
					}
					else
					{
						int e = dy;
						dy = e * 2;
						dx = dx * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((y1 = y1 - 1) == y2)
								break;
							if((e = e + dx) > 0)
							{
								x1 = x1 + 1;
								e = e + dy;
							}
						}
					}
				}
			}
			else
			{
				do
				{
					Points.push_back(std::make_pair(x1, y1));
				}while((x1 = x1 + 1) != x2);
			}
		}
		else
		{
			if((dy = y2 - y1) != 0)
			{
				if(dy > 0)
				{
					if(-dx >= dy)
					{
						int e = dx;
						dx = e * 2;
						dy = dy * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((x1 = x1 - 1) == x2)
								break;
							if((e = e + dy) >= 0)
							{
								y1 = y1 + 1;
								e = e + dx;
							}
						}
					}
					else
					{
						int e = dy;
						dy = e * 2;
						dx = dx * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((y1 = y1 + 1) == y2)
								break;
							if((e = e + dx) <= 0)
							{
								x1 = x1 - 1;
								e = e + dy;
							}
						}
					}
				}
				else
				{
					if(dx <= dy)
					{
						int e = dx;
						dx = e * 2;
						dy = dy * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((x1 = x1 - 1) == x2)
								break;
							if((e = e - dy) >= 0)
							{
								y1 = y1 - 1;
								e = e + dx;
							}
						}
					}
					else
					{
						int e = dy;
						dy = e * 2;
						dx = dx * 2;
						while(true)
						{
							Points.push_back(std::make_pair(x1, y1));
							if((y1 = y1 - 1) == y2)
								break;
							if((e = e - dx) >= 0)
							{
								x1 = x1 - 1;
								e = e + dy;
							}
						}
					}
				}
			}
			else
			{
				do
				{
					Points.push_back(std::make_pair(x1, y1));
				}while((x1 = x1 - 1) != x2);
			}
		}
	}
	else
	{
		if((dy = y2 - y1) != 0)
		{
			if(dy > 0)
			{
				do
				{
					Points.push_back(std::make_pair(x1, y1));
				}while((y1 = y1 + 1) != y2);
			}
			else
			{
				do
				{
					Points.push_back(std::make_pair(x1, y1));
				}while((y1 = y1 - 1) != y2);
			}
		}
	}

	return Points;
}

/**
    * @brief Sauvegarde la geometry dans un fichier image
    */
void SaveGeometry(std::string name, const geos::geom::Geometry* G)
{	
	int Scale = 1;
	const geos::geom::CoordinateSequence *coord;

	coord = G->getCoordinates();

	int Xmin = -1, Ymin = -1, Xmax = 0, Ymax = 0;

	for(int i = 0; i < coord->size(); i++)
	{
		int x = coord->getAt(i).x * Scale;
		int y = coord->getAt(i).y * Scale;

		if(Xmin == -1 || x < Xmin)
			Xmin = x;
		if(Ymin == -1 || y < Ymin)
			Ymin = y;
		if(x > Xmax)
			Xmax = x;
		if(y > Ymax)
			Ymax = y;
	}

	Xmin--;
	Ymin--;

	int width = Xmax - Xmin + Scale;
	int height = Ymax - Ymin + Scale;

	int* Im = new int[width * height];
	memset(Im, 1, width*height*sizeof(int));
	int* ImHoles = new int[width * height];
	memset(ImHoles, 1, width*height*sizeof(int));
	bool Holes = false; //Passe à true en présence d'holes

	int NbGeo = G->getNumGeometries();

	for(int i = 0; i < NbGeo; i++)
	{
		const geos::geom::Geometry *Geo = G->getGeometryN(i);

		const geos::geom::Polygon *p = dynamic_cast<const geos::geom::Polygon*>(Geo);
		if(p)
		{
			coord = p->getExteriorRing()->getCoordinates();
			for(int j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
			{
				int x1 = Scale*coord->getAt(j).x - Xmin;
				int y1 = Scale*coord->getAt(j).y - Ymin;

				int	x2 = Scale*coord->getAt(j+1).x - Xmin;
				int	y2 = Scale*coord->getAt(j+1).y - Ymin;
					
				std::vector<std::pair<int,int>> Points = TracerSegment(x1, y1, x2, y2);

				for(std::vector<std::pair<int,int>>::iterator it = Points.begin(); it != Points.end(); ++it)
				{
					int pos = it->first + it->second * width;
					if(pos >= width * height || pos < 0)
						std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
					else
					{
						Im[pos] = 0;
						ImHoles[pos] = 0;
					}
				}
			}

			for(int k = 0; k < p->getNumInteriorRing(); k++) //On parcourt les holes du polygon
			{
				Holes = true;
				coord = p->getInteriorRingN(k)->getCoordinates();
				for(int j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
				{
					int x1 = Scale*coord->getAt(j).x - Xmin;
					int y1 = Scale*coord->getAt(j).y - Ymin;

					int	x2 = Scale*coord->getAt(j+1).x - Xmin;
					int	y2 = Scale*coord->getAt(j+1).y - Ymin;
						
					std::vector<std::pair<int,int>> Points = TracerSegment(x1, y1, x2, y2);
					//Points.push_back(std::make_pair(x1, y1));

					for(std::vector<std::pair<int,int>>::iterator it = Points.begin(); it != Points.end(); ++it)
					{
						int pos = it->first + it->second * width;
						if(pos >= width * height || pos < 0)
							std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
						else
							ImHoles[pos] = 0;
					}
				}
			}
		}
		else
		{
			std::cout << "Geometry n'est pas un polygon. \n";
			coord = Geo->getCoordinates();

			for(int j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
			{
				int x1 = Scale*coord->getAt(j).x - Xmin;
				int y1 = Scale*coord->getAt(j).y - Ymin;

				int	x2 = Scale*coord->getAt(j+1).x - Xmin;
				int	y2 = Scale*coord->getAt(j+1).y - Ymin;
					
				std::vector<std::pair<int,int>> Points = TracerSegment(x1, y1, x2, y2);

				for(std::vector<std::pair<int,int>>::iterator it = Points.begin(); it != Points.end(); ++it)
				{
					int pos = it->first + it->second * width;
					if(pos >= width * height || pos < 0)
						std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
					else
						Im[pos] = 0;
				}
			}
		}
	}

	if(Holes)
		SaveImage(name + "withHoles", ImHoles, width, height);
	else
		SaveImage(name, Im, width, height);
		
	delete [] Im;
	delete [] ImHoles;
}

osg::ref_ptr<osg::Geode> buildOsgGDAL(OGRDataSource* poDS, geos::geom::Geometry ** ShapeGeo)
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
					std::vector<geos::geom::Geometry*> * Holes = new std::vector<geos::geom::Geometry*>;//

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
					}
					
                    for(int i=0; i<nbPoints; ++i)//Pour récupérer les points de l'exterior ring
                    {
                        OGRPoint p;
                        poLR->getPoint(i, &p);
                        osg::Vec3d pt = osg::Vec3d(p.getX(), p.getY(), p.getZ()) - osg::Vec3d(643000.0, 6857000.0, 50.0);
                        vertices->push_back(pt);
                        indices->push_back(i);

						temp.add(geos::geom::Coordinate(p.getX(), p.getY()));//
                    }
					if(temp.size() > 3)//
					{
						shell=factory->createLinearRing(temp);
						P = factory->createPolygon(shell, Holes);
						if(P->isValid())
							Polys.push_back(P);
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

			//SaveGeometry(poDS->GetName(), MP);

			*ShapeGeo = MP;
        }

        return geode;
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
