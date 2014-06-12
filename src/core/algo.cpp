
////////////////////////////////////////////////////////////////////////////////
#include "algo.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "application.hpp"
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
#include "gui/moc/mainWindow.hpp"

#include "export/exportCityGML.hpp"
#include "geos/geom/Polygon.h"
#include "geos/geom/Coordinate.h"
#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Dimension.h"
#include "geos/geom/Envelope.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/LinearRing.h"
#include "geos/geom/Point.h"
#include "geos/geom/PrecisionModel.h"
#include "geos/io/WKTReader.h"
#include "geos/util/IllegalArgumentException.h"
#include "geos/operation/union/CascadedPolygonUnion.h"
#include "geos/simplify/DouglasPeuckerSimplifier.h"
#include "geos/simplify/TopologyPreservingSimplifier.h"
#include "geos/operation/distance/DistanceOp.h"
#include "geos/operation/buffer/BufferOp.h"
#include "geos/geom/util/LinearComponentExtracter.h"
#include "geos/noding/GeometryNoder.h"
#include "geos/operation/polygonize/Polygonizer.h"
#include "geos/triangulate/DelaunayTriangulationBuilder.h"
#include "geos/algorithm/LineIntersector.h"
////////////////////////////////////////////////////////////////////////////////
#ifdef __unix
#include <cerrno>
typedef int errno_t;
errno_t fopen_s(FILE **f, const char *name, const char *mode) {
    errno_t ret = 0;
    assert(f);
    *f = fopen(name, mode);
    /* Can't be sure about 1-to-1 mapping of errno and MS' errno_t */
    if (!*f)
        ret = errno;
    return ret;
}
#endif // __unix
////////////////////////////////////////////////////////////////////////////////
typedef std::pair<double,double> Point;
typedef std::vector<Point> Polygon2D;
typedef std::set<Polygon2D> PolySet;

////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
	double Scale = 10; //Définit le zoom des images sauvegardées, avec 1 = 1 mètre/pixel.

    /**
     * @brief projete les toits du CityObject selectioné sur le plan (xy)
     * @param obj CityObject séléctioné
     * @param roofProj un set de Polygon, le résultat de la projection
     */
    void projectRoof(citygml::CityObject* obj, PolySet &roofProj, double * heightmax, double * heightmin)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
        if(obj->getType() == citygml::COT_RoofSurface) //Si surface de toit : COT_RoofSurface COT_WallSurface
		{
            std::vector<citygml::Geometry*> geoms = obj->getGeometries();
            std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
            for(; itGeom != geoms.end(); ++itGeom) //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)
			{
                std::vector<citygml::Polygon*> polys = (*itGeom)->getPolygons();
                std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
                for(; itPoly != polys.end(); ++itPoly) //Pour chaque polygone
				{
                    Polygon2D poly;
                    citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                    const std::vector<TVec3d> vertices = ring->getVertices();
                    std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
                    for(; itVertices != vertices.end(); ++itVertices)//pour Chaque sommet
					{
                        TVec3d point = *itVertices;
                        poly.push_back(std::make_pair(point.x - offset_.x, point.y - offset_.y)); //on récupere le point
						if(point.z > *heightmax)
							*heightmax = point.z;

                       //std::cout << " (x,y) = (" << point.x<< "," << point.y<< ")" << std::endl;
                    }
                    roofProj.insert(poly); // on récupere le polygone
                }
            }
        }
		else if(obj->getType() == citygml::COT_WallSurface)
		{
			std::vector<citygml::Geometry*> geoms = obj->getGeometries();
            std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
            for(; itGeom != geoms.end(); ++itGeom) //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)
			{
                std::vector<citygml::Polygon*> polys = (*itGeom)->getPolygons();
                std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
                for(; itPoly != polys.end(); ++itPoly) //Pour chaque polygone
				{
                    citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                    const std::vector<TVec3d> vertices = ring->getVertices();
                    std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
                    for(; itVertices != vertices.end(); ++itVertices)//pour Chaque sommet
					{
                        TVec3d point = *itVertices;
						if(point.z < *heightmin || *heightmin == -1)
							*heightmin = point.z;
                    }
                }
            }
		}
        citygml::CityObjects cityObjects = obj->getChildren();
        citygml::CityObjects::iterator itObj = cityObjects.begin();
        for(; itObj != cityObjects.end(); ++itObj)
		{
            projectRoof(*itObj,roofProj, heightmax, heightmin);
        }

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
     * @brief Sauvegarde une image binaire dans un fichier pgm
     */
	void SaveImage(std::string name, int* Im, int width, int height)
	{
		FILE *out;
		errno_t err;
		name = name + ".pgm";
		char* nameC = (char*)name.c_str();
		err = fopen_s(&out, nameC,"w"); 
		fprintf(out,"P2\n%d %d\n1\n", width, height); 
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
     * @brief Sauvegarde une image RGB dans un fichier ppm
     */
	void SaveImageRGB(std::string name, int* ImR, int* ImG, int* ImB, int width, int height)
	{
		FILE *out;
		errno_t err;
		name = name + ".ppm";
		char* nameC = (char*)name.c_str();
		err = fopen_s(&out, nameC,"w"); 
		fprintf(out,"P3\n%d %d\n255", width, height); 
		for(int h = height - 1; h >= 0; h--)
		{
			fprintf(out, "\n");
			for(int w = 0; w < width; w++)
			{
				int pos = w + h*width;
				fprintf(out,"%d %d %d ", (unsigned char)(255*ImR[pos]), (unsigned char)(255*ImG[pos]), (unsigned char)(255*ImB[pos]));//(unsigned char)
			}
		}

		fclose(out); 

		std::cout<<"Image " << name << " creee !"<<std::endl;
	}
	
	void SaveRecursiveGeometry(const geos::geom::Geometry * Geo, int* Im, int height, int width, int Xmin, int Ymin)
	{
		if(Geo->getNumGeometries() > 1)
		{
			for(int i = 0; i < Geo->getNumGeometries(); ++i)
				SaveRecursiveGeometry(Geo->getGeometryN(i), Im, height, width, Xmin, Ymin);
		}
		else
		{
			const geos::geom::CoordinateSequence *coord;
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
							Im[pos] = 0;
					}
				}

				for(int k = 0; k < p->getNumInteriorRing(); k++) //On parcourt les holes du polygon
				{
					delete coord;
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
								Im[pos] = 0;
						}
					}
				}
			}
			else
			{
				coord = Geo->getCoordinates();
				if(coord->size() != 0)
				{
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
			delete coord;
		}
	}

	/**
     * @brief Sauvegarde la geometry dans un fichier image
     */
	void SaveGeometry(std::string name, const geos::geom::Geometry* G)
	{	
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

		Xmin --;
		Ymin --;

		delete coord;

		//std::cout << Xmin << ";" << Xmax << "  " << Ymin << ";" << Ymax << std::endl;
		
		int width = Xmax - Xmin + Scale;
		int height = Ymax - Ymin + Scale;

		int* Im = new int[width * height];
		memset(Im, 1, width*height*sizeof(int));

		int NbGeo = G->getNumGeometries();

		for(int i = 0; i < NbGeo; i++)
		{
			const geos::geom::Geometry *Geo = G->getGeometryN(i);

			SaveRecursiveGeometry(Geo, Im, height, width, Xmin, Ymin);
		}

		SaveImage(name, Im, width, height);
		
		delete [] Im;
	}

	/**
     * @brief Sauvegarde 3 geometry dans un même fichier image dans les trois canaux RGB
     */
	void Save3GeometryRGB(std::string name, const geos::geom::Geometry* G1,  const geos::geom::Geometry* G2,  const geos::geom::Geometry* G3)
	{	
		const geos::geom::CoordinateSequence *coord;

		int Xmin = -1, Ymin = -1, Xmax = 0, Ymax = 0;

		for(int g = 0; g < 3; g ++)
		{
			if(g==0)
				coord = G1->getCoordinates();
			else if(g==1)
				coord = G2->getCoordinates();
			else
				coord = G3->getCoordinates();

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
		}		
		delete coord;

		Xmin --;
		Ymin --;

		//std::cout << Xmin << ";" << Xmax << "  " << Ymin << ";" << Ymax << std::endl;
		int width = Xmax - Xmin + Scale;
		int height = Ymax - Ymin + Scale;

		int* Im = new int[width * height];
		int* ImR = new int[width * height];
		int* ImG = new int[width * height];
		int* ImB = new int[width * height];

		for(int g = 0; g < 3; g ++)
		{			
			memset(Im, 1, width*height*sizeof(int));
			//////// Pour avoir un canal vide (utile pour avoir une taille d'image fixe)
			/*if(g==1)
			{
				memcpy(ImG, Im, width*height*sizeof(int));
				continue;
			}*/
			////////
			int NbGeo;

			if(g==0)
				NbGeo = G1->getNumGeometries();
			else if(g==1)
				NbGeo = G2->getNumGeometries();
			else
				NbGeo = G3->getNumGeometries();			

			for(int i = 0; i < NbGeo; i++)
			{
				const geos::geom::Geometry *Geo;

				if(g==0)
					Geo = G1->getGeometryN(i);
				else if(g==1)
					Geo = G2->getGeometryN(i);
				else
					Geo = G3->getGeometryN(i);

				SaveRecursiveGeometry(Geo, Im, height, width, Xmin, Ymin);
			}
			if(g==0)
			{
				memcpy(ImR, Im, width*height*sizeof(int));
			}
			else if(g==1)
			{
				memcpy(ImG, Im, width*height*sizeof(int));
			}
			else
			{
				memcpy(ImB, Im, width*height*sizeof(int));
			}
		}
				
		SaveImageRGB(name, ImR, ImG, ImB, width, height);		
		
		delete [] Im;
		delete [] ImR;
		delete [] ImG;
		delete [] ImB;
	}

	/**
     * @brief Récupère l'enveloppe d'une geometry
     */
	geos::geom::Geometry * GetEnveloppe(geos::geom::MultiPolygon * MP)//Attention, le cas où les polygon à unir ont déjà des trous fonctionne peut être mal
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		//std::cout << "Mise en place de l'union des polygons" << std::endl;

		geos::geom::Geometry* ResUnion = factory->createEmptyGeometry();//MP->getGeometryN(0)->clone();
				
		for(int i = 0; i < MP->getNumGeometries(); i++)	//On parcourt tous les polygons que l'on veut unir
		{
			try	//On vérifie qu'il n'y ait pas d'exceptions faisant planter le logiciel
			{
				geos::geom::Geometry* tmp = ResUnion;
				ResUnion = ResUnion->Union(MP->getGeometryN(i)); //On fait l'union avant de la vérifier
				delete tmp;
				
				std::vector<geos::geom::Geometry*> Polys; //Vecteur contenant les différents polygones de l'union au fur et à mesure
												
				for(int j = 0; j < ResUnion->getNumGeometries(); j++) //L'union peut être constitué de plusieurs polygons disjoints
				{
					geos::geom::CoordinateArraySequence tempcoord;
					const geos::geom::CoordinateSequence *coordGeo = ResUnion->getGeometryN(j)->getCoordinates(); //On récupère la liste des points de la géométrie courante

					double FirstPoint[2];
					FirstPoint[0] = -1;

					bool isHole = false;
					geos::geom::LinearRing * shell;
					std::vector<geos::geom::Geometry*> * Holes = new std::vector<geos::geom::Geometry*>; //Vecteur contenant tous les polygones à l'intérieur du premier, qui sont donc considérés comme des trous

					for(int k = 0; k < coordGeo->size(); k++) //On parcourt tous les points pour retrouver ceux qui apparaissent deux fois et qui définissent un polygon qu'il faut extraire
					{
						double x = coordGeo->getAt(k).x;
						double y = coordGeo->getAt(k).y;

						tempcoord.add(geos::geom::Coordinate(x, y));
												
						if(FirstPoint[0] == -1)
						{
							FirstPoint[0] = x;
							FirstPoint[1] = y;
						}
						else if(x == FirstPoint[0] && y == FirstPoint[1])
						{
							if(!isHole)
							{
								shell = factory->createLinearRing(tempcoord);
								FirstPoint[0] = -1;
								isHole = true;
								tempcoord.clear();
								
								//break;//////////////////////// A ENLEVER POUR AVOIR LES TROUS DANS LES POLYGONS
							}
							else
							{
								geos::geom::LinearRing* Hole = factory->createLinearRing(tempcoord);
								
								if(tempcoord.size() > 3 && factory->createPolygon(Hole, NULL)->getArea() > 1)
									Holes->push_back((geos::geom::Geometry*)Hole);
								FirstPoint[0] = -1;
								tempcoord.clear();
							}
						}
					}
					delete coordGeo;
					geos::geom::Polygon *P;

					if(Holes->size() == 0)
						P = factory->createPolygon(shell, NULL);
					else
						P = factory->createPolygon(shell, Holes);

					//Holes->clear();
					//delete Holes;

					Polys.push_back(P);
				}
				delete ResUnion;
				ResUnion = factory->createMultiPolygon(Polys);
			}
			catch(std::exception& e)
			{
				std::cout << e.what() << '\n';
			}
		}

		return ResUnion;
	}

	/**
     * @brief Convertit les données citygml de projection au sol en multipolygon pour GEOS
     */
	geos::geom::MultiPolygon * ConvertToGeos(PolySet &roofPoints)
	{
		//std::cout << "Debut de ConvertToGeos" << std::endl;
		
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		std::vector<geos::geom::Geometry*> Polys;
		geos::geom::LinearRing *shell;
		geos::geom::Polygon* P;
		
		for(PolySet::const_iterator poly = roofPoints.begin(); poly != roofPoints.end(); ++poly)
		{
			geos::geom::CoordinateArraySequence temp;
			
			if(poly->size() < 3)
			{
				//std::cout << "Mauvais polygon detecte, poly->size() = " << poly->size() << std::endl;
				continue;
			}

			for(Polygon2D::const_iterator point = poly->begin(); point != poly->end(); ++point)
			{
				temp.add(geos::geom::Coordinate(point->first, point->second));
			}
			temp.add(geos::geom::Coordinate(poly->begin()->first, poly->begin()->second));

			shell=factory->createLinearRing(temp);

			P = factory->createPolygon(shell, NULL);
			
			Polys.push_back(P); 
		}
		//std::cout << "Creation du Multipolygon ..." << std::endl;

		geos::geom::MultiPolygon *MP = factory->createMultiPolygon(Polys);

		//std::cout << "MultiPolygon cree, il contient : "<< MP->getNumGeometries() << " Polygons." << std::endl;

		if(MP->getNumGeometries() == 0)
			return NULL;

		return MP;
	}
	
	/**
     * @brief Teste la geometry Geo avec Geo1 et Geo2 pour déterminer de laquelle est la plus proche. Retourne 1 pour Geo1 et 2 pour Geo2
     */
	int GetNearestGeo(const geos::geom::Geometry * Geo, const geos::geom::Geometry * Geo1, const geos::geom::Geometry * Geo2)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		geos::geom::CoordinateSequence * Coords = Geo->getCoordinates();

		double D1 = 0;
		double D2 = 0;

		for(int i = 0; i < Coords->getSize(); ++i)
		{			
			geos::geom::Point * P = factory->createPoint(Coords->getAt(i));
			D1 += P->distance(Geo1);
			D2 += P->distance(Geo2);
			/*if(P->distance(Geo1) < P->distance(Geo2))
				Nb1++;
			else if(P->distance(Geo1) > P->distance(Geo2))
				Nb2++;*/
			delete P;
		}
		delete Coords;
		if(D1 <= D2)
			return 1;
		else
			return 2;
	}

	/**
     * @brief Va couper en deux les Polys qui intersectent une des Lines. On suppose qu'une ligne coupe un polygon en seulement deux polygons.
     */
	std::vector<geos::geom::Geometry*> SplitPolygon(std::vector<geos::geom::Geometry*> Polys, std::vector<geos::geom::Geometry*> Lines)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		std::vector<geos::geom::Geometry*> Res;

		for(int i = 0; i < Polys.size(); ++i)
		{
			geos::geom::Geometry * Poly = Polys[i];

			bool Test = false; //Passe à true si le poly intersecte une ligne
			for(int j = 0; j < Lines.size(); ++j)
			{
				geos::geom::Geometry * Line = Lines[j];

				//Save3GeometryRGB("TEST", Poly, Line, Poly);
				if(!Poly->intersects(Line) || Poly->intersection(Line)->getNumPoints() != 2)
				{
					continue;
				}
				Test = true;
				bool CrossedLine = false; //Passe à true lorsqu'on travaillera sur la deuxième moitié du polygon
				geos::geom::CoordinateSequence* Coords = Poly->getCoordinates();
				geos::geom::CoordinateSequence* Poly1 = new geos::geom::CoordinateArraySequence;
				geos::geom::CoordinateSequence* Poly2 = new geos::geom::CoordinateArraySequence;

				for(int k = 0; k < Coords->size() - 1; ++k)
				{
					geos::geom::CoordinateSequence* EdgeCoord = new geos::geom::CoordinateArraySequence;
					EdgeCoord->add(Coords->getAt(k));
					EdgeCoord->add(Coords->getAt(k+1));
					geos::geom::LineString * Edge = factory->createLineString(EdgeCoord);

					//Save3GeometryRGB("TESTLine" + std::to_string(k), Poly, Edge, Poly);
					if(!Edge->intersects(Line) || (Edge->intersection(Line)->getCoordinates()->getAt(0).x == EdgeCoord->getAt(0).x && Edge->intersection(Line)->getCoordinates()->getAt(0).y == EdgeCoord->getAt(0).y))
					{
						if(!CrossedLine)
						{
							if(k == 0)
							{
								Poly1->add(EdgeCoord->getAt(0));
							}
							Poly1->add(EdgeCoord->getAt(1));
						}
						else
						{
							Poly2->add(EdgeCoord->getAt(1));
						}
						delete Edge;
						continue;
					}
					CrossedLine = !CrossedLine; //On a franchi Line

					if(CrossedLine)
					{
						geos::geom::Coordinate CoordInter = Edge->intersection(Line)->getCoordinates()->getAt(0);
						if(k == 0)
						{
							if(EdgeCoord->getAt(0).x != CoordInter.x || EdgeCoord->getAt(0).y != CoordInter.y)
								Poly1->add(EdgeCoord->getAt(0));
						}

						Poly1->add(CoordInter);
						Poly2->add(CoordInter);
						if(EdgeCoord->getAt(1).x != CoordInter.x || EdgeCoord->getAt(1).y != CoordInter.y)
							Poly2->add(EdgeCoord->getAt(1));
					}
					else
					{
						geos::geom::Coordinate CoordInter = Edge->intersection(Line)->getCoordinates()->getAt(0);

						Poly2->add(CoordInter);
						Poly1->add(CoordInter);
						if(EdgeCoord->getAt(1).x != CoordInter.x || EdgeCoord->getAt(1).y != CoordInter.y)
							Poly1->add(EdgeCoord->getAt(1));
					}
					delete Edge;
				}
				geos::geom::Polygon * Pol1 = factory->createPolygon(factory->createLinearRing(Poly1), NULL);
				Res.push_back(Pol1);
				if(Poly2->size() > 0)
				{
					Poly2->add(Poly2->getAt(0)); //Pour boucler
					geos::geom::Polygon * Pol2 = factory->createPolygon(factory->createLinearRing(Poly2), NULL);
					Res.push_back(Pol2);					
				}
				//Save3GeometryRGB("TEST2", Pol1, factory->createPolygon(factory->createLinearRing(Poly2), NULL), Pol1);
			}
			if(!Test)
				Res.push_back(Poly);
		}
		//SaveGeometry("TEST3", factory->createGeometryCollection(Res));
		return Res;
	}

	/**
     * @brief Convertit les données GEOS en LOD0 de CityGML
     */
	citygml::Geometry* BuildLOD0FromGEOS(std::string name, geos::geom::Geometry * Geometry, double heightmax, double heightmin)
	{
		citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
		for(int i = 0; i < Geometry->getNumGeometries(); ++i)	//Pour chaque polygon de MP
		{
			citygml::Polygon * Poly = new citygml::Polygon("PolyTest");
			citygml::LinearRing * Ring = new citygml::LinearRing("RingTest",true);

			geos::geom::Geometry * TempGeo =  Geometry->getGeometryN(i)->clone();	//clone pour récupérer un const
			geos::geom::CoordinateSequence * Coords = TempGeo->getCoordinates();	//Récupère tous les points du polygon courant de MP

			for(int j = 0; j < Coords->size(); ++j)
			{
				citygml::Polygon * Poly2 = new citygml::Polygon("PolyTest2");
				citygml::LinearRing * Ring2 = new citygml::LinearRing("RingTest2",true);

				int x1 = Coords->getAt(j).x;
				int y1 = Coords->getAt(j).y;
				
				Ring->addVertex(TVec3d(x1, y1, heightmax));

				int x2, y2;
				if(j < Coords->size() - 1)
				{
					x2 = Coords->getAt(j+1).x;
					y2 = Coords->getAt(j+1).y;
				}
				else
				{
					x2 = Coords->getAt(0).x;
					y2 = Coords->getAt(0).y;
				}

				Ring2->addVertex(TVec3d(x1, y1, heightmin));
				Ring2->addVertex(TVec3d(x2, y2, heightmin));
				Ring2->addVertex(TVec3d(x2, y2, heightmax));
				Ring2->addVertex(TVec3d(x1, y1, heightmax));
				Poly2->addRing(Ring2);
				Geom->addPolygon(Poly2);
			}
			Poly->addRing(Ring);
			Geom->addPolygon(Poly);
		}

		return Geom;
	}

	/**
     * @brief Convertit les données GEOS en LOD1 de CityGML
     */
    citygml::CityModel* BuildLOD1FromGEOS(geos::geom::Geometry * Geometry, std::vector<std::pair<double, double>> Hauteurs)
	{
        citygml::CityModel* model = new citygml::CityModel;
		for(int i = 0; i < Geometry->getNumGeometries(); ++i)
		{		
			geos::geom::Geometry * TempGeo =  Geometry->getGeometryN(i)->clone();
			if(TempGeo->getGeometryType() != "Polygon")
				continue;

			citygml::Geometry* Wall = new citygml::Geometry("GeoWall_Building_" + std::to_string(i), citygml::GT_Wall, 0);
			citygml::Geometry* Roof = new citygml::Geometry("GeoRoof_Building_" + std::to_string(i), citygml::GT_Roof, 0);

			double heightmax = Hauteurs[i].second + Hauteurs[i].first;
			double heightmin = Hauteurs[i].second;

			citygml::Polygon * PolyRoof = new citygml::Polygon("PolyRoof");
			citygml::LinearRing * RingRoof = new citygml::LinearRing("RingRoof",true);			
			
			geos::geom::CoordinateSequence * Coords = TempGeo->getCoordinates();	//Récupère tous les points de la geometry

			for(int j = 0; j < Coords->size() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
			{
				citygml::Polygon * PolyWall = new citygml::Polygon("PolyWall_" + std::to_string(j));
				citygml::LinearRing * RingWall = new citygml::LinearRing("RingWall_" + std::to_string(j),true);

				int x1 = Coords->getAt(j).x;
				int y1 = Coords->getAt(j).y;
				
				RingRoof->addVertex(TVec3d(x1, y1, heightmax));

				int x2, y2;
				x2 = Coords->getAt(j+1).x;
				y2 = Coords->getAt(j+1).y;

				RingWall->addVertex(TVec3d(x1, y1, heightmin));
				RingWall->addVertex(TVec3d(x2, y2, heightmin));
				RingWall->addVertex(TVec3d(x2, y2, heightmax));
				RingWall->addVertex(TVec3d(x1, y1, heightmax));
				PolyWall->addRing(RingWall);
				Wall->addPolygon(PolyWall);
			}
			PolyRoof->addRing(RingRoof);
			Roof->addPolygon(PolyRoof);			

			citygml::CityObject* BuildingCO = new citygml::Building("LOD1_Building_" + std::to_string(i));
			citygml::CityObject* WallCO = new citygml::WallSurface("LOD1_Wall_" + std::to_string(i));
			citygml::CityObject* RoofCO = new citygml::RoofSurface("LOD1_Roof_" + std::to_string(i));

			WallCO->addGeometry(Wall);
			RoofCO->addGeometry(Roof);
			model->addCityObject(WallCO);
			model->addCityObject(RoofCO);
			BuildingCO->insertNode(WallCO);
			BuildingCO->insertNode(RoofCO);
			model->addCityObject(BuildingCO);
			model->addCityObjectAsRoot(BuildingCO);

			//std::cout << "Avancement creation LOD1 : " << i+1 << "/" << Geometry->getNumGeometries() << "\r" << std::flush;
		}		

        citygml::ExporterCityGML exporter;
        exporter.exportCityModel(*model, "test.citygml");
		
		std::cout << std::endl << "LOD1 cree.\n";

		citygml::ParserParams params;
		model->finish(params);

        return model;
	}

	/**
     * @brief Convertit une geometry (MultiPolygon) GEOS en geometry CityGML
     */
	citygml::Geometry* ConvertToCityGML(std::string name, geos::geom::Geometry * Geometry)
	{
		citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
		for(int i = 0; i < Geometry->getNumGeometries(); i++)	//Pour chaque polygon de MP
		{
			citygml::Polygon * Poly = new citygml::Polygon("PolyTest");
			citygml::LinearRing * Ring = new citygml::LinearRing("RingTest",true);

			geos::geom::Geometry * TempGeo =  Geometry->getGeometryN(i)->clone();	//clone pour récupérer un const
			geos::geom::CoordinateSequence * Coords = TempGeo->getCoordinates();	//Récupère tous les points du polygon courant de MP

			for(int j = 0; j < Coords->size(); j++)
			{
				Ring->addVertex(TVec3d(Coords->getAt(j).x, Coords->getAt(j).y, 0));
			}
			Poly->addRing(Ring);
			Geom->addPolygon(Poly);
		}

		return Geom;
	}

	/**
     * @brief Relie deux ensembles de geometry en assignant aux geometry de l'une celles qui lui correspondent dans l'autre
     */
	std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > LinkGeos(geos::geom::Geometry * Shape, geos::geom::Geometry * Enveloppe)
	{
		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Res;

		int NbGeoS = Shape->getNumGeometries();
		int NbGeoE = Enveloppe->getNumGeometries();
		
		Res.first.resize(NbGeoS);
		Res.second.resize(NbGeoE);

		for(int i = 0; i < NbGeoS; ++i)
		{
			geos::geom::Geometry * GeoS = Shape->getGeometryN(i)->clone();
			std::vector<int> Vec1;
			for(int j = 0; j < NbGeoE; ++j)
			{
				std::vector<int> Vec2;
				geos::geom::Geometry * GeoE = Enveloppe->getGeometryN(j)->clone();

				double Area = GeoS->intersection(GeoE)->getArea();

				if((Area/GeoS->getArea()) > 0.1 || (Area/GeoE->getArea()) > 0.1)
				{
					//std::cout << "Area : " << GeoS->getArea() << " ; " << GeoE->getArea() << " ; " << Area << std::endl;
					Res.first[i].push_back(j);
					Res.second[j].push_back(i);
				}
				delete GeoE;
			}
			delete GeoS;
			if((i+1)%100 == 0)
			std::cout << "Avancement de LinkGeos : " << i+1 << " / " << NbGeoS << "\r" << std::flush;
		}

		std::cout << "\n";
		//std::cout << Res.first.size() << " , " << NbGeoS << std::endl << Res.second.size() << " , " << NbGeoE << std::endl;
		return Res;
	}

	/**
     * @brief Compare deux geometry en retournant les liens entre les polygons de deux geometry et l'information sur ces liens : si un polygone se retrouve dans les deux geometry, dans une seule ou s'il a été modifié
     */
	std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > CompareGeos(geos::geom::Geometry * Geo1, geos::geom::Geometry * Geo2)
	{
		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Res;

		int NbGeo1 = Geo1->getNumGeometries();
		int NbGeo2 = Geo2->getNumGeometries();

		Res.first.resize(NbGeo1);
		Res.second.resize(NbGeo2);

		double moyenne = 0;
		int cpt = 0;

		for(int i = 0; i < NbGeo1; ++i)
		{
			geos::geom::Geometry * SGeo1 = Geo1->getGeometryN(i)->clone();
			for(int j = 0; j < NbGeo2; ++j)
			{
				geos::geom::Geometry * SGeo2 = Geo2->getGeometryN(j)->clone();

				double Area = SGeo1->intersection(SGeo2)->getArea();
				double val1 = Area/SGeo1->getArea();
				double val2 = Area/SGeo2->getArea();

				if(val1 > 0.99 && val2 > 0.99)//Les polygons sont identiques
				{
					moyenne += val1;
					moyenne += val2;
					cpt +=2;
					Res.first[i].push_back(-1);
					Res.second[j].push_back(-1);
					Res.first[i].push_back(j);
					Res.second[j].push_back(i);
					break;
				}
				else if(val1 > 0.5 && val2 > 0.5)//Le polygon a été modifié
				{
					Res.first[i].push_back(-2);
					Res.second[j].push_back(-2);
					Res.first[i].push_back(j);
					Res.second[j].push_back(i);
					break;
				}
				delete SGeo2;
			}
			delete SGeo1;
			
			std::cout << "Avancement de CompareGeos : " << i << " / " << NbGeo1 << "\r" << std::flush;
		}
		std::cout << "\n";
		std::cout << "Moyenne = " << moyenne/cpt << std::endl;

		return Res;
	}

	/**
     * @brief Convertit un polygon avec des trous en un ensemble de geometries simples
     */
	geos::geom::Geometry * ConvertSimpleGeom(const geos::geom::Geometry * Geo)
	{
		if(Geo->getGeometryType() != "Polygon")
			return Geo->clone();

		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		geos::geom::Polygon * Poly = dynamic_cast<geos::geom::Polygon *>(Geo->clone());

		std::vector<geos::geom::Geometry *> PolyToGeo;
		PolyToGeo.push_back(Poly->getExteriorRing()->clone());
		for(int j = 0; j < Poly->getNumInteriorRing(); ++j)
		{
			PolyToGeo.push_back(Poly->getInteriorRingN(j)->clone());
		}
			
		geos::geom::Geometry * GeoRes = factory->createGeometryCollection(PolyToGeo);
		return GeoRes;
	}

	void Algo::generateLOD0(const URI& uri)
    {
		/////////////////////////////////// Traitement bâtiment par bâtiment 

        std::cout << "void Algo::generateLOD0(const URI& uri)" << std::endl;
        Polygon2D tmp;
        log() << "generateLOD0 on "<< uri.getStringURI() << "\n";
        citygml::CityObject* obj = app().getScene().getCityObjectNode(uri);

        if(obj)
        {
            log() << uri.getStringURI() << "CityObject found\n";
            PolySet roofPoints;			
			double heightmax = 0, heightmin = -1;
            projectRoof(obj,roofPoints, &heightmax, &heightmin);

			std::string name = obj->getId();

			geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
			SaveGeometry(name + "_MP", GeosObj);
			geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
			SaveGeometry(name + "_Enveloppe", Enveloppe);

			//Afficher le LOD0 dans le fenêtre VCITY
					
			citygml::Geometry* geom = new citygml::Geometry(obj->getId()+"_lod0", citygml::GT_Wall, 0);
			geom = BuildLOD0FromGEOS(name, Enveloppe, heightmax, heightmin);
			citygml::CityObject* obj2 = new citygml::WallSurface("tmpObj");
			obj2->addGeometry(geom);
			obj->insertNode(obj2);
			std::cout << "Lod 0 exporte en cityGML" << std::endl;

			//Pour afficher le ground dans VCity
			/*citygml::Geometry* geom = new citygml::Geometry(obj->getId()+"_lod0", citygml::GT_Ground, 0);
			geom = ConvertToCityGML(name, Enveloppe);
			citygml::CityObject* obj2 = new citygml::GroundSurface("tmpObj");
			obj2->addGeometry(geom);
			obj->insertNode(obj2)
			std::cout << "Lod 0 exporte en cityGML" << std::endl;*/
        }
    }

	/**
     * @brief Charge le CityGML et le découpe à l'aide des polygons Geos représentant les bâtiments
     */
	void ExtruderBatiments(geos::geom::Geometry * Batiments)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;

		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();
		int cpt = 0;

		// pour toutes les polygones de coupe

		for(int i = 0; i < tiles.size(); i++)//On parcourt les tuiles du CityGML
		{
			citygml::CityModel* model = tiles[i]->getCityModel();

			for(citygml::CityObject* Building : model->getCityObjectsRoots())//On parcourt les bâtiments de la tuile
			{
				if(Building->getType() != citygml::COT_Building)
					continue;

				for(citygml::CityObject* object : Building->getChildren())//On parcourt les objets (Wall et Roof) du bâtiment
				{					
					if(object->getType() == citygml::COT_WallSurface)
					{
						for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
						{
							for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
							{
								citygml::LinearRing* ring = PolygonCityGML->getExteriorRing();
								const std::vector<TVec3d> vertices = ring->getVertices();
								std::vector<TVec3d>::const_iterator itVertices = vertices.begin();

								citygml::LinearRing* ringResult = new citygml::LinearRing("ring_result", true);
								
								while(itVertices != vertices.end()-1)//pour Chaque sommet
								{
									TVec3d P1 = *itVertices;
									++itVertices;
									TVec3d P2 = *itVertices;
									geos::geom::CoordinateSequence * LineCoords = new geos::geom::CoordinateArraySequence;
									geos::geom::Coordinate C1(P1.x - offset_.x, P1.y - offset_.y);
									geos::geom::Coordinate C2(P2.x - offset_.x, P2.y - offset_.y);
									LineCoords->add(C1);
									LineCoords->add(C2);

									geos::geom::LineString * Line = factory->createLineString(LineCoords); //Représente l'arrête courante du CityGML

									// pour tous les plans de coupe du polygone de coupe courant
									// for plan de coupe
										// si intersection
											// insertion dans ringResult
											// on regarde si les extremités sont dans le polygone que l'on découpe
											// si oui, insertion dans ringResult


									// on calcul les parametres de coupe t
									// quand il y une coupe, on utilise le dernier point de coupe comme raccord pour la prochaine coupe

									for(int j = 0; j < Batiments->getNumGeometries(); ++j)//On cherche si la ligne intersecte un bâtiments du cadastre
									{
										const geos::geom::Geometry * Bati = Batiments->getGeometryN(j);

										for(int k = 0; k < Bati->getNumGeometries(); ++k) //Si le bâtiment est en plusieurs parties
										{
											geos::geom::Geometry * BatiPart1 = Bati->getGeometryN(k)->clone();

											if(BatiPart1->getGeometryType() == "Polygon")
											{
												const geos::geom::Geometry * tmp = BatiPart1;
												BatiPart1 = ConvertSimpleGeom(BatiPart1);
												delete tmp;												
											}

											for(int t = 0; t < BatiPart1->getNumGeometries(); ++t)
											{
												geos::geom::Geometry * BatiPart = BatiPart1->getGeometryN(t)->clone();
												if(!BatiPart->intersects(Line))
												{
													//delete BatiPart;
													//delete BatiCoords;

													continue; //Le bâtiment du cadastre n'intersecte pas l'arrête courante du CityGML
												}
												geos::geom::CoordinateSequence * BatiCoords = BatiPart->getCoordinates();

												for(int c = 0; c < BatiCoords->size() - 1; ++c)
												{
													geos::geom::CoordinateSequence * LineCoords = new geos::geom::CoordinateArraySequence;
													LineCoords->add(BatiCoords->getAt(c));
													LineCoords->add(BatiCoords->getAt(c+1));
													geos::geom::LineString * LineBati = factory->createLineString(LineCoords); //Représente l'arrête courant du batiment issu du Cadastre

													if(!LineBati->intersects(Line))
													{
														//delete LineCoords;
														//delete LineBati;
														continue; //L'arrête du bâtiment du cadastre n'intersecte pas l'arrête courante du CityGML
													}
													geos::geom::Geometry * Inter = LineBati->intersection(Line);//Doit représenter le point d'intersection entre les deux arrêtes
													if(Inter->getNumPoints() != 1)
													{
														//cpt++;
														//Save3GeometryRGB("NonInter" + std::to_string(cpt), BatiPart, Line, LineBati);
														//delete Inter
														//delete LineCoords;
														//delete LineBati;
														continue;//L'intersection n'est pas qu'un seul point : les arrêtes sont colinéaires, ce qui ne convient pas
													}
													if((Inter->getCoordinates()->getAt(0).x != C1.x || Inter->getCoordinates()->getAt(0).y != C1.y) && (Inter->getCoordinates()->getAt(0).x != C2.x || Inter->getCoordinates()->getAt(0).y != C2.y))
													{
														cpt++;													
														//Save3GeometryRGB("PointInter" + std::to_string(cpt), BatiPart, Line, LineBati);
														geos::geom::CoordinateSequence * LineTest = new geos::geom::CoordinateArraySequence;
														geos::geom::Coordinate Temp = Inter->getCoordinates()->getAt(0);
														Temp.x += 2;
														Temp.y += 2;
														LineTest->add(Inter->getCoordinates()->getAt(0));
														LineTest->add(Temp);
														Save3GeometryRGB("PointInter" + std::to_string(cpt) + "_2", BatiPart, Line, factory->createLineString(LineTest));
													}
												}
												//delete BatiPart;
												//delete BatiCoords;
											}
											//delete BatiPart1;											
										}										
										//delete Bati;
									}
									//delete Line;
									//delete LineCoords;
								}
							}
						}
					}
					else if(object->getType() == citygml::COT_RoofSurface)
					{

					}
				}
			}
		}
	}

    void Algo::generateLOD0Scene(geos::geom::Geometry * Shape)//LOD0 sur toute la scène + Comparaison entre CityGML et Cadastre
    {
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		
		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		geos::geom::Geometry * EnveloppeCity = NULL;
		
		std::vector<geos::geom::Geometry *> VecGeos;

		for(int i = 0; i < tiles.size(); i++)//Création de l'enveloppe city à partir des données citygml
		{
			citygml::CityModel* model = tiles[i]->getCityModel();

			citygml::CityObjects objs = model->getCityObjectsRoots();

			int cpt = 0;
		
			for(citygml::CityObjects::iterator it = objs.begin(); it < objs.end(); ++it)
			{
				citygml::CityObject* obj = *it;
				if(obj->getType() == citygml::COT_Building)
				{
					PolySet roofPoints;
					
					double heightmax = 0, heightmin = -1;//Hauteurs min et max du bâtiment
					projectRoof(obj, roofPoints, &heightmax, &heightmin);

					std::string name = obj->getId();

					geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
					
					/////////////
					for(int y = 0; y < GeosObj->getNumGeometries(); y++)
					{
						if(GeosObj->getGeometryN(y)->isValid())
							VecGeos.push_back(GeosObj->getGeometryN(y)->clone());
					}
					/////////////

					geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
					delete GeosObj;
					
					if(EnveloppeCity == NULL)
						EnveloppeCity = Enveloppe;
					else
					{
						geos::geom::Geometry * tmp = EnveloppeCity;
						EnveloppeCity = EnveloppeCity->Union(Enveloppe);
						delete tmp;
					}

					//Afficher le LOD0 dans la fenêtre VCITY					
					/*citygml::Geometry* geom = new citygml::Geometry(obj->getId()+"_lod0", citygml::GT_Wall, 0);
					geom = BuildLOD0FromGEOS(name, Enveloppe, heightmax, heightmin);
					citygml::CityObject* obj2 = new citygml::WallSurface("tmpObj");
					obj2->addGeometry(geom);
					obj->insertNode(obj2);
					std::cout << "Lod 0 exporte en cityGML" << std::endl;*/		

					////Pour afficher le ground dans VCity
					//citygml::Geometry* geom = new citygml::Geometry(obj->getId()+"_lod0", citygml::GT_Ground, 0);
					//geom = ConvertToCityGML(name, Enveloppe);
					//citygml::CityObject* obj2 = new citygml::GroundSurface("tmpObj");
					//obj2->addGeometry(geom);
					//obj->insertNode(obj2);
					//std::cout << "Lod 0 exporte en cityGML" << std::endl;
				}
				cpt++;
				if(cpt%10 == 0)
					std::cout << "Avancement : " << cpt << "/" << objs.size() << " batiments traites.\r" << std::flush;
			}
			std::cout << std::endl;
		}	

		geos::geom::Geometry * City = factory->createGeometryCollection(VecGeos); //Contient tous les polygons 

		////////////////////////////////////////////////////////////////////// Compare le cadastre et le CityGML
				
		if(Shape == NULL || Shape->isEmpty())
		{
			std::cout << "Shape NULL. \n";
			return;
		}

		///////////// Relie les polygons du CityGML et du Cadastre
		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Link = LinkGeos(Shape, EnveloppeCity); 
		/////////////

		std::vector<geos::geom::Geometry*> GeoRes; //Contiendra les bâtiments de CityGML découpés par le Shape

		for(int i = 0; i < EnveloppeCity->getNumGeometries(); ++i)//On parcourt tous les polygons du CityGML
		{
			if(Link.second[i].size() <= 1)
				continue;

			const geos::geom::Polygon * CurrPolyE = dynamic_cast<const geos::geom::Polygon*>(EnveloppeCity->getGeometryN(i));
			
			//Le but de ces lignes est de convertir le polygon avec son exterior ring et ses trous en un ensemble de geometry contenant ceux ci sans qu'ils soient encore liés. On peut ainsi parcourir seulement les arrêtes du polygon sans la notion d'intérieur
			geos::geom::Geometry * CurrGeoE = ConvertSimpleGeom(CurrPolyE);

			std::vector<geos::geom::Geometry *> Shape1;			//Polygon de base
			std::vector<geos::geom::Geometry *> NewShape;		//Intersection avec le cityGML

			std::vector<geos::geom::Geometry *> GeoS;//Contiendra tous les polygons du shape liés au polygon du CityGML
			for(int j = 0; j < Link.second[i].size(); ++j)//Remplissage de GeoS
			{
				geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j])->clone()); //Polygon du shape courant					
				GeoS.push_back(CurrPolyS);
			}
				
			geos::geom::Geometry* UnionPolyS = geos::operation::geounion::CascadedPolygonUnion::Union(factory->createMultiPolygon(GeoS)); //Enveloppe du shape

			std::vector<geos::geom::Geometry *> PolyToGeo;
			for(int k = 0; k < UnionPolyS->getNumGeometries(); ++k)//On parcourt tous les polygons de UnionPolyS pour faire un ensemble de geometry contenant de manière indifférente tous les external ring et les interiors rings.
			//On pourra ainsi calculer l'intersection d'un point avec tous les rings de cette union sans être gêné par le fait qu'un polygon soit "plein".
			//Sinon, un point à l'intérieur du polygon est considéré comme intersect même s'il ne se trouve pas sur les bords.
			{
				geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(UnionPolyS->getGeometryN(k)->clone());
				if(CurrPolyS == NULL)
					continue;
				PolyToGeo.push_back(CurrPolyS->getExteriorRing()->clone());
				for(int j = 0; j < CurrPolyS->getNumInteriorRing(); ++j)
				{
					PolyToGeo.push_back(CurrPolyS->getInteriorRingN(j)->clone());
				}
			}				
			geos::geom::Geometry * CurrGeoS = factory->createGeometryCollection(PolyToGeo);

			for(int j = 0; j < Link.second[i].size(); ++j)//On manipule les polygons EXTERIEURS du shape (ceux qui intersectent CurrGeoS, qui représente l'enveloppe) : on prend un polygon, on le dilate, on lui soustrait tous les autres puis
			//on prend son intersection avec l'enveloppe du CityGML. On obtient ainsi des polygons contenu dans le CityGML et l'union de tous ces polygons doit normalement occuper une grande partie du CityGML
			{
				const geos::geom::Polygon * CurrPolyS = dynamic_cast<const geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j]));
				if(CurrPolyS->isEmpty() || !CurrPolyS->intersects(CurrGeoS))
				{
					Shape1.push_back(CurrPolyS->clone());
					NewShape.push_back(CurrPolyS->clone());
					continue;
				}

				Shape1.push_back(CurrPolyS->clone());

				geos::operation::buffer::BufferParameters BP(1, geos::operation::buffer::BufferParameters::CAP_FLAT, geos::operation::buffer::BufferParameters::JoinStyle::JOIN_MITRE, 2);

				geos::operation::buffer::BufferOp Buffer(CurrPolyS, BP);
				
				geos::geom::Geometry * CurrPolyS2 = Buffer.getResultGeometry(2);

				for(int k = 0; k < Link.second[i].size(); ++k)
				{
					if(k == j)
						continue;
					geos::geom::Geometry * tmp = CurrPolyS2;
					CurrPolyS2 = CurrPolyS2->difference(Shape->getGeometryN(Link.second[i][k]));
					delete tmp;
				}
				
				geos::geom::Geometry * tmp = CurrPolyS2;
				CurrPolyS2 = CurrPolyE->intersection(CurrPolyS2);
				delete tmp;

				NewShape.push_back(CurrPolyS2);
			}			
			geos::geom::Geometry * ShapeRes = factory->createGeometryCollection(NewShape);

			

			////////////////////////////Eliminer les superpositions des polygons :
			
			for(int j = 0; j < NewShape.size(); ++j)//Pour chaque polygon obtenu après l'intersection ...
			{
				geos::geom::Geometry * Geo = NewShape[j];
				
				for(int k = j+1; k < NewShape.size(); ++k)//On le compare avec les autres pour extraire ceux qui s'intersectent
				{
					const geos::geom::Geometry * Geo2 = NewShape[k];

					geos::geom::Geometry * InterGeo = Geo2->intersection(Geo);		
					
					if(InterGeo->isEmpty())//Si les deux polygons ne s'intersectent pas, on passe aux suivants
						continue;

					std::vector <geos::geom::Geometry*> InterVec;//Contiendra les polygons (surfaces communes) formés par l'opérateur intersection entre les deux polygons courant
					
					std::vector <geos::geom::Geometry*> LineVec;//Contiendra les lignes (arrêtes communes) formées par l'opérateur intersection entre les deux polygons courant

					for(int t = 0; t < InterGeo->getNumGeometries(); ++t)//On parcourt les polygons formant l'intersection entre les deux polygons j et k
					{
						const geos::geom::Geometry * GeoTemp = InterGeo->getGeometryN(t);

						//std::cout << GeoTemp->getGeometryType() << " ";

						if(GeoTemp->getGeometryType() == "Polygon" && GeoTemp->getArea() > 0.001) //Seuil pour éliminer les polygons "plats"
							InterVec.push_back(GeoTemp->clone());
						if(GeoTemp->getGeometryType() != "LineString")
							continue;

						//Seules les lignes passeront donc par là : le but est de prolonger la ligne pour couper en deux les polygons qu'elle touche en projetant le point d'intersection de l'autre côté du polygon,
						//sur l'enveloppe du CityGML

						geos::geom::CoordinateSequence * coords = GeoTemp->getCoordinates();
						geos::geom::Point * P1 = factory->createPoint(coords->getAt(0));
						geos::geom::Point * P2 = factory->createPoint(coords->getAt(1));

						for(int p = 0; p < InterGeo->getNumGeometries(); ++p)//On regarde quels sont les polygons qui touchent les linestring en reparcourant la liste des geometry de l'intersection
						{
							const geos::geom::Geometry * GeoTemp2 = InterGeo->getGeometryN(p);							

							if(GeoTemp2->getGeometryType() != "Polygon" || GeoTemp2->getArea() < 0.001)
								continue;

							//Seuls les polygons non "plats" passeront donc par là

							if(P1->intersects(GeoTemp2))//Si le premier point du linestring touche un polygon
							{
								geos::geom::CoordinateSequence * tempcoords = new geos::geom::CoordinateArraySequence;
								tempcoords->add(coords->getAt(0));

								geos::geom::CoordinateSequence * test = geos::operation::distance::DistanceOp::nearestPoints(CurrGeoE, P1);
								//geos::geom::Point * P = factory->createPoint(test->getAt(0));
								//tempcoords->add(geos::operation::distance::DistanceOp::nearestPoints(GeoTemp2, P)->getAt(0)); //On reprojette le point sur le polygon que l'on veut couper en deux pour être sûr de ne pas tomber un pixel à côté

								geos::geom::Coordinate C = test->getAt(0);
								C.x = 2 * C.x - P1->getX();
								C.y = 2 * C.y - P1->getY();
								tempcoords->add(C);

								geos::geom::CoordinateSequence* EdgeCoord = new geos::geom::CoordinateArraySequence;
								EdgeCoord->add(coords->getAt(0));
								EdgeCoord->add(C);
								geos::geom::LineString * Edge = factory->createLineString(EdgeCoord);

								//Save3GeometryRGB("TestLine", Edge, GeoTemp2, Edge);

								if(Edge->intersection(GeoTemp2)->getNumPoints() == 2)
									LineVec.push_back(factory->createLineString(tempcoords));//On crée une linestring entre le point qui touche un polygon et son projeté sur le CityGML pour couper ce polygon en deux

								delete Edge;
							}
							if(P2->intersects(GeoTemp2))//Si le second point du linestring touche un polygon
							{
								geos::geom::CoordinateSequence * tempcoords = new geos::geom::CoordinateArraySequence;
								tempcoords->add(coords->getAt(1));

								geos::geom::CoordinateSequence * test = geos::operation::distance::DistanceOp::nearestPoints(CurrGeoE, P2);
								//geos::geom::Point * P = factory->createPoint(test->getAt(0));
								//tempcoords->add(geos::operation::distance::DistanceOp::nearestPoints(GeoTemp2, P)->getAt(0));

								geos::geom::Coordinate C = test->getAt(0);
								C.x = 2 * C.x - P2->getX();
								C.y = 2 * C.y - P2->getY();
								tempcoords->add(C);

								geos::geom::CoordinateSequence* EdgeCoord = new geos::geom::CoordinateArraySequence;
								EdgeCoord->add(coords->getAt(1));
								EdgeCoord->add(C);
								geos::geom::LineString * Edge = factory->createLineString(EdgeCoord);

								if(Edge->intersection(GeoTemp2)->getNumPoints() == 2)
									LineVec.push_back(factory->createLineString(tempcoords));

								delete Edge;
							}
						}
					}

					if(InterVec.size() == 0)
						continue;

					geos::geom::Geometry* GeoLines = factory->createGeometryCollection(LineVec);

					geos::geom::Geometry * InterGeo2 = factory->createGeometryCollection(InterVec);

					//SaveGeometry("TEST", InterGeo2);

					std::vector<geos::geom::Geometry*> VecGeo; //Contiendra les geometries à assimiler à Geo (qu'il faudra donc retirer à Geo2)
					std::vector<geos::geom::Geometry*> VecGeo2;//Contiendra les geometries à assimiler à Geo2 (qu'il faudra donc retirer à Geo)

					std::vector<geos::geom::Geometry*> SplitPoly = SplitPolygon(InterVec, LineVec);

					for(int t = 0; t < SplitPoly.size(); ++t)
					{
						//Save3GeometryRGB("TEST3", SplitPoly[t], Shape->getGeometryN(Link.second[i][j]), Shape->getGeometryN(Link.second[i][k]));
						int n = GetNearestGeo(SplitPoly[t], Shape->getGeometryN(Link.second[i][j]), Shape->getGeometryN(Link.second[i][k]));
						if(n == 1)
							VecGeo.push_back(SplitPoly[t]/*->buffer(0.01)*/);
						else
							VecGeo2.push_back(SplitPoly[t]/*->buffer(0.01)*/);
					}
					NewShape[k] = Geo2->difference(factory->createGeometryCollection(VecGeo)->Union().release());					
					NewShape[j] = Geo->difference(factory->createGeometryCollection(VecGeo2)->Union().release());
					
					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_1_Poly1", NewShape[j], factory->createGeometryCollection(VecGeo2)->Union().release(), NewShape[j]);
					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_2_Poly2", NewShape[k], factory->createGeometryCollection(VecGeo)->Union().release(), NewShape[k]);
					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_3_Avant" , Geo2, Geo, Geo2);
					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_4_Apres" , NewShape[k], NewShape[j], NewShape[k]);

					Geo = NewShape[j];
				}
			}
			geos::geom::Geometry * ShapeRes2 = factory->createGeometryCollection(NewShape);
			//Scale = 10;
			//Save3GeometryRGB("Shape_" + std::to_string(i), EnveloppeCity, Shape, EnveloppeCity);
			//Save3GeometryRGB("ShapeRes_" + std::to_string(i), EnveloppeCity, ShapeRes, EnveloppeCity);
			//Scale = 20;
			//Save3GeometryRGB("ShapeRes2_" + std::to_string(i), EnveloppeCity, ShapeRes2, EnveloppeCity);

			for(int t = 0; t < ShapeRes2->getNumGeometries(); ++t)
			{
				GeoRes.push_back(ShapeRes2->getGeometryN(t)->clone());
			}
		}
		geos::geom::GeometryCollection * Batiments = factory->createGeometryCollection(GeoRes);
		//Scale = 1;
		//SaveGeometry("Batiments", Batiments);

		ExtruderBatiments(Batiments);

		delete Batiments;
    }

	void Algo::CompareTiles()//Lorsqu'il y a deux tuiles dans VCity, cette fonction crée une image les regroupant pour pouvoir les comparer
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		if(tiles.size() != 2)
			return;
		
		geos::geom::Geometry * EnveloppeCity[2];//On part du principe que le plus vieux est dans le 0 et le plus récent dans le 1
		EnveloppeCity[0] = NULL;
		EnveloppeCity[1] = NULL;
		
		for(int i = 0; i < 2; ++i)
		{
			citygml::CityModel* model = tiles[i]->getCityModel();
						
			citygml::CityObjects objs = model->getCityObjectsRoots();

			int cpt = 0;
		
			for(citygml::CityObjects::iterator it = objs.begin(); it < objs.end(); ++it)
			{
				citygml::CityObject* obj = *it;
				if(obj->getType() == citygml::COT_Building)
				{
					PolySet roofPoints;
					
					double heightmax = 0, heightmin = -1;
					projectRoof(obj,roofPoints, &heightmax, &heightmin);
					
					geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
					geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
					delete GeosObj;
					if(EnveloppeCity[i] == NULL)
						EnveloppeCity[i] = Enveloppe;
					else
					{
						geos::geom::Geometry * tmp = EnveloppeCity[i];
						EnveloppeCity[i] = EnveloppeCity[i]->Union(Enveloppe);
						delete tmp;
					}
				}
				cpt++;
				std::cout << "Avancement tuile " << i+1 << " : " << cpt << "/" << objs.size() << " batiments traites.\n";
			}
		}

		//Save3GeometryRGB("BatiCompare", EnveloppeCity[0], EnveloppeCity[1], factory->createEmptyGeometry());

		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Compare = CompareGeos(EnveloppeCity[0], EnveloppeCity[1]);

		std::vector<geos::geom::Geometry *> BatiDetruits;
		std::vector<geos::geom::Geometry *> BatiCrees;
		std::vector<geos::geom::Geometry *> BatiModifies1;
		std::vector<geos::geom::Geometry *> BatiModifies2;
		std::vector<geos::geom::Geometry *> BatiInchanges;

		for(int i = 0; i < EnveloppeCity[0]->getNumGeometries(); ++i)
		{
			if(Compare.first[i].size() == 0)
				BatiDetruits.push_back(EnveloppeCity[0]->getGeometryN(i)->clone());
			else
			{
				if(Compare.first[i][0] == -1)
					BatiInchanges.push_back(EnveloppeCity[1]->getGeometryN(Compare.first[i][1])->clone());
				else if(Compare.first[i][0] == -2)
				{
					BatiModifies1.push_back(EnveloppeCity[0]->getGeometryN(i)->clone());
					BatiModifies2.push_back(EnveloppeCity[1]->getGeometryN(Compare.first[i][1])->clone());
				}
			}
		}
		for(int i = 0; i < EnveloppeCity[1]->getNumGeometries(); ++i)
		{
			if(Compare.second[i].size() == 0)
				BatiCrees.push_back(EnveloppeCity[1]->getGeometryN(i)->clone());
		}
		//Save3GeometryRGB("BatiCrees", EnveloppeCity[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiCrees));
		//Save3GeometryRGB("BatiDetruits", EnveloppeCity[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiDetruits));
		//Save3GeometryRGB("BatiModifies", EnveloppeCity[1], factory->createGeometryCollection(BatiModifies1), factory->createGeometryCollection(BatiModifies2));		
		//Save3GeometryRGB("BatiInchanges", EnveloppeCity[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiInchanges));

		
		Save3GeometryRGB("BatiCrees", EnveloppeCity[1]->difference(factory->createGeometryCollection(BatiCrees)), factory->createGeometryCollection(BatiCrees), factory->createEmptyGeometry());

		for(auto& it : BatiInchanges) delete it;
		for(auto& it : BatiModifies2) delete it;
		for(auto& it : BatiModifies1) delete it;
		for(auto& it : BatiCrees) delete it;
		for(auto& it : BatiDetruits) delete it;

		delete EnveloppeCity[1];
		delete EnveloppeCity[0];
	}

	void Algo::generateLOD1(geos::geom::Geometry * Shape, std::vector<std::pair<double, double>> Hauteurs)//Hauteurs : Liste les hauteurs et Zmin des polygons de ShapeGeo
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		//m_model = BuildLOD1FromGEOS(Shape, Hauteurs);
		//return; //Pour avoir le LOD1

		if(Shape == NULL || Shape->isEmpty())
		{
            m_model = nullptr;
			std::cout << "Shape NULL. \n";
			return;
		}
		//SaveGeometry("Shape", Shape);
		//geos::geom::Geometry * ShapeSimp = geos::simplify::TopologyPreservingSimplifier::simplify(Shape, 2).release();
		//SaveGeometry("Shape_Simplified", ShapeSimp);

		//BuildLOD1FromGEOS(Shape, Hauteurs);

		geos::geom::Geometry * ShapeRes = Shape->buffer(4)->buffer(-4);

		//SaveGeometry("Shape_Close", ShapeRes);
		
		std::vector<std::pair<double, double>> Hauteurs2;
		std::vector<geos::geom::Geometry *> GeosWithoutHoles;

		for(int i = 0; i < ShapeRes->getNumGeometries(); ++i)
		{
			const geos::geom::Geometry * CurrGeo = ShapeRes->getGeometryN(i);
			
			const geos::geom::Polygon *p;
			if(CurrGeo->getGeometryType() != "Polygon")
				continue;

			p = dynamic_cast<const geos::geom::Polygon*>(CurrGeo);

			double H = 0, Zmin = -9999;
			int cpt = 0;
			for(int j = 0; j < Shape->getNumGeometries(); ++j)
			{
				if(Hauteurs[j].first == -1)
					continue;
				if(Shape->getGeometryN(j)->intersects(CurrGeo))
				{
					H += Hauteurs[j].first;
					Hauteurs[j].first = -1;
					if(Zmin == -9999 || Zmin > Hauteurs[j].second)
						Zmin = Hauteurs[j].second;
					cpt++;
				}
			}
			std::pair<double, double> Temp(H/cpt, Zmin);
			Hauteurs2.push_back(Temp);

			GeosWithoutHoles.push_back(factory->createPolygon(factory->createLinearRing(p->getExteriorRing()->getCoordinates()), NULL));
			//GeosWithoutHoles.push_back(CurrGeo->buffer(CurrGeo->getLength()/10)->buffer(-CurrGeo->getLength()/10));
		}
		geos::geom::Geometry * ShapeResWithoutHoles = factory->createGeometryCollection(GeosWithoutHoles);

		//SaveGeometry("Shape_Close_WithoutHoles", ShapeResWithoutHoles);

		geos::geom::Geometry * ShapeResWithoutHolesSimp = geos::simplify::TopologyPreservingSimplifier::simplify(ShapeResWithoutHoles, 4).release();
		//SaveGeometry("Shape_Close_WithoutHoles_Simplified", ShapeResWithoutHolesSimp);

        m_model = BuildLOD1FromGEOS(ShapeResWithoutHoles, Hauteurs2);
        //m_model = BuildLOD1FromGEOS(ShapeResWithoutHolesSimp, Hauteurs2);
	}
////////////////////////////////////////////////////////////////////////////////
    citygml::CityModel* Algo::getCitymodel()
    {
        return m_model;
    }
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////

