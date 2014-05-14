
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
#include <geos/operation/distance/DistanceOp.h>


typedef std::pair<double,double> Point;
typedef std::vector<Point> Polygon2D;
typedef std::set<Polygon2D> PolySet;

////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////

	double Scale = 2;

    /**
     * @brief projete les toits du CityObject selectioné sur le plan (xy)
     * @param obj CityObject séléctioné
     * @param roofProj un set de Polygon, le résultat de la projection
     */
    void projectRoof(citygml::CityObject* obj, PolySet &roofProj, double * heightmax, double * heightmin)
	{
        if(obj->getType() == citygml::COT_RoofSurface) //Si surface de toit : COT_RoofSurface COT_WallSurface
		{
            std::vector<citygml::Geometry*> geoms = obj->getGeometries();//& geoms
            std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
            for(; itGeom != geoms.end(); ++itGeom) //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)
			{
                std::vector<citygml::Polygon*> polys = (*itGeom)->getPolygons();//& polys
                std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
                for(; itPoly != polys.end(); ++itPoly) //Pour chaque polygone
				{
                    Polygon2D poly;
                    citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                    const std::vector<TVec3d> vertices = ring->getVertices();//& vertices
                    std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
                    for(; itVertices != vertices.end(); ++itVertices)//pour Chaque sommet
					{
                        TVec3d point = *itVertices;
                        poly.push_back(std::make_pair(point.x,point.y)); //on récupere le point
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
        citygml::CityObjects cityObjects = obj->getChildren();//& cityObjects
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
				delete coord;
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
								ImHoles[pos] = 0;
						}
					}
				}
			}
			else
			{
				std::cout << "Geometry n'est pas un polygon. \n";
				delete coord;
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
			SaveImage(name/* + "withHoles"*/, ImHoles, width, height);
		else
			SaveImage(name, Im, width, height);
		
		delete [] Im;
		delete [] ImHoles;
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

		Xmin --;
		Ymin --;

		int width = Xmax - Xmin + Scale;
		int height = Ymax - Ymin + Scale;

		int* Im = new int[width * height];
		int* ImR = new int[width * height];
		int* ImG = new int[width * height];
		int* ImB = new int[width * height];
		int* ImHoles = new int[width * height];
		int* ImHolesR = new int[width * height];
		int* ImHolesG = new int[width * height];
		int* ImHolesB = new int[width * height];
		bool Holes = false; //Passe à true en présence d'holes

		for(int g = 0; g < 3; g ++)
		{			
			memset(Im, 1, width*height*sizeof(int));			
			memset(ImHoles, 1, width*height*sizeof(int));

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

				const geos::geom::Polygon *p = dynamic_cast<const geos::geom::Polygon*>(Geo);
				if(p)
				{
					delete coord;
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
									ImHoles[pos] = 0;
							}
						}
					}
				}
				else
				{
					std::cout << "Geometry n'est pas un polygon. \n";
					delete coord;
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
			if(g==0)
			{
				memcpy(ImR, Im, width*height*sizeof(int));
				memcpy(ImHolesR, ImHoles, width*height*sizeof(int));
			}
			else if(g==1)
			{
				memcpy(ImG, Im, width*height*sizeof(int));
				memcpy(ImHolesG, ImHoles, width*height*sizeof(int));
			}
			else
			{
				memcpy(ImB, Im, width*height*sizeof(int));
				memcpy(ImHolesB, ImHoles, width*height*sizeof(int));
			}
		}
		
		if(Holes)
			SaveImageRGB(name/* + "withHoles"*/, ImHolesR, ImHolesG, ImHolesB, width, height);
		else
			SaveImageRGB(name, ImR, ImG, ImB, width, height);
		
		delete [] Im;
		delete [] ImR;
		delete [] ImG;
		delete [] ImB;
		delete [] ImHoles;
		delete [] ImHolesR;
		delete [] ImHolesG;
		delete [] ImHolesB;
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
					std::vector<geos::geom::Geometry*> Holes;// = new std::vector<geos::geom::Geometry*>; //Vecteur contenant tous les polygones à l'intérieur du premier, qui sont donc considérés comme des trous

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
									Holes.push_back((geos::geom::Geometry*)Hole);
								FirstPoint[0] = -1;
								tempcoord.clear();
							}
						}
					}
					delete coordGeo;
					geos::geom::Polygon *P;

					if(Holes.size() == 0)
						P = factory->createPolygon(shell, NULL);
					else
						P = factory->createPolygon(shell, &Holes);

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
     * @brief Convertit les données GEOS en LOD0 de CityGML
     */
	citygml::Geometry* BuildLOD0FromGEOS(std::string name, geos::geom::Geometry * Geometry, double heightmax, double heightmin)
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
			for(int j = 0; j < NbGeoE; ++j)
			{
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
			//std::cout << "Avancement de LinkGeos : " << i+1 << " / " << NbGeoS << std::endl;
		}

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

				if(val1 > 0.95 && val2 > 0.95)//Les polygons sont identiques
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
			
			std::cout << "Avancement de CompareGeos : " << i << " / " << NbGeo1 << std::endl;
		}

		std::cout << "Moyenne = " << moyenne/cpt << std::endl;

		return Res;
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

    void Algo::generateLOD0Scene(geos::geom::Geometry ** ShapeGeo)
    {
        //LOD0 sur toute la scène
		
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		geos::geom::Geometry * EnveloppeCity = NULL;

		for(int i = 0; i < tiles.size(); i++)//Création de l'enveloppe city à partir des données citygml
		{
			citygml::CityModel* model = tiles[i]->getCityModel();

			//std::cout << "tile enveloppe " << tiles[i]->getEnvelope() << std::endl;
			
			citygml::CityObjects& objs = model->getCityObjectsRoots();

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
					geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
					
					if(EnveloppeCity == NULL)
						EnveloppeCity = Enveloppe;
					else
						EnveloppeCity = EnveloppeCity->Union(Enveloppe);

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
				std::cout << "Avancement : " << cpt << "/" << objs.size() << " batiments traites.\n";
			}
		}

		//////////////////////////////////////////////////////////////////////
		
		geos::geom::Geometry * Shape = (*ShapeGeo);

		if(Shape == NULL)
			return;

		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Link = LinkGeos(Shape, EnveloppeCity); 
		int cpt = 0;
		
		for(int i = 0; i < EnveloppeCity->getNumGeometries(); i++)//On parcourt tous les polygons de l'enveloppe
		{
			geos::geom::Polygon * CurrPolyE = dynamic_cast<geos::geom::Polygon*>(EnveloppeCity->getGeometryN(i)->clone());
			//Le but de ces lignes est de convertir le polygon avec son exterior ring et ses trous en un ensemble de geometry contenant ceux ci sans qu'ils soient encore liés. On peut ainsi parcourir seulement les arrêtes du polygon sans la notion d'intérieur
			std::vector<geos::geom::Geometry *> PolyToGeo;
			PolyToGeo.push_back(CurrPolyE->getExteriorRing()->clone());
			for(int j = 0; j < CurrPolyE->getNumInteriorRing(); j++)
			{
				PolyToGeo.push_back(CurrPolyE->getInteriorRingN(j)->clone());
			}
			geos::geom::Geometry * CurrGeoE = factory->createGeometryCollection(PolyToGeo);
			////////////////////////////
			
			if(Link.second[i].size() > 0)//On vérifie qu'au moins un polygon du shape lui soit associé
			{				
				std::vector<geos::geom::Geometry *> Geos;//Contiendra tous les polygons du shape liés au polygon de l'enveloppe
				std::vector<geos::geom::Geometry *> Geos2;//Contiendra ces même polygons une fois modifiés pour coller à l'enveloppe
				for(int j = 0; j < Link.second[i].size(); j++)//Remplissage de Geos
				{
					geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j])->clone()); //Polygon du shape courant					
					Geos.push_back(CurrPolyS);
				}
				
				geos::geom::Geometry* UnionPolyS = geos::operation::geounion::CascadedPolygonUnion::Union(factory->createMultiPolygon(Geos)); //Enveloppe du shape
				
				std::vector<geos::geom::Geometry *> PolyToGeo;
				for(int k = 0; k < UnionPolyS->getNumGeometries(); k++)//On parcourt tous les polygons de UnionPolys pour faire un ensemble de geometry contenant de manière indifférente tous les external ring et les interiors rings. On pourra ainsi calculer l'intersection d'un point avec tous les rings de cette union sans être gêné par le fait qu'un polygon soit "plein". Sinon, un point à l'intérieur du polygon est considéré comme intersect même s'il ne se trouve pas sur les bords.
				{
					geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(UnionPolyS->getGeometryN(k)->clone());
					if(CurrPolyS == NULL)
					{
						continue;
					}
					PolyToGeo.push_back(CurrPolyS->getExteriorRing()->clone());
					for(int j = 0; j < CurrPolyS->getNumInteriorRing(); j++)
					{
						PolyToGeo.push_back(CurrPolyS->getInteriorRingN(j)->clone());
					}
				}				
				geos::geom::Geometry * CurrGeoS = factory->createGeometryCollection(PolyToGeo);

				//On va modifier le shape en recherchant les points situés sur son enveloppe et en les déplacements sur l'enveloppe du CityGML en conservant les polygons ainsi modifiés
				for(int j = 0; j < Link.second[i].size(); j++)//Remplissage de Geos2
				{
					geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j])->clone());
					geos::geom::CoordinateSequence * CurrPolyExt = CurrPolyS->getExteriorRing()->getCoordinates();
					geos::geom::CoordinateArraySequence CoordExt;//Contiendra les coordonnées modifiées de l'exterior ring du polygon courant

					for(int k = 0; k < CurrPolyExt->getSize(); k++)//Traitement de l'exterior ring
					{
						geos::geom::Point * CurrPoint = factory->createPoint(CurrPolyExt->getAt(k));

						if(CurrPoint->intersects(CurrGeoS))
						{
							CoordExt.add(geos::operation::distance::DistanceOp::nearestPoints(CurrGeoE, CurrPoint)->getAt(0));
						}
						else
						{
							CoordExt.add(*CurrPoint->getCoordinate());
						}				
					}

					std::vector<geos::geom::Geometry*> PolyInt;// = new std::vector<geos::geom::Geometry*>;//Contiendra les interior rings modifiés du polygon courant

					for(int p = 0; p < CurrPolyS->getNumInteriorRing(); p++)
					{
						geos::geom::CoordinateSequence * CurrPolyInt = CurrPolyS->getInteriorRingN(p)->getCoordinates();
						geos::geom::CoordinateArraySequence CoordInt;//Contiendra les coordonnées modifiées de l'interior ring p du polygon courant

						for(int k = 0; k < CurrPolyInt->getSize(); k++)//Traitement de l'interior ring p
						{
							geos::geom::Point * CurrPoint = factory->createPoint(CurrPolyInt->getAt(k));

							if(CurrPoint->intersects(CurrGeoS))
							{
								CoordInt.add(geos::operation::distance::DistanceOp::nearestPoints(CurrGeoE, CurrPoint)->getAt(0));
							}
							else
							{
								CoordInt.add(*CurrPoint->getCoordinate());
							}
						}
						PolyInt.push_back(factory->createLinearRing(CoordInt));
					}
					Geos2.push_back(factory->createPolygon(factory->createLinearRing(CoordExt), &PolyInt));
				}
				cpt++;
				Save3GeometryRGB("Poly" + std::to_string(cpt), CurrPolyE, factory->createEmptyGeometry(), factory->createEmptyGeometry());//Affiche l'enveloppe du citygml
				Save3GeometryRGB("Poly" + std::to_string(cpt) + "_", factory->createEmptyGeometry(), UnionPolyS, factory->createEmptyGeometry());//Affiche l'enveloppe du shape
				Save3GeometryRGB("Poly" + std::to_string(cpt) + "_test1", CurrPolyE, factory->createGeometryCollection(Geos), factory->createEmptyGeometry());//Affiche l'enveloppe avec les polygons du shape associés
				try{
				Save3GeometryRGB("Poly" + std::to_string(cpt) + "_test2", CurrPolyE, factory->createGeometryCollection(Geos2), factory->createEmptyGeometry());//Affiche l'enveloppe avec les polygons du shape modifiés
				}
				catch(std::exception& e)
				{
					std::cout << Geos2.size() << std::endl;
					std::cout << e.what() << '\n';
				}
			}
		}

		
		/*geos::geom::Geometry * ShapeU; //Contient tous les polygons de Shape dans une geometry valide
				
		std::cout << "Creation de ShapeU ... \n";
		ShapeU = geos::operation::geounion::CascadedPolygonUnion::Union(dynamic_cast<geos::geom::MultiPolygon*>(Shape));
		std::cout << "Creation de ShapeU terminee. \n" << "ShapeU valid = " << ShapeU->isValid() << " Shape valid = " << Shape->isValid() << std::endl;

		SaveGeometry("Shape", Shape);
		SaveGeometry("ShapeU", ShapeU);

		SaveGeometry("EnveloppeCity", EnveloppeCity);

		geos::geom::Geometry * TempGeo = factory->createEmptyGeometry();
			
		Save3GeometryRGB("Shape_Enveloppe", Shape, EnveloppeCity, TempGeo);

		std::cout << "Creation de l'Union ..." << std::endl;
		try
		{
			TempGeo = ShapeU->Union(EnveloppeCity);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << '\n';
		}
		SaveGeometry("Union", TempGeo);
		Save3GeometryRGB("Shape_Enveloppe_Union", ShapeU, EnveloppeCity, TempGeo);

		std::cout << "Creation de l'intersection ..." << std::endl;
		TempGeo = ShapeU->intersection(EnveloppeCity);
		SaveGeometry("Intersection", TempGeo);
		Save3GeometryRGB("Shape_Enveloppe_Intersection", ShapeU, EnveloppeCity, TempGeo);

		std::cout << "Creation de la SymDifference ..." << std::endl;
		TempGeo = ShapeU->symDifference(EnveloppeCity);
		SaveGeometry("SymDifference", TempGeo);
		Save3GeometryRGB("Shape_Enveloppe_SymDifference", ShapeU, EnveloppeCity, TempGeo);

		//std::cout << "Creation de Shape - Enveloppe ..." << std::endl;
		//SaveGeometry("S-E", ShapeU->difference(EnveloppeCity));
		//std::cout << "Creation de Enveloppe - Shape ..." << std::endl;
		//SaveGeometry("E-S", EnveloppeCity->difference(ShapeU));*/

		/*SaveGeometry("Enveloppe_City", EnveloppeCity);

		//SaveGeometry("Enveloppe_City_Simplified", geos::simplify::TopologyPreservingSimplifier::simplify(EnveloppeCity, 2).get());*/
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
						
			citygml::CityObjects objs = model->getCityObjectsRoots();//&objs

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

		Save3GeometryRGB("BatiCompare", EnveloppeCity[0], EnveloppeCity[1], factory->createEmptyGeometry());

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
		
		Save3GeometryRGB("BatiCrees", EnveloppeCity[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiCrees));
		Save3GeometryRGB("BatiDetruits", EnveloppeCity[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiDetruits));
		Save3GeometryRGB("BatiModifies", EnveloppeCity[1], factory->createGeometryCollection(BatiModifies1), factory->createGeometryCollection(BatiModifies2));		
		Save3GeometryRGB("BatiInchanges", EnveloppeCity[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiInchanges));

		for(auto& it : BatiInchanges) delete it;
		for(auto& it : BatiModifies2) delete it;
		for(auto& it : BatiModifies1) delete it;
		for(auto& it : BatiCrees) delete it;
		for(auto& it : BatiDetruits) delete it;

		delete EnveloppeCity[1];
		delete EnveloppeCity[0];
	}

////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////

