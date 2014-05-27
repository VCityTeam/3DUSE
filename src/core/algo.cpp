
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

typedef std::pair<double,double> Point;
typedef std::vector<Point> Polygon2D;
typedef std::set<Polygon2D> PolySet;

////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
	double Scale = 1; //Définit le zoom des images sauvegardées, avec 1 = 1 mètre/pixel.

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
                        poly.push_back(std::make_pair(point.x - 1840000, point.y - 5170000)); //on récupere le point //1841069.875000 5175491.500000
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

		//std::cout << Xmin << ";" << Xmax << "  " << Ymin << ";" << Ymax << std::endl;
		
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
						{
							std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
							int a;
							std::cin >> a;
						}
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
							{
								std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
								int a;
								std::cin >> a;
							}
							else
								ImHoles[pos] = 0;
						}
					}
				}
			}
			else
			{
				//std::cout << "Geometry n'est pas un polygon. \n";
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
						{
							std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
							int a;
							std::cin >> a;
						}
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
					//std::cout << "Geometry n'est pas un polygon. Type = "<< Geo->getGeometryType() <<  std::endl;
					if(Geo->getNumGeometries() == 1)
					{
						delete coord;
						coord = Geo->getCoordinates();

						if(coord->size() != 0)
						{
							for(int j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
							{
								int x1 = Scale*coord->getAt(j).x - Xmin;
								int y1 = Scale*coord->getAt(j).y - Ymin;

								int	x2 = Scale*coord->getAt(j+1).x - Xmin;
								int	y2 = Scale*coord->getAt(j+1).y - Ymin;

								//std::cout << x1 << ";" << y1 << "  " << x2 << ";" << y2 << "\n";
					
								std::vector<std::pair<int,int>> Points = TracerSegment(x1, y1, x2, y2);

								for(std::vector<std::pair<int,int>>::iterator it = Points.begin(); it != Points.end(); ++it)
								{
									int pos = it->first + it->second * width;
									if(pos >= width * height || pos < 0)
										std::cout << "Probleme creation image. Position en dehors de l'image." << std::endl;
									else
									{
										//std::cout << pos << std::endl;
										Im[pos] = 0;
										ImHoles[pos] = 0;
									}
								}
							}
						}
						else
							std::cout << "Geo Vide \n";
					}
					else if(Geo->getNumGeometries() > 1)
					{
						for(int k = 0; k < Geo->getNumGeometries(); k++)
						{
							const geos::geom::Geometry * TempGeo = Geo->getGeometryN(k);
							delete coord;
							coord = TempGeo->getCoordinates();

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
										{
											Im[pos] = 0;
											ImHoles[pos] = 0;
										}
									}
								}
							}
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
				
		//if(Holes)
			SaveImageRGB(name/* + "withHoles"*/, ImHolesR, ImHolesG, ImHolesB, width, height);
		//else
		//	SaveImageRGB(name, ImR, ImG, ImB, width, height);		
		
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
     * @brief Realise l'operation overlaySnapRounded de JTS TestBuilder pour séparer la geometry G1 (polygon) grâce à G2 (multi linestrings)
     */
	geos::geom::Geometry * overlaySnapRounded(const geos::geom::Geometry* G1, const geos::geom::Geometry* G2)
	{		
		const geos::geom::GeometryFactory * factory = G1->getFactory();

		std::vector<const geos::geom::LineString*> Lines;
		std::vector<geos::geom::Geometry*> GeoLines;
		geos::geom::util::LinearComponentExtracter::getLines(*G1, Lines);
		if(!G2->isEmpty())
			geos::geom::util::LinearComponentExtracter::getLines(*G2, Lines);
		
		for(int i = 0; i < Lines.size(); ++i)
		{
			GeoLines.push_back(Lines[i]->clone());
		}
		std::auto_ptr<geos::geom::Geometry> nodedLinework = geos::noding::GeometryNoder::node(*factory->createGeometryCollection(GeoLines));
		std::auto_ptr<geos::geom::Geometry> nodedDedupedLinework = nodedLinework->Union();

		geos::operation::polygonize::Polygonizer Polygonizer;
		Polygonizer.add(nodedDedupedLinework.get());

		std::vector<geos::geom::Polygon*> * polys = Polygonizer.getPolygons();
		std::vector<geos::geom::Geometry*> * Geos = new std::vector<geos::geom::Geometry*>;

		for(int i = 0; i < polys->size(); ++i)
		{
			if(!polys->at(i)->isValid())
				std::cout << "NonValid \n";
			else
				Geos->push_back(polys->at(i));
		}
		return factory->createGeometryCollection(Geos);
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
	void BuildLOD1FromGEOS(geos::geom::Geometry * Geometry, std::vector<std::pair<double, double>> Hauteurs)
	{
		citygml::CityModel model;

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
			BuildingCO->insertNode(WallCO);
			BuildingCO->insertNode(RoofCO);
			model.addCityObjectAsRoot(BuildingCO);

			//std::cout << "Avancement creation LOD1 : " << i+1 << "/" << Geometry->getNumGeometries() << "\r" << std::flush;
		}		

		citygml::ExporterCityGML exporter;
		exporter.exportCityModel(model, "test.citygml");
		
		std::cout << std::endl << "LOD1 cree.\n";
	}

	/*void BuildLOD1FromGEOS(geos::geom::Geometry * Geometry, std::vector<std::pair<double, double>> Hauteurs)
	{
		citygml::Geometry* Wall = new citygml::Geometry("LOD1_Wall", citygml::GT_Wall, 0);
		citygml::Geometry* Roof = new citygml::Geometry("LOD1_Roof", citygml::GT_Roof, 0);

		for(int i = 0; i < Geometry->getNumGeometries(); ++i)
		{
			double heightmax = Hauteurs[i].second + Hauteurs[i].first;
			double heightmin = Hauteurs[i].second;
			citygml::Polygon * Poly = new citygml::Polygon("PolyTest");
			citygml::LinearRing * Ring = new citygml::LinearRing("RingTest",true);

			geos::geom::Geometry * TempGeo =  Geometry->getGeometryN(i)->clone();
			
			//if(TempGeo->getGeometryType() != "Polygon")
			//	continue;
			geos::geom::CoordinateSequence * Coords = TempGeo->getCoordinates();	//Récupère tous les points de la geometry

			for(int j = 0; j < Coords->size() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
			{
				citygml::Polygon * Poly2 = new citygml::Polygon("PolyTest2");
				citygml::LinearRing * Ring2 = new citygml::LinearRing("RingTest2",true);

				int x1 = Coords->getAt(j).x;
				int y1 = Coords->getAt(j).y;
				
				Ring->addVertex(TVec3d(x1, y1, heightmax));

				int x2, y2;
				x2 = Coords->getAt(j+1).x;
				y2 = Coords->getAt(j+1).y;

				Ring2->addVertex(TVec3d(x1, y1, heightmin));
				Ring2->addVertex(TVec3d(x2, y2, heightmin));
				Ring2->addVertex(TVec3d(x2, y2, heightmax));
				Ring2->addVertex(TVec3d(x1, y1, heightmax));
				Poly2->addRing(Ring2);
				Wall->addPolygon(Poly2);
			}
			Poly->addRing(Ring);
			Roof->addPolygon(Poly);
		}
		
		citygml::CityModel model;// = new citygml::CityModel();

		citygml::CityObject* obj = new citygml::WallSurface("tmpObj1");
		citygml::CityObject* obj2 = new citygml::RoofSurface("tmpObj2");
		obj->addGeometry(Wall);
		obj2->addGeometry(Roof);
		model.addCityObjectAsRoot(obj);
		model.addCityObjectAsRoot(obj2);
		
		citygml::ExporterCityGML exporter;
		exporter.exportCityModel(model, "test.citygml");
		
		std::cout << "LOD1 cree.\n";
	}*/

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

    void Algo::generateLOD0Scene(geos::geom::Geometry * Shape)//LOD0 sur toute la scène + Comparaison entre CityGML et Cadastre
    {		
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		/*geos::geom::CoordinateSequence* temp = new geos::geom::CoordinateArraySequence;

		temp->add(geos::geom::Coordinate(20, 20));
		temp->add(geos::geom::Coordinate(80, 20));
		temp->add(geos::geom::Coordinate(80, 80));
		temp->add(geos::geom::Coordinate(20, 80));
		temp->add(geos::geom::Coordinate(20, 20));
	
		geos::geom::LinearRing * shell2=factory->createLinearRing(temp);

		geos::geom::Polygon* P2 = factory->createPolygon(shell2, NULL);

		geos::geom::CoordinateSequence* temp2 = new geos::geom::CoordinateArraySequence;


		temp2->add(geos::geom::Coordinate(50, 21));
		temp2->add(geos::geom::Coordinate(50, 80));

		geos::geom::LineString * LS = factory->createLineString(temp2);

		geos::geom::Geometry * Res = overlaySnapRounded(P2, LS);

		Save3GeometryRGB("TEST" , P2, LS, factory->createEmptyGeometry());
		Save3GeometryRGB("TEST2" , P2, LS, Res);

		std::cout << P2->getNumGeometries() << std::endl;
		std::cout << Res->getNumGeometries() << std::endl;

		return;*/

		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		geos::geom::Geometry * EnveloppeCity = NULL;

		for(int i = 0; i < tiles.size(); i++)//Création de l'enveloppe city à partir des données citygml
		{
			citygml::CityModel* model = tiles[i]->getCityModel();

			//std::cout << "tile enveloppe " << tiles[i]->getEnvelope() << std::endl;
			
			citygml::CityObjects objs = model->getCityObjectsRoots();//& objs

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
					std::cout << "Avancement : " << cpt << "/" << objs.size() << " batiments traites.\n";
			}
		}

		////////////////////////////////////////////////////////////////////// Compare le cadastre et le CityGML

		
		
		if(Shape == NULL)
		{
			std::cout << "Shape NULL. \n";
			return;
		}

		///////////// Relie les polygons du CityGML et du Cadastre
		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Link = LinkGeos(Shape, EnveloppeCity); 
		/////////////
		int cpt = 0;

		for(int i = 0; i < EnveloppeCity->getNumGeometries(); ++i)//On parcourt tous les polygons du CityGML
		{
			if(Link.second[i].size() <= 1)
				continue;
			std::cout<< EnveloppeCity->getGeometryN(i)->getGeometryType() << std::endl;
			const geos::geom::Polygon * CurrPolyE = dynamic_cast<const geos::geom::Polygon*>(EnveloppeCity->getGeometryN(i));
			//Le but de ces lignes est de convertir le polygon avec son exterior ring et ses trous en un ensemble de geometry contenant ceux ci sans qu'ils soient encore liés. On peut ainsi parcourir seulement les arrêtes du polygon sans la notion d'intérieur
			std::vector<geos::geom::Geometry *> PolyToGeo2;
			PolyToGeo2.push_back(CurrPolyE->getExteriorRing()->clone());
			for(int j = 0; j < CurrPolyE->getNumInteriorRing(); ++j)
			{
				PolyToGeo2.push_back(CurrPolyE->getInteriorRingN(j)->clone());
			}
			geos::geom::Geometry * CurrGeoE = factory->createGeometryCollection(PolyToGeo2);

			std::vector<geos::geom::Geometry *> Shape1;			//Polygon de base
			//std::vector<geos::geom::Geometry *> NewShape;		//Polygon dilaté
			//std::vector<geos::geom::Geometry *> NewShape2;	//Difference avec les autres polygons
			std::vector<geos::geom::Geometry *> NewShape3;		//Intersection avec le cityGML
			std::vector<geos::geom::Geometry *> NewShape4;		//Ouverture morphologique

			std::vector<geos::geom::Geometry *> Geos;//Contiendra tous les polygons du shape liés au polygon du CityGML
			for(int j = 0; j < Link.second[i].size(); ++j)//Remplissage de Geos
			{
				geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j])->clone()); //Polygon du shape courant					
				Geos.push_back(CurrPolyS);
			}
				
			geos::geom::Geometry* UnionPolyS = geos::operation::geounion::CascadedPolygonUnion::Union(factory->createMultiPolygon(Geos)); //Enveloppe du shape

			std::vector<geos::geom::Geometry *> PolyToGeo;
			for(int k = 0; k < UnionPolyS->getNumGeometries(); ++k)//On parcourt tous les polygons de UnionPolys pour faire un ensemble de geometry contenant de manière indifférente tous les external ring et les interiors rings. On pourra ainsi calculer l'intersection d'un point avec tous les rings de cette union sans être gêné par le fait qu'un polygon soit "plein". Sinon, un point à l'intérieur du polygon est considéré comme intersect même s'il ne se trouve pas sur les bords.
			{
				geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(UnionPolyS->getGeometryN(k)->clone());
				if(CurrPolyS == NULL)
				{
					continue;
				}
				PolyToGeo.push_back(CurrPolyS->getExteriorRing()->clone());
				for(int j = 0; j < CurrPolyS->getNumInteriorRing(); ++j)
				{
					PolyToGeo.push_back(CurrPolyS->getInteriorRingN(j)->clone());
				}
			}				
			geos::geom::Geometry * CurrGeoS = factory->createGeometryCollection(PolyToGeo);

			for(int j = 0; j < Link.second[i].size(); ++j)
			{
				const geos::geom::Polygon * CurrPolyS = dynamic_cast<const geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j]));
				if(CurrPolyS->isEmpty() || !CurrPolyS->intersects(CurrGeoS))
				{
					Shape1.push_back(CurrPolyS->clone());
					NewShape3.push_back(CurrPolyS->clone());
					NewShape4.push_back(CurrPolyS->clone());
					continue;
				}

				cpt++;
				std::string name;
				if(cpt < 10)
					name = "Poly00" + std::to_string(cpt);
				else if(cpt < 100)
					name = "Poly0" + std::to_string(cpt);
				else
					name = "Poly" + std::to_string(cpt);

				Shape1.push_back(CurrPolyS->clone());

				//Save3GeometryRGB(name, CurrPolyE, CurrPolyS, factory->createEmptyGeometry());

				geos::operation::buffer::BufferParameters BP(1, geos::operation::buffer::BufferParameters::CAP_FLAT, geos::operation::buffer::BufferParameters::JoinStyle::JOIN_MITRE, 2);

				geos::operation::buffer::BufferOp Buffer(CurrPolyS, BP);				
				
				geos::geom::Geometry * CurrPolyS2 = Buffer.getResultGeometry(2);//CurrPolyS->buffer(2, 0, geos::operation::buffer::BufferOp::CAP_BUTT);

				//Save3GeometryRGB(name + "_1", CurrPolyE, CurrPolyS2, CurrPolyS);

				for(int k = 0; k < Link.second[i].size(); ++k)
				{
					if(k == j)
						continue;
					geos::geom::Geometry * tmp = CurrPolyS2;
					CurrPolyS2 = CurrPolyS2->difference(Shape->getGeometryN(Link.second[i][k]));
					delete tmp;
				}

				//Save3GeometryRGB(name + "_2", CurrPolyE, CurrPolyS2, CurrPolyS);

				geos::geom::Geometry * tmp = CurrPolyS2;
				CurrPolyS2 = CurrPolyS2->intersection(CurrPolyE);
				delete tmp;

				NewShape3.push_back(CurrPolyS2);

				//Save3GeometryRGB(name + "_3", CurrPolyE, CurrPolyS2, CurrPolyS);

				/*geos::operation::buffer::BufferParameters BP2(1, geos::operation::buffer::BufferParameters::CAP_FLAT, geos::operation::buffer::BufferParameters::JoinStyle::JOIN_MITRE, 2);

				geos::operation::buffer::BufferOp Buffer2(CurrPolyS2, BP);

				geos::geom::Geometry * CurrPolyS3 = Buffer2.getResultGeometry(-1);//CurrPolyS2->buffer(-2, 0, geos::operation::buffer::BufferOp::CAP_BUTT);

				if(CurrPolyS3->isEmpty())
				{
					NewShape4.push_back(CurrPolyS2);
					continue;
				}

				//Save3GeometryRGB(name + "_4", CurrPolyE, CurrPolyS3, CurrPolyS);

				geos::operation::buffer::BufferParameters BP3(1, geos::operation::buffer::BufferParameters::CAP_FLAT, geos::operation::buffer::BufferParameters::JoinStyle::JOIN_MITRE, 2);

				geos::operation::buffer::BufferOp Buffer3(CurrPolyS3, BP);	

				geos::geom::Geometry * CurrPolyS4 = Buffer3.getResultGeometry(1);//CurrPolyS3->buffer(2, 0, geos::operation::buffer::BufferOp::CAP_BUTT);
								
				//Save3GeometryRGB(name + "_5", CurrPolyE, CurrPolyS4, CurrPolyS);

				NewShape4.push_back(CurrPolyS4);*/
			}
			//Save3GeometryRGB("Im_" + std::to_string(i) + "_1", CurrPolyE, factory->createGeometryCollection(Shape1), factory->createEmptyGeometry());//Polygon de base
			////Save3GeometryRGB("Im_" + std::to_string(i) + "_2", CurrPolyE, factory->createGeometryCollection(NewShape), factory->createEmptyGeometry());//Polygon dilaté
			////Save3GeometryRGB("Im_" + std::to_string(i) + "_3", CurrPolyE, factory->createGeometryCollection(NewShape2), factory->createEmptyGeometry());//Difference avec les autres polygons
			//Save3GeometryRGB("Im_" + std::to_string(i) + "_4", CurrPolyE, factory->createGeometryCollection(NewShape3), factory->createEmptyGeometry());//Intersection avec le cityGML
			////Save3GeometryRGB("Im_" + std::to_string(i) + "_5", CurrPolyE, factory->createGeometryCollection(NewShape4), factory->createEmptyGeometry());//Ouverture morphologique
			
			for(int j = 0; j < NewShape3.size(); ++j)//Pour chaque polygon obtenu après l'intersection ...
			{
				geos::geom::Geometry * Geo = NewShape3[j];
				
				for(int k = j+1; k < NewShape3.size(); ++k)//On le compare avec les autres pour extraire ceux qui s'intersectent
				{
					const geos::geom::Geometry * Geo2 = NewShape3[k];

					geos::geom::Geometry * InterGeo = Geo2->intersection(Geo);
					std::vector <geos::geom::Geometry*> InterVec;
					
					if(InterGeo->isEmpty())
						continue;
					
					std::vector <geos::geom::Geometry*> LineVec;

					for(int t = 0; t < InterGeo->getNumGeometries(); ++t)//On parcourt les polygons formant l'intersection entre les deux polygons j et k
					{
						const geos::geom::Geometry * GeoTemp = InterGeo->getGeometryN(t);

						//std::cout << GeoTemp->getGeometryType() << " ";

						if(GeoTemp->getGeometryType() == "Polygon" && GeoTemp->getArea() > 0.001)
						{
							InterVec.push_back(GeoTemp->clone());
						}
						if(GeoTemp->getGeometryType() != "LineString")
							continue;

						geos::geom::CoordinateSequence * coords = GeoTemp->getCoordinates();
						geos::geom::Point * P1 = factory->createPoint(coords->getAt(0));
						geos::geom::Point * P2 = factory->createPoint(coords->getAt(1));

						for(int p = 0; p < InterGeo->getNumGeometries(); ++p)//On regarde quels sont les polygons qui touchent les linestring
						{
							if(t == p)
								continue;
							const geos::geom::Geometry * GeoTemp2 = InterGeo->getGeometryN(p);							

							if(GeoTemp2->getGeometryType() != "Polygon" || GeoTemp2->getArea() < 0.001)
								continue;

							if(P1->intersects(GeoTemp2))//Si le premier point du linestring touche un polygon
							{
								geos::geom::CoordinateSequence * tempcoords = new geos::geom::CoordinateArraySequence;
								tempcoords->add(coords->getAt(0));

								geos::geom::CoordinateSequence * test = geos::operation::distance::DistanceOp::nearestPoints(CurrGeoE, P1);
								geos::geom::Point * P = factory->createPoint(test->getAt(0));
								tempcoords->add(geos::operation::distance::DistanceOp::nearestPoints(InterGeo, P)->getAt(0));

								LineVec.push_back(factory->createLineString(tempcoords));//On crée une linestring entre le point qui touche un polygon et son projeté sur le CityGML
							}
							if(P2->intersects(GeoTemp2))//Si le second point du linestring touche un polygon
							{
								geos::geom::CoordinateSequence * tempcoords = new geos::geom::CoordinateArraySequence;
								tempcoords->add(coords->getAt(1));

								geos::geom::CoordinateSequence * test = geos::operation::distance::DistanceOp::nearestPoints(CurrGeoE, P2);
								geos::geom::Point * P = factory->createPoint(test->getAt(0));
								tempcoords->add(geos::operation::distance::DistanceOp::nearestPoints(InterGeo, P)->getAt(0));

								//geos::geom::Coordinate C = test->getAt(0);
								//C.x = 2 * C.x - P1->getX();
								//C.y = 2 * C.y - P1->getY();
								//tempcoords->add(C);

								//tempcoords->add(test->getAt(0));

								//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test_3" + std::to_string(p), Geo2, Geo, factory->createLineString(tempcoords));
								//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test_4" + std::to_string(p), GeoTemp2, InterGeo, factory->createLineString(tempcoords));
								
								LineVec.push_back(factory->createLineString(tempcoords));//On crée une linestring entre le point qui touche un polygon et son projeté sur le CityGML
							}
						}
					}

					geos::geom::Geometry* GeoLines = factory->createGeometryCollection(LineVec);

					geos::geom::Geometry * InterGeo2 = factory->createGeometryCollection(InterVec);

					geos::geom::Geometry * Res = overlaySnapRounded(InterGeo2, GeoLines);	//Pour découper les polygons avec les lignes de GeoLines				

					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test0" , Geo2, GeoLines, InterGeo2);

					//if(!Res->isEmpty())
					//	Res = Res->buffer(0.1);

					NewShape3[k] = Geo2->difference(Res);
					NewShape3[j] = Geo->difference(Res);

					/*for(int z = 0; z < Res->getNumGeometries(); ++z)
					{
						std::cout << Res->getGeometryN(z)->getGeometryType() << std::endl;
						std::cout << Res->getGeometryN(z)->getArea() << std::endl;
						std::cout << Res->getGeometryN(z)->isEmpty() << std::endl;
						std::cout << Res->getGeometryN(z)->isValid() << std::endl;
					}

					std::cout << "Geo : \n";
					for(int z = 0; z < Geo->getNumGeometries(); ++z)
					{
						std::cout << Geo->getGeometryN(z)->getGeometryType() << std::endl;
						std::cout << Geo->getGeometryN(z)->getArea() << std::endl;
						std::cout << Geo->getGeometryN(z)->isEmpty() << std::endl;
						std::cout << Geo->getGeometryN(z)->isValid() << std::endl;
					}*/

					
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "Diff1" , Geo2, Geo, factory->createEmptyGeometry());
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "Diff2" , Geo2, Geo, Res);
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "Diff3" , NewShape3[j], Geo, Res);
					
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "Diff4" , Geo->symDifference(Res), Geo, Res);

					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test2" , Geo2, Geo, InterGeo2);

					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test3" , Geo2, Geo, Res);						

					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test4" , NewShape3[k], NewShape3[j], GeoLines);

					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test5" , InterGeo2, factory->createEmptyGeometry(), GeoLines);
						
					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test6" , InterGeo2, factory->createEmptyGeometry(), Res);
					
					//Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "test1" , Geo2, Shape->getGeometryN(Link.second[i][j]), Shape->getGeometryN(Link.second[i][k]));					

					Geo = NewShape3[j];
				}
			}
			Save3GeometryRGB("Im_" + std::to_string(i) + "_6", CurrPolyE, factory->createGeometryCollection(NewShape3), factory->createEmptyGeometry());//Intersection avec le cityGML

			/*for(int j = 0; j < NewShape3.size() - 1; ++j)
			{
				const geos::geom::Geometry * Geo1 = NewShape3[j];
				for(int k = j + 1; k < NewShape3.size(); ++k)
				{
					const geos::geom::Geometry * Geo2 = NewShape3[k];
					if(!Geo1->intersects(Geo2))
						continue;
					try
					{
						const geos::geom::Geometry * inter = Geo1->intersection(Geo2);
						if(int n = inter->getNumGeometries() > 0)
						{
							for(int t = 0; t < n; ++t)
							{
								if(inter->getGeometryN(t)->getGeometryType() == "Polygon")
									std::cout << "Polygon " << j << " " << k << std::endl;
							}
						}
					}
					catch(std::exception& e)
					{
						std::cout << j << "; " << k << "   " <<  e.what() << '\n';
					}
				}
			}*/
		}



		/*for(int i = 0; i < EnveloppeCity->getNumGeometries(); ++i)//On parcourt tous les polygons de l'enveloppe
		{
			const geos::geom::Polygon * CurrPolyE = dynamic_cast<const geos::geom::Polygon*>(EnveloppeCity->getGeometryN(i));
			//Le but de ces lignes est de convertir le polygon avec son exterior ring et ses trous en un ensemble de geometry contenant ceux ci sans qu'ils soient encore liés. On peut ainsi parcourir seulement les arrêtes du polygon sans la notion d'intérieur
			std::vector<geos::geom::Geometry *> PolyToGeo;
			PolyToGeo.push_back(CurrPolyE->getExteriorRing()->clone());
			for(int j = 0; j < CurrPolyE->getNumInteriorRing(); ++j)
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
					const geos::geom::Polygon * CurrPolyS = dynamic_cast<const geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j]));
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
						delete CurrPoint;
					}

					std::vector<geos::geom::Geometry*> * PolyInt = new std::vector<geos::geom::Geometry*>;//Contiendra les interior rings modifiés du polygon courant
					//std::vector<geos::geom::Geometry*> PolyInt; 

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
							delete CurrPoint;
						}
						PolyInt->push_back(factory->createLinearRing(CoordInt));
					}
					Geos2.push_back(factory->createPolygon(factory->createLinearRing(CoordExt), PolyInt));

					delete CurrPolyS;
					delete CurrPolyExt;
				}
				cpt++;
				Save3GeometryRGB("Poly" + std::to_string(cpt), CurrPolyE, factory->createEmptyGeometry(), factory->createEmptyGeometry());//Affiche l'enveloppe du citygml
				Save3GeometryRGB("Poly" + std::to_string(cpt) + "_", factory->createEmptyGeometry(), UnionPolyS, factory->createEmptyGeometry());//Affiche l'enveloppe du shape
				Save3GeometryRGB("Poly" + std::to_string(cpt) + "_test1", CurrPolyE, factory->createGeometryCollection(Geos), factory->createEmptyGeometry());//Affiche l'enveloppe avec les polygons du shape associés
				Save3GeometryRGB("Poly" + std::to_string(cpt) + "_test2", CurrPolyE, factory->createGeometryCollection(Geos2), factory->createEmptyGeometry());//Affiche l'enveloppe avec les polygons du shape modifiés

				delete UnionPolyS;
				delete CurrGeoS;
				delete CurrPolyE;
				for(auto& it : Geos) delete it;
				for(auto& it : Geos2) delete it;
				for(auto& it : PolyToGeo) delete it;
			}
			delete CurrGeoE;
		}*/
		
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

		//SaveGeometry("Enveloppe_City_Simplified", geos::simplify::TopologyPreservingSimplifier::simplify(EnveloppeCity, 2).get());
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

	void Algo::generateLOD1(geos::geom::Geometry * Shape, std::vector<std::pair<double, double>> Hauteurs)//Hauteurs : Liste les hauteurs et Zmin des polygons de ShapeGeo
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		if(Shape == NULL)
		{
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

		BuildLOD1FromGEOS(ShapeResWithoutHolesSimp, Hauteurs2);
	}
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////

