// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "algo.hpp"
////////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif // _WIN32

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
#include "geos/simplify/TopologyPreservingSimplifier.h"
#include "geos/operation/distance/DistanceOp.h"
#include "geos/operation/buffer/BufferOp.h"


#include "src/gui/osg/osgGDAL.hpp"
#include "osg/Geode"
#include "osg/Geometry"
////////////////////////////////////////////////////////////////////////////////
#if defined(__unix) || defined(__APPLE__)
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
    Algo::Algo()
        : m_model(nullptr)
    {

    }
    ////////////////////////////////////////////////////////////////////////////////
    Algo::~Algo()
    {
        delete m_model;
    }
	////////////////////////////////////////////////////////////////////////////////
	double Scale = 1; //Définit le zoom des images sauvegardées, avec Scale = 1 <=> 1 mètre/pixel = précision au mètre près.

	/**
	* @brief Projette les toits du CityObject sélectionné sur le plan (xy)
	* @param obj CityObject sélectionné
	* @param roofProj un set de Polygon, le résultat de la projection
	* @param heightmax Enregistre le Zmax des murs du bâtiment
    * @param heightmin Enregistre le Zmin des murs du bâtiment
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
	void GetFootprint(citygml::CityObject* obj, OGRMultiPolygon * FootPrint, double * heightmax, double * heightmin)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
		if(obj->getType() == citygml::COT_RoofSurface) //Si surface de toit : COT_RoofSurface COT_WallSurface
		{
			for(citygml::Geometry* Geom : obj->getGeometries())
			{
				for(citygml::Polygon* Poly : Geom->getPolygons())
                {
					OGRPolygon * OgrPoly = new OGRPolygon;
					OGRLinearRing * OgrRing = new OGRLinearRing;
					for(TVec3d Vertices : Poly->getExteriorRing()->getVertices())
					{
						OgrRing->addPoint(Vertices.x - offset_.x, Vertices.y - offset_.y);
						
						if(Vertices.z > *heightmax)
							*heightmax = Vertices.z;
						//std::cout << " (x,y) = (" << Vertices.x<< "," << Vertices.y<< ")" << std::endl;
					}
					OgrRing->closeRings();
                    if(OgrRing->getNumPoints() > 3)//Le polygone ne sera créé qu'à partir de 4 points
                    {
                        OgrPoly->addRingDirectly(OgrRing);
                        if(OgrPoly->IsValid())
                            FootPrint->addGeometryDirectly(OgrPoly); // on récupere le polygone
                    }
				}
			}
		}
		else if(obj->getType() == citygml::COT_WallSurface)//Remplissage de la hauteur min des murs (correspondant au "sol" du bâtiment)
		{
			for(citygml::Geometry* Geom : obj->getGeometries())
			{
				for(citygml::Polygon* Poly : Geom->getPolygons())
				{
					for(TVec3d Vertices : Poly->getExteriorRing()->getVertices())
					{
						if(Vertices.z < *heightmin || *heightmin == -1)
							*heightmin = Vertices.z;
					}
				}
			}
		}
		for(citygml::CityObject* Obj : obj->getChildren())//Pour descendre dans le bâtiment jusqu'à arriver aux Wall/Roofs
		{
			GetFootprint(Obj, FootPrint, heightmax, heightmin);
		}
	}

	/**
	* @brief Convertit les données citygml de projection au sol en multipolygon pour GEOS
	* @param roofPoints Contient les polygones à convertir en Geos
	*/
	geos::geom::MultiPolygon * ConvertToGeos(PolySet &roofPoints)
	{
		//std::cout << "Debut de ConvertToGeos" << std::endl;

		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		std::vector<geos::geom::Geometry*>* Polys = new std::vector<geos::geom::Geometry*>();
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

			P = factory->createPolygon(shell, nullptr);

			Polys->push_back(P);
		}
		//std::cout << "Creation du Multipolygon ..." << std::endl;

		geos::geom::MultiPolygon *MP = factory->createMultiPolygon(Polys);

		//std::cout << "MultiPolygon cree, il contient : "<< MP->getNumGeometries() << " Polygons." << std::endl;

		if(MP->getNumGeometries() == 0)
			return nullptr;

		return MP;
	}

	/**
	* @brief Trace la ligne entre deux points de coordonnées (x1,y1) et (x2,y2) par [Bresenham] 
	[Bresenham]: http://fr.wikipedia.org/wiki/Algorithme_de_trac%C3%A9_de_segment_de_Bresenham "Algorithme de tracé de segment de Bresenham"
	* @param x1 Coordonnée x du premier point
	* @param y1 Coordonnée y du premier point
	* @param x2 Coordonnée x du second point
	* @param y2 Coordonnée y du second point
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
	* @brief Sauvegarde une image en niveaux de gris dans un fichier pgm
	* @param name Nom de l'image à enregistrer
	* @param Im Tableau de int contenant les pixels de l'image
	* @param width Largeur de l'image
	* @param height Hauteur de l'image
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
	* @param name Nom de l'image à enregistrer
	* @param ImR Tableau de int contenant le canal rouge des pixels de l'image
	* @param ImR Tableau de int contenant le canal vert des pixels de l'image
	* @param ImR Tableau de int contenant le canal bleu des pixels de l'image
	* @param width Largeur de l'image
	* @param height Hauteur de l'image 
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
	* @brief Parcourt de manière récursive toutes les geometries de Geo pour récupérer toutes leurs informations
	* @param Im Tableau de int contenant les pixels de l'image
	* @param width Largeur de l'image
	* @param height Hauteur de l'image
	* @param Xmin Permet de situer en X l'image par rapport à l'espace de données initial
	* @param Ymin Permet de situer en Y l'image par rapport à l'espace de données initial
	*/
	void SaveRecursiveGeometry(const geos::geom::Geometry * Geo, int* Im, int height, int width, int Xmin, int Ymin)
	{
		if(Geo->getNumGeometries() > 1)//Si Geo est encore un ensemble de geometry, on continue de parcourir ses fils
		{
			for(size_t i = 0; i < Geo->getNumGeometries(); ++i)
				SaveRecursiveGeometry(Geo->getGeometryN(i), Im, height, width, Xmin, Ymin);
		}
		else
		{
			const geos::geom::CoordinateSequence *coord;
			const geos::geom::Polygon *p = dynamic_cast<const geos::geom::Polygon*>(Geo);
			if(p)//Si p est un polygon
			{
				coord = p->getExteriorRing()->getCoordinates();
                for(size_t j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
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

                for(size_t k = 0; k < p->getNumInteriorRing(); k++) //On parcourt les holes du polygon
				{
					delete coord;
					coord = p->getInteriorRingN(k)->getCoordinates();
                    for(size_t j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
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
			else//si p n'est pas un polygon
			{
				coord = Geo->getCoordinates();
				if(coord->size() != 0)
				{
                    for(size_t j = 0; j < coord->size() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
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
	void SaveRecursiveGeometry(const OGRGeometry * Geo, int* Im, int height, int width, int Xmin, int Ymin)
	{
		if(Geo->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon)//Si Geo est encore un ensemble de geometry, on continue de parcourir ses fils
		{
			const OGRMultiPolygon * GeoCollection = dynamic_cast<const OGRMultiPolygon*>(Geo);
			for(int i = 0; i < GeoCollection->getNumGeometries(); ++i)
				SaveRecursiveGeometry(GeoCollection->getGeometryRef(i), Im, height, width, Xmin, Ymin);
		}
		else if(Geo->getGeometryType() == OGRwkbGeometryType::wkbPolygon || Geo->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon25D)
		{
			const OGRPolygon *Polygon = dynamic_cast<const OGRPolygon*>(Geo);
			const OGRLineString *ExtRing = Polygon->getExteriorRing();

			for(int j = 0; j < ExtRing->getNumPoints() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
			{
				OGRPoint* Point = new OGRPoint;
				ExtRing->getPoint(j, Point);
				int x1 = Scale * (Point->getX() - Xmin);
				int y1 = Scale * (Point->getY() - Ymin);
				delete Point;
				Point = new OGRPoint;
				ExtRing->getPoint(j+1, Point);
				int x2 = Scale * (Point->getX() - Xmin);
				int y2 = Scale * (Point->getY() - Ymin);
				delete Point;

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
			for(int k = 0; k < Polygon->getNumInteriorRings(); k++) //On parcourt les holes du polygon
			{
				const OGRLineString *IntRing = Polygon->getInteriorRing(k);
				for(int j = 0; j < IntRing->getNumPoints() - 1; j++) //Répétition du premier point à la fin donc pas besoin de tout parcourir
				{
					OGRPoint* Point = new OGRPoint;
					IntRing->getPoint(j, Point);
					int x1 = Scale * (Point->getX() - Xmin);
					int y1 = Scale * (Point->getY() - Ymin);
					delete Point;
					Point = new OGRPoint;
					ExtRing->getPoint(j+1, Point);
					int x2 = Scale * (Point->getX() - Xmin);
					int y2 = Scale * (Point->getY() - Ymin);
					delete Point;

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
			delete Polygon;
		}
		else
			std::cout << "Geometry n'est pas un polygon" << std::endl;
	}

	/**
	* @brief Sauvegarde la geometry dans un fichier image
	* @param name Nom de l'image à enregistrer
	* @param G Geometry à enregistrer dans un fichier image
	*/
	void SaveGeometry(std::string name, const geos::geom::Geometry* G)
	{	
		const geos::geom::CoordinateSequence *coord;

		coord = G->getCoordinates();

		int Xmin = -1, Ymin = -1, Xmax = 0, Ymax = 0;

        for(size_t i = 0; i < coord->size(); i++)
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

	void SaveGeometry(std::string name, const OGRMultiPolygon* G)
	{	
		OGREnvelope * Envelope = new OGREnvelope;
		G->getEnvelope(Envelope);

		int Xmin = Envelope->MinX, Ymin = Envelope->MinY, Xmax = Envelope->MaxX, Ymax = Envelope->MaxY;

		//std::cout << Xmin << ";" << Xmax << "  " << Ymin << ";" << Ymax << std::endl;

		int width = Xmax - Xmin + 1;
		int height = Ymax - Ymin + 1;

		int* Im = new int[width * height];
		memset(Im, 1, width*height*sizeof(int));

		int NbGeo = G->getNumGeometries();
		for(int i = 0; i < NbGeo; i++)
		{
			const OGRGeometry * Geo = G->getGeometryRef(i);

			SaveRecursiveGeometry(Geo, Im, height, width, Xmin, Ymin);
		}
		SaveImage(name, Im, width, height);

		delete [] Im;
	}

	/**
	* @brief Sauvegarde la geometry dans un fichier shape
	* @param name Nom du shape à enregistrer
	* @param G Geometry à enregistrer
	*/
	void SaveGeometrytoShapeRecursive(const geos::geom::Geometry* Geo, OGRLayer * Layer)
	{
		if(Geo->getNumGeometries() > 1)//Si Geo est encore un ensemble de geometry, on continue de parcourir ses fils
		{
			for(size_t i = 0; i < Geo->getNumGeometries(); ++i)
				SaveGeometrytoShapeRecursive(Geo->getGeometryN(i), Layer);
		}
		else if(Geo->getGeometryType() == "Polygon")
		{
			TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;

			const geos::geom::Polygon *p = dynamic_cast<const geos::geom::Polygon*>(Geo);

			OGRPolygon * Polygon = new OGRPolygon;
			OGRLinearRing * ExtRing = new OGRLinearRing;

			const geos::geom::CoordinateSequence * coord = p->getExteriorRing()->getCoordinates();
			for(size_t j = 0; j < coord->size(); j++)
			{
				double x = coord->getAt(j).x + offset_.x;
				double y = coord->getAt(j).y + offset_.y;
				
				ExtRing->addPoint(x, y);
			}
			ExtRing->closeRings();
			Polygon->addRingDirectly(ExtRing);
			
			for(size_t k = 0; k < p->getNumInteriorRing(); k++)//On parcourt les holes du polygon
			{
				delete coord;
				coord = p->getInteriorRingN(k)->getCoordinates();
				OGRLinearRing * IntRing = new OGRLinearRing;
				for(size_t j = 0; j < coord->size(); j++)
				{
					double x = coord->getAt(j).x + offset_.x;
					double y = coord->getAt(j).y + offset_.y;
				
					IntRing->addPoint(x, y);
				}
				IntRing->closeRings();
				Polygon->addRingDirectly(IntRing);
			}

			delete coord;
			OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());
			Feature->SetGeometry(Polygon);
			Layer->CreateFeature(Feature);

			OGRFeature::DestroyFeature(Feature);
		}
	}
	void SaveGeometrytoShape(std::string name, const geos::geom::Geometry* G)
	{
		const char * DriverName = "ESRI Shapefile";
		OGRSFDriver * Driver;

		OGRRegisterAll();
		Driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(DriverName);
		if( Driver == NULL )
		{
			printf( "%s driver not available.\n", DriverName );
			return;
		}
		OGRDataSource * DS;
		name = name + ".shp";
		
		remove(name.c_str());
		DS = Driver->CreateDataSource(name.c_str(), NULL);

		OGRLayer * Layer = DS->CreateLayer("Layer1");

		geos::geom::Geometry * Geo = G->clone();

		for(int i = 0; i < Geo->getNumGeometries(); ++i)
		{
			SaveGeometrytoShapeRecursive(Geo->getGeometryN(i), Layer);
		}

		delete Geo;

		OGRDataSource::DestroyDataSource(DS);
	}

	/**
	* @brief Sauvegarde 3 geometry dans un même fichier image dans les trois canaux RGB
	* @param name Nom de l'image à enregistrer
	* @param G1 Geometry à enregistrer dans le canal rouge du fichier image
	* @param G2 Geometry à enregistrer dans le canal vert du fichier image
	* @param G3 Geometry à enregistrer dans le canal bleu du fichier image
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

            for(size_t i = 0; i < coord->size(); i++)
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
	* @param MP Ensemble de polygones sur lequel on va générer une enveloppe
	*/
	geos::geom::Geometry * GetEnveloppe(geos::geom::MultiPolygon * MP)//Attention, le cas où les polygon à unir ont déjà des trous fonctionne peut être mal
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		//std::cout << "Mise en place de l'union des polygons" << std::endl;

		geos::geom::Geometry* ResUnion = factory->createEmptyGeometry();//MP->getGeometryN(0)->clone();

        for(size_t i = 0; i < MP->getNumGeometries(); i++)	//On parcourt tous les polygons que l'on veut unir
		{
			try	//On vérifie qu'il n'y ait pas d'exceptions faisant planter le logiciel
			{
				geos::geom::Geometry* tmp = ResUnion;
				ResUnion = ResUnion->Union(MP->getGeometryN(i)); //On fait l'union avant de la vérifier
				delete tmp;

				std::vector<geos::geom::Geometry*>* Polys = new std::vector<geos::geom::Geometry*>(); //Vecteur contenant les différents polygones de l'union au fur et à mesure

                for(size_t j = 0; j < ResUnion->getNumGeometries(); j++) //L'union peut être constitué de plusieurs polygons disjoints
				{
					geos::geom::CoordinateArraySequence tempcoord;
					const geos::geom::CoordinateSequence *coordGeo = ResUnion->getGeometryN(j)->getCoordinates(); //On récupère la liste des points de la géométrie courante

					double FirstPoint[2];
					FirstPoint[0] = -1;

					bool isHole = false;
					geos::geom::LinearRing * shell;
					std::vector<geos::geom::Geometry*> * Holes = new std::vector<geos::geom::Geometry*>; //Vecteur contenant tous les polygones à l'intérieur du premier, qui sont donc considérés comme des trous

                    for(size_t k = 0; k < coordGeo->size(); k++) //On parcourt tous les points pour retrouver ceux qui apparaissent deux fois et qui définissent un polygon qu'il faut extraire
					{
						double x = coordGeo->getAt(k).x;
						double y = coordGeo->getAt(k).y;

						tempcoord.add(coordGeo->getAt(k));//Pour avoir le z en plus

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

								//break;//////////////////////// A COMMENTER POUR AVOIR LES TROUS DANS LES POLYGONS
							}
							else
							{
								if(tempcoord.size() > 3)
								{
									geos::geom::LinearRing* Hole = factory->createLinearRing(tempcoord);
									geos::geom::Polygon* polyArea = factory->createPolygon(*Hole, std::vector<geos::geom::Geometry*>());

									if(polyArea->getArea() > 1)
										Holes->push_back(static_cast<geos::geom::Geometry*>(Hole));

									delete polyArea;
								}
								FirstPoint[0] = -1;
								tempcoord.clear();
							}
						}
					}
					delete coordGeo;
					geos::geom::Polygon *P;

					if(Holes->size() == 0)
						P = factory->createPolygon(shell, nullptr);
					else
						P = factory->createPolygon(shell, Holes);

					Polys->push_back(P);
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
	OGRMultiPolygon * GetEnveloppe(OGRMultiPolygon * MP)
	{
		//std::cout << "Mise en place de l'union des polygons" << std::endl;

		OGRGeometry* ResUnion = new OGRMultiPolygon;

        ResUnion = MP->UnionCascaded();

        //OGRGeometry** Polys = new OGRGeometry*[MP->getNumGeometries()]; //Vecteur contenant les différents polygones de l'union au fur et à mesure

        //for(int i = 0; i < MP->getNumGeometries(); ++i)	//On parcourt tous les polygons que l'on veut unir
        //{

           // Polys[i] = MP->getGeometryRef(i)->clone();
            /*try	//On vérifie qu'il n'y ait pas d'exceptions faisant planter le logiciel
            {
                OGRGeometry* tmp = ResUnion;
                ResUnion = tmp->Union(MP->getGeometryRef(i)); //On fait l'union avant de la vérifier
                delete tmp;
				std::vector<OGRGeometry*>* Polys = new std::vector<OGRGeometry*>(); //Vecteur contenant les différents polygones de l'union au fur et à mesure

                //int *a;
                //OGRGeometryFactory::organizePolygons(Polys, 10, a, true);

                if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon)
                {
                    OGRMultiPolygon * GeoCollection = dynamic_cast<OGRMultiPolygon*>(ResUnion->clone());
                    for(size_t j = 0; j < GeoCollection->getNumGeometries(); ++j) //L'union peut être constitué de plusieurs polygons disjoints
                    {
                        OGRGeometry * Geometry = GeoCollection->getGeometryRef(j);
                        if(Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon || Geometry->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon25D)
						{
							//OGRPolygon * Polygon = dynamic_cast<OGRPolygon*>(Geometry);
							//OGRLinearRing * Ring = Polygon->getExteriorRing();

							geos::geom::CoordinateArraySequence tempcoord;
                            const geos::geom::CoordinateSequence *coordGeo = GeoCollection->getGeometryRef(j)->getCoordinates(); //On récupère la liste des points de la géométrie courante

							double FirstPoint[2];
							FirstPoint[0] = -1;

							bool isHole = false;
							geos::geom::LinearRing * shell;
							std::vector<geos::geom::Geometry*> * Holes = new std::vector<geos::geom::Geometry*>; //Vecteur contenant tous les polygones à l'intérieur du premier, qui sont donc considérés comme des trous

							for(size_t k = 0; k < coordGeo->size(); ++k) //On parcourt tous les points pour retrouver ceux qui apparaissent deux fois et qui définissent un polygon qu'il faut extraire
							{
								double x = coordGeo->getAt(k).x;
								double y = coordGeo->getAt(k).y;

								tempcoord.add(coordGeo->getAt(k));//Pour avoir le z en plus

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

										//break;//////////////////////// A COMMENTER POUR AVOIR LES TROUS DANS LES POLYGONS
									}
									else
									{
										if(tempcoord.size() > 3)
										{
											geos::geom::LinearRing* Hole = factory->createLinearRing(tempcoord);
											geos::geom::Polygon* polyArea = factory->createPolygon(*Hole, std::vector<geos::geom::Geometry*>());

											if(polyArea->getArea() > 1)
												Holes->push_back(static_cast<geos::geom::Geometry*>(Hole));

											delete polyArea;
										}
										FirstPoint[0] = -1;
										tempcoord.clear();
									}
								}
							}
							delete coordGeo;
							geos::geom::Polygon *P;

							if(Holes->size() == 0)
								P = factory->createPolygon(shell, nullptr);
							else
								P = factory->createPolygon(shell, Holes);

							Polys->push_back(P);
						}
					}
				}
				delete ResUnion;
                ResUnion = factory->createMultiPolygon(Polys);
			}
			catch(std::exception& e)
			{
				std::cout << e.what() << '\n';
            }*/
        //}

        //int *NbValidGeom;
        //ResUnion = OGRGeometryFactory::organizePolygons(Polys, MP->getNumGeometries(), NbValidGeom);

		//On travaille avec des OGRMultiPolygon pour avoir un format universel, il faut donc transformer la geometry en collection.
		if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon)//La geometry est en fait un ensemble de geometry : plusieurs bâitments
		{
			OGRMultiPolygon * GeoCollection = dynamic_cast<OGRMultiPolygon*>(ResUnion->clone());
			return GeoCollection;
		}
		else if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon25D)//La geometry est en fait un seul polygon : un seul bâtiment
		{
			OGRMultiPolygon * GeoCollection = new OGRMultiPolygon;
			GeoCollection->addGeometryDirectly(ResUnion);
			return GeoCollection;
		}
		return nullptr;
	}

	/**
	* @brief Teste la geometry Geo avec Geo1 et Geo2 pour déterminer de laquelle est la plus proche. Retourne 1 pour Geo1 et 2 pour Geo2
	* @param Geo Geometry que l'on veut situer
	* @param Geo1 Première geometry de comparaison
	* @param Geo2 Première geometry de comparaison
	*/
	int GetNearestGeo(const geos::geom::Geometry * Geo, const geos::geom::Geometry * Geo1, const geos::geom::Geometry * Geo2)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		geos::geom::CoordinateSequence * Coords = Geo->getCoordinates();

		double D1 = 0;
		double D2 = 0;

        for(size_t i = 0; i < Coords->getSize(); ++i)
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
	* @param Polys Vecteur contenant les polygones à découper
	* @param Lines Vecteur contenant les lignes permettant de découper les polygones
	*/
	std::vector<geos::geom::Geometry*> SplitPolygon(std::vector<geos::geom::Geometry*> Polys, std::vector<geos::geom::Geometry*> Lines)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		std::vector<geos::geom::Geometry*> Res;

        for(size_t i = 0; i < Polys.size(); ++i)
		{
			geos::geom::Geometry * Poly = Polys[i];

			bool Test = false; //Passe à true si le poly intersecte une ligne
            for(size_t j = 0; j < Lines.size(); ++j)
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
                for(size_t k = 0; k < Coords->size() - 1; ++k)
				{
					geos::geom::CoordinateSequence* EdgeCoord = new geos::geom::CoordinateArraySequence;
					EdgeCoord->add(Coords->getAt(k));
					EdgeCoord->add(Coords->getAt(k+1));
					geos::geom::LineString * Edge = factory->createLineString(EdgeCoord);

					//Save3GeometryRGB("TESTLine" + std::to_string(k), Poly, Edge, Poly);

					geos::geom::Geometry * Intersection = Edge->intersection(Line);
					geos::geom::CoordinateSequence * CoordsIntersection = Intersection->getCoordinates();
					if(!Edge->intersects(Line) || (CoordsIntersection->getAt(0).x == EdgeCoord->getAt(0).x && CoordsIntersection->getAt(0).y == EdgeCoord->getAt(0).y))
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
						geos::geom::Coordinate CoordInter = CoordsIntersection->getAt(0);
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
						geos::geom::Coordinate CoordInter = CoordsIntersection->getAt(0);

						Poly2->add(CoordInter);
						Poly1->add(CoordInter);
						if(EdgeCoord->getAt(1).x != CoordInter.x || EdgeCoord->getAt(1).y != CoordInter.y)
							Poly1->add(EdgeCoord->getAt(1));
					}
					delete CoordsIntersection;
					delete Intersection;
					delete Edge;
				}
				
				if(Poly1->size() > 2 && (Poly1->getAt(0).x != Poly1->getAt(Poly1->size()-1).x || Poly1->getAt(0).y != Poly1->getAt(Poly1->size()-1).y))
				{
					Poly1->add(Poly1->getAt(0));//Pour boucler le ring si c'est nécessaire
				}
				if(Poly1->size() > 3)
				{
					geos::geom::Polygon * Pol1 = factory->createPolygon(factory->createLinearRing(Poly1), nullptr);
					Res.push_back(Pol1);
				}
				if(Poly2->size() > 2)
				{
					Poly2->add(Poly2->getAt(0)); //Pour boucler
					geos::geom::Polygon * Pol2 = factory->createPolygon(factory->createLinearRing(Poly2), nullptr);
					Res.push_back(Pol2);
				}
				//Save3GeometryRGB("TEST2", Pol1, factory->createPolygon(factory->createLinearRing(Poly2), nullptr), Pol1);
			}
			if(!Test)
				Res.push_back(Poly);
		}
		//SaveGeometry("TEST3", factory->createGeometryCollection(Res));
		return Res;
	}

	/**
	* @brief Convertit les données GEOS issues du shape en LOD1 de CityGML
	* @param Geos Contient les geometries représentant des bâtiments issus du fichier .shp
	* @param Hauteurs Contient les hauteurs (Zmin, Zmax) de chaque bâtiment
	*/
	citygml::CityModel* ConvertShapeToLOD1(geos::geom::Geometry * Geos, std::vector<std::pair<double, double>> Hauteurs)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
		citygml::CityModel* model = new citygml::CityModel;

		for(size_t i = 0; i < Geos->getNumGeometries(); ++i)
		{
			TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
			const geos::geom::Geometry * TempGeo = Geos->getGeometryN(i);
			if(TempGeo->getGeometryType() != "Polygon")
				continue;

            citygml::Geometry* Wall = new citygml::Geometry("GeoWall_Building_" + std::to_string(i), citygml::GT_Wall, 1);
            citygml::Geometry* Roof = new citygml::Geometry("GeoRoof_Building_" + std::to_string(i), citygml::GT_Roof, 1);

			double heightmax = Hauteurs[i].second + Hauteurs[i].first;
			double heightmin = Hauteurs[i].second;

			citygml::Polygon * PolyRoof = new citygml::Polygon("PolyRoof");
			citygml::LinearRing * RingRoof = new citygml::LinearRing("RingRoof",true);			

			geos::geom::CoordinateSequence * Coords = TempGeo->getCoordinates();	//Récupère tous les points de la geometry

            for(size_t j = 0; j < Coords->size() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
			{
				citygml::Polygon * PolyWall = new citygml::Polygon("PolyWall_" + std::to_string(j));
				citygml::LinearRing * RingWall = new citygml::LinearRing("RingWall_" + std::to_string(j),true);

				double x1 = Coords->getAt(j).x + offset_.x;
				double y1 = Coords->getAt(j).y + offset_.y;

				RingRoof->addVertex(TVec3d(x1, y1, heightmax));

				double x2 = Coords->getAt(j+1).x + offset_.x;
				double y2 = Coords->getAt(j+1).y + offset_.y;

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

        citygml::ExporterCityGML exporter("test.citygml");
        exporter.exportCityModel(*model);

		std::cout << std::endl << "LOD1 cree.\n";

		citygml::ParserParams params;
		model->finish(params);

		return model;
	}
	citygml::CityObject* Algo::ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;

		citygml::CityObject* BuildingCO = new citygml::Building("LOD1_" + name);
		citygml::CityObject* WallCO = new citygml::WallSurface("LOD1_" + name);
		citygml::CityObject* RoofCO = new citygml::RoofSurface("LOD1_" + name);

		for(int i = 0; i < Enveloppe->getNumGeometries(); ++i)
		{
			const OGRPolygon * Poly = dynamic_cast<const OGRPolygon *>(Enveloppe->getGeometryRef(i));
			if(!Poly || !Poly->IsValid())
				continue;

            citygml::Geometry* Wall = new citygml::Geometry(name + "_Wall_" + std::to_string(i), citygml::GT_Wall, 1);
			citygml::Geometry* Roof = new citygml::Geometry(name + "_Roof_" + std::to_string(i), citygml::GT_Roof, 1);

			citygml::Polygon * PolyRoof = new citygml::Polygon(name + "_PolyRoof_" + std::to_string(i));
			citygml::LinearRing * RingRoof = new citygml::LinearRing(name + "_RingRoof_" + std::to_string(i), true);

			const OGRLinearRing * ExtRing = Poly->getExteriorRing();

			for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
			{
				citygml::Polygon * PolyWall = new citygml::Polygon(name + "_PolyWall_" + std::to_string(i) + "_" + std::to_string(j));
				citygml::LinearRing * RingWall = new citygml::LinearRing(name + "_RingWall_" + std::to_string(i) + "_" + std::to_string(j), true);

				OGRPoint * point = new OGRPoint;
				ExtRing->getPoint(j, point);
				double x1 = point->getX() + offset_.x;
				double y1 = point->getY() + offset_.y;
				delete point;

				RingRoof->addVertex(TVec3d(x1, y1, *heightmax));

                point = new OGRPoint;
				ExtRing->getPoint(j+1, point);
				double x2 = point->getX() + offset_.x;
				double y2 = point->getY() + offset_.y;
                delete point;

				RingWall->addVertex(TVec3d(x1, y1, *heightmin));
				RingWall->addVertex(TVec3d(x2, y2, *heightmin));
				RingWall->addVertex(TVec3d(x2, y2, *heightmax));
				RingWall->addVertex(TVec3d(x1, y1, *heightmax));

				PolyWall->addRing(RingWall);
				Wall->addPolygon(PolyWall);
			}
			PolyRoof->addRing(RingRoof);
			Roof->addPolygon(PolyRoof);

			WallCO->addGeometry(Wall);
			RoofCO->addGeometry(Roof);
			BuildingCO->insertNode(WallCO);
			BuildingCO->insertNode(RoofCO);

			//std::cout << "Avancement creation LOD1 : " << i+1 << "/" << Geometry->getNumGeometries() << "\r" << std::flush;
		}

		return BuildingCO;
	}
	citygml::CityObject* Algo::ConvertLOD1ToCityGML(std::string name, geos::geom::Geometry * Geometry, double * heightmax, double * heightmin)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;

		citygml::CityObject* BuildingCO = new citygml::Building("LOD1_" + name);
		citygml::CityObject* WallCO = new citygml::WallSurface("LOD1_" + name);
		citygml::CityObject* RoofCO = new citygml::RoofSurface("LOD1_" + name);
		citygml::CityObject* GroundCO = new citygml::GroundSurface("LOD1_" + name);

		for(int i = 0; i < Geometry->getNumGeometries(); ++i)
		{
			geos::geom::Geometry * TempGeo =  Geometry->getGeometryN(i)->clone();
			geos::geom::CoordinateSequence * Coords;
			geos::geom::Polygon * PolyGeos = dynamic_cast<geos::geom::Polygon *>(TempGeo);

			citygml::Geometry* Wall = new citygml::Geometry(name + "_Wall_" + std::to_string(i), citygml::GT_Wall, 1);
			citygml::Geometry* Roof = new citygml::Geometry(name + "_Roof_" + std::to_string(i), citygml::GT_Roof, 1);
			citygml::Geometry* Ground = new citygml::Geometry(name + "_Ground_" + std::to_string(i), citygml::GT_Ground, 0);

			citygml::Polygon * PolyRoof = new citygml::Polygon(name + "_PolyRoof_" + std::to_string(i));//Polygone représentant le roof
			citygml::Polygon * PolyGround = new citygml::Polygon(name + "_PolyGround_" + std::to_string(i));//Polygone représentant l'emprise au sol
			citygml::LinearRing * RingRoof = new citygml::LinearRing(name + "_RingRoof_" + std::to_string(i), true);//Ring extérieur
			citygml::LinearRing * RingGround = new citygml::LinearRing(name + "_RingGround_" + std::to_string(i), true);//Ring extérieur

			if(!PolyGeos || PolyGeos->getNumInteriorRing() == 0) //Si ce n'est pas un polygone (geometry simple) ou si c'est un polygone sans trou, on parcourt simplement les coordonnées pour les copier dans des données CityGML
			{
				Coords = TempGeo->getCoordinates();
				for(size_t j = 0; j < Coords->size(); ++j)
				{
					citygml::Polygon * PolyWall = new citygml::Polygon(name + "_PolyWall_" + std::to_string(i) + "_" + std::to_string(j));
					citygml::LinearRing * RingWall = new citygml::LinearRing(name + "_RingWall_" + std::to_string(i) + "_" + std::to_string(j), true);

					double x1 = Coords->getAt(j).x + offset_.x;
					double y1 = Coords->getAt(j).y + offset_.y;

					RingRoof->addVertex(TVec3d(x1, y1, *heightmax));
					RingGround->addVertex(TVec3d(x1,y1, *heightmin));


					if(j < Coords->size() - 1) //Il n'y a pas besoin de construire de mur à partir du dernier point puisqu'il correspond au premier qui a déjà été traité
					{
						double x2 = Coords->getAt(j+1).x + offset_.x;
						double y2 = Coords->getAt(j+1).y + offset_.y;

						RingWall->addVertex(TVec3d(x1, y1, *heightmin));
						RingWall->addVertex(TVec3d(x2, y2, *heightmin));
						RingWall->addVertex(TVec3d(x2, y2, *heightmax));
						RingWall->addVertex(TVec3d(x1, y1, *heightmax));

						PolyWall->addRing(RingWall);
						Wall->addPolygon(PolyWall);
					}
				}
				PolyRoof->addRing(RingRoof);
				Roof->addPolygon(PolyRoof);
				PolyGround->addRing(RingGround);
				Ground->addPolygon(PolyGround);
			}
			else //C'est un polygone avec trou(s) : un exterior ring et des interior ring
			{
				const geos::geom::LineString * ExtRing = PolyGeos->getExteriorRing();
				Coords = ExtRing->getCoordinates();
				for(size_t j = 0; j < Coords->size(); ++j)
				{
					citygml::Polygon * PolyWall = new citygml::Polygon(name + "_PolyWall_" + std::to_string(i) + "_" + std::to_string(j));
					citygml::LinearRing * RingWall = new citygml::LinearRing(name + "_RingWall_" + std::to_string(i) + "_" + std::to_string(j), true);

					double x1 = Coords->getAt(j).x + offset_.x;
					double y1 = Coords->getAt(j).y + offset_.y;

					RingRoof->addVertex(TVec3d(x1, y1, *heightmax));
					RingGround->addVertex(TVec3d(x1,y1, *heightmin));

					if(j < Coords->size() - 1) //Il n'y a pas besoin de construire de mur à partir du dernier point puisqu'il correspond au premier qui a déjà été traité
					{
						double x2 = Coords->getAt(j+1).x + offset_.x;
						double y2 = Coords->getAt(j+1).y + offset_.y;

						RingWall->addVertex(TVec3d(x1, y1, *heightmin));
						RingWall->addVertex(TVec3d(x2, y2, *heightmin));
						RingWall->addVertex(TVec3d(x2, y2, *heightmax));
						RingWall->addVertex(TVec3d(x1, y1, *heightmax));

						PolyWall->addRing(RingWall);
						Wall->addPolygon(PolyWall);
					}
				}
				PolyRoof->addRing(RingRoof);//Ajout de l'exterior ring au PolyRoof
				PolyGround->addRing(RingGround);

				for(size_t k = 0; k < PolyGeos->getNumInteriorRing(); ++k)
				{
					citygml::LinearRing * RingRoofInt = new citygml::LinearRing(name + "_RingRoofInt_" + std::to_string(i) + "_" + std::to_string(k), false);//Ring intérieur
					citygml::LinearRing * RingGroundInt = new citygml::LinearRing(name + "_RingGroundInt_" + std::to_string(i) + "_" + std::to_string(k), false);//Ring intérieur

					const geos::geom::LineString * IntRingGeos = PolyGeos->getInteriorRingN(k);
					Coords = nullptr;
					delete Coords;
					Coords = IntRingGeos->getCoordinates();
					for(size_t j = 0; j < Coords->size(); ++j)
					{
						citygml::Polygon * PolyWall = new citygml::Polygon(name + "_PolyWall_" + std::to_string(i) + "_" + std::to_string(j));
						citygml::LinearRing * RingWall = new citygml::LinearRing(name + "_RingWall_" + std::to_string(i) + "_" + std::to_string(j), true);

						double x1 = Coords->getAt(j).x + offset_.x;
						double y1 = Coords->getAt(j).y + offset_.y;

						RingRoofInt->addVertex(TVec3d(x1, y1, *heightmax));
						RingGroundInt->addVertex(TVec3d(x1, y1, *heightmin));

						if(j < Coords->size() - 1) //Il n'y a pas besoin de construire de mur à partir du dernier point puisqu'il correspond au premier qui a déjà été traité
						{
							double x2 = Coords->getAt(j+1).x + offset_.x;
							double y2 = Coords->getAt(j+1).y + offset_.y;

							RingWall->addVertex(TVec3d(x1, y1, *heightmin));
							RingWall->addVertex(TVec3d(x2, y2, *heightmin));
							RingWall->addVertex(TVec3d(x2, y2, *heightmax));
							RingWall->addVertex(TVec3d(x1, y1, *heightmax));

							PolyWall->addRing(RingWall);
							Wall->addPolygon(PolyWall);
						}
					}
					PolyRoof->addRing(RingRoofInt);//Ajout d'un interior ring au PolyRoof
					PolyGround->addRing(RingGroundInt);//Ajout d'un interior ring au PolyRoof
				}
				Roof->addPolygon(PolyRoof);
				Ground->addPolygon(PolyGround);
			}
			WallCO->addGeometry(Wall);
			RoofCO->addGeometry(Roof);
			GroundCO->addGeometry(Ground);
			BuildingCO->insertNode(WallCO);
			BuildingCO->insertNode(GroundCO);

			delete TempGeo;
			delete Coords;
			//std::cout << "Avancement creation LOD1 : " << i+1 << "/" << Geometry->getNumGeometries() << "\r" << std::flush;
		}

		return BuildingCO;
	}

	/**
	* @brief Convertit une geometry (MultiPolygon) GEOS en geometry CityGML.
	* @param name Nom de l'objet CityGML créé
	* @param Geometry Geometry à convertir en objet CityGML
	* @param Zmin Permet de situer le LOD0 dans l'espace
	*/
	citygml::Geometry* Algo::ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
        citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
        for(int i = 0; i < Geometry->getNumGeometries(); ++i)
		{
			citygml::Polygon * Poly = new citygml::Polygon("Polygon");
			citygml::LinearRing * Ring = new citygml::LinearRing("ExteriorRing", true);

			if(Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon)
				continue;

			OGRPolygon * Polygon =  dynamic_cast<OGRPolygon*>(Geometry->getGeometryRef(i)->clone());

			OGRLinearRing * ExtRing = Polygon->getExteriorRing();

			for(int j = 0; j < ExtRing->getNumPoints(); ++j)
			{
				OGRPoint * point = new OGRPoint;
				ExtRing->getPoint(j, point);
				Ring->addVertex(TVec3d(point->getX() + offset_.x, point->getY() + offset_.y, *heightmin));
			}
			Poly->addRing(Ring);
            for(int k = 0; k < Polygon->getNumInteriorRings(); ++k)
			{
				citygml::LinearRing * IntRingCityGML = new citygml::LinearRing("InteriorRing", false);//False pour signifier que le linearring correspond à un interior ring
				OGRLinearRing * IntRing = Polygon->getInteriorRing(k);

                for(int j = 0; j < IntRing->getNumPoints(); ++j)
				{
					OGRPoint * point = new OGRPoint;
					IntRing->getPoint(j, point);
					IntRingCityGML->addVertex(TVec3d(point->getX() + offset_.x, point->getY() + offset_.y, *heightmin));
				}
				Poly->addRing(IntRingCityGML);
			}
			Geom->addPolygon(Poly);

			delete Polygon;
		}
		return Geom;
	}	
	citygml::Geometry* Algo::ConvertLOD0ToCityGML(std::string name, geos::geom::Geometry * Geometry, double Zmin)
	{
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
        citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
		for(size_t i = 0; i < Geometry->getNumGeometries(); ++i)
		{
			citygml::Polygon * Poly = new citygml::Polygon("Polygon");
			citygml::LinearRing * Ring = new citygml::LinearRing("ExteriorRing", true);

			geos::geom::Geometry * TempGeo =  Geometry->getGeometryN(i)->clone();
			geos::geom::CoordinateSequence * Coords;
			geos::geom::Polygon * PolyGeos = dynamic_cast<geos::geom::Polygon *>(TempGeo);

			if(!PolyGeos || PolyGeos->getNumInteriorRing() == 0)
			{
				Coords = TempGeo->getCoordinates();
				for(size_t j = 0; j < Coords->size(); ++j)
				{
					Ring->addVertex(TVec3d(Coords->getAt(j).x + offset_.x, Coords->getAt(j).y + offset_.y, Zmin));
				}
				Poly->addRing(Ring);
				Geom->addPolygon(Poly);
			}
			else
			{
				const geos::geom::LineString * ExtRing = PolyGeos->getExteriorRing();
				Coords = ExtRing->getCoordinates();
				for(size_t j = 0; j < Coords->size(); ++j)
				{
					Ring->addVertex(TVec3d(Coords->getAt(j).x + offset_.x, Coords->getAt(j).y + offset_.y, Zmin));
				}
				Poly->addRing(Ring);
				for(size_t k = 0; k < PolyGeos->getNumInteriorRing(); ++k)
				{
					citygml::LinearRing * IntRing = new citygml::LinearRing("InteriorRing", false);//False pour signifier que le linearring correspond à un interior ring
					const geos::geom::LineString * IntRingGeos = PolyGeos->getInteriorRingN(k);
					Coords = nullptr;
					delete Coords;
					Coords = IntRingGeos->getCoordinates();
					for(size_t j = 0; j < Coords->size(); ++j)
					{
						IntRing->addVertex(TVec3d(Coords->getAt(j).x + offset_.x, Coords->getAt(j).y + offset_.y, Zmin));
					}
					Poly->addRing(IntRing);
				}
				Geom->addPolygon(Poly);
			}
			delete TempGeo;
			delete Coords;
		}
		return Geom;
	}

	/**
	* @brief Relie deux ensembles de geometry en assignant aux geometry de l'une celles qui lui correspondent dans l'autre
	* @param Shape Geometries issues du fichier shape
	* @param Enveloppe Geometries issues du fichier CityGML
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
            const geos::geom::Geometry * GeoS = Shape->getGeometryN(i);
			for(int j = 0; j < NbGeoE; ++j)
			{
                const geos::geom::Geometry * GeoE = Enveloppe->getGeometryN(j);

				double Area = GeoS->intersection(GeoE)->getArea();

				if((Area/GeoS->getArea()) > 0.1 || (Area/GeoE->getArea()) > 0.1)
				{
					//std::cout << "Area : " << GeoS->getArea() << " ; " << GeoE->getArea() << " ; " << Area << std::endl;
					Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
				}
			}
			if((i+1)%100 == 0)
				std::cout << "Avancement de LinkGeos : " << i+1 << " / " << NbGeoS << "\r" << std::flush;
		}

		std::cout << "\n";
		//std::cout << Res.first.size() << " , " << NbGeoS << std::endl << Res.second.size() << " , " << NbGeoE << std::endl;
		return Res;
	}

	/**
	* @brief Calcule la distance de Hausdorff unidirectionnelle entre un nuage de points et un triangle
	* @param Points Correspond au nuage de points
	* @param Geo Correspond au triangle sur lequel seront projetés les points
	*/
	double Hausdorff(geos::geom::CoordinateSequence * Points, const geos::geom::Geometry * Geo)
	{
		double D = 0;
        for(size_t i = 0; i < Points->size(); ++i)
		{
			double D_1 = 10000;
			geos::geom::Coordinate P0 = Points->getAt(i);
            for(size_t j = 0; j < Geo->getNumGeometries(); ++j)
			{
				double D_2;
				const geos::geom::Geometry * Triangle = Geo->getGeometryN(j);
				if(Triangle->getNumPoints() > 4 || Triangle->getArea() < 0.01)//Si la géometry courante n'est pas un triangle
				{
					continue;
				}
				geos::geom::CoordinateSequence * TempCoord = Triangle->getCoordinates();
				geos::geom::Coordinate P1 = TempCoord->getAt(0); //Point du triangle
				geos::geom::Coordinate P2 = TempCoord->getAt(1);
				geos::geom::Coordinate P3 = TempCoord->getAt(2);
				delete TempCoord;

				geos::geom::Coordinate P1P0(P0.x - P1.x, P0.y - P1.y, P0.z - P1.z); //Vecteur P1P0
				geos::geom::Coordinate P2P0(P0.x - P2.x, P0.y - P2.y, P0.z - P2.z);
				geos::geom::Coordinate P3P0(P0.x - P3.x, P0.y - P3.y, P0.z - P3.z);
				double nP1P0 = sqrt(P1P0.x * P1P0.x + P1P0.y * P1P0.y + P1P0.z * P1P0.z); //Norme du vecteur P1P0
				double nP2P0 = sqrt(P2P0.x * P2P0.x + P2P0.y * P2P0.y + P2P0.z * P2P0.z);
				double nP3P0 = sqrt(P3P0.x * P3P0.x + P3P0.y * P3P0.y + P3P0.z * P3P0.z);

				if(nP1P0 == 0 || nP2P0 == 0 || nP3P0 == 0) // Si le point P0 est confondu avec l'un des points du triangle
				{
					D_1 = 0;
					break;
				}

				geos::geom::Coordinate P1P2(P2.x - P1.x, P2.y - P1.y, P2.z - P1.z); //Vecteur P1P2
				geos::geom::Coordinate P1P3(P3.x - P1.x, P3.y - P1.y, P3.z - P1.z);
				geos::geom::Coordinate P2P3(P3.x - P2.x, P3.y - P2.y, P3.z - P2.z);
                //double nP1P2 = sqrt(P1P2.x * P1P2.x + P1P2.y * P1P2.y + P1P2.z * P1P2.z); //Norme du vecteur P1P2
                //double nP1P3 = sqrt(P1P3.x * P1P3.x + P1P3.y * P1P3.y + P1P3.z * P1P3.z);
                //double nP2P3 = sqrt(P2P3.x * P2P3.x + P2P3.y * P2P3.y + P2P3.z * P2P3.z);

				geos::geom::Coordinate Np(P1P2.y * P1P3.z - P1P2.z * P1P3.y, P1P2.z * P1P3.x - P1P2.x * P1P3.z, P1P2.x * P1P3.y - P1P2.y * P1P3.x); //Normal du triangle
				double nNp = sqrt(Np.x * Np.x + Np.y * Np.y + Np.z * Np.z);

				double cosa = (P1P0.x * Np.x + P1P0.y * Np.y + P1P0.z * Np.z)/(nP1P0 * nNp); //Calcul du cosinus de langle a entre Np et P1P0

				double nP0P0_ = nP1P0 * cosa;
				geos::geom::Coordinate P0P0_(- nP0P0_ * Np.x / nNp, - nP0P0_ * Np.y / nNp, - nP0P0_ * Np.z / nNp); //Vecteur P0P0_, P0_ étant le projeté de P0 sur le plan du triangle

				geos::geom::Coordinate P0_(P0.x + P0P0_.x, P0.y + P0P0_.y, P0.z + P0P0_.z); // Position du projeté de P0 sur le plan du triangle

				double s, t;//Coordonnées barycentriques du point P0_ par rapport au point P1 et aux vecteurs P1P2 et P1P3

				t = (P1.y * P1P2.x - P1.x * P1P2.y + P1P2.y * P0_.x - P1P2.x * P0_.y) / (P1P2.y * P1P3.x - P1P2.x * P1P3.y);
				s = (P0_.x - P1.x - t * P1P3.x) / P1P2.x;
				
				if(t >= 0 && s >= 0 && t + s <= 1)//Le projeté est dans le triangle
				{
					D_2 = nP0P0_;
					if(D_1 > D_2)
						D_1 = D_2;
				}
				else//Le projeté est en dehors du triangle
				{
					//On va donc le comparer aux trois arrêtes du triangle en le projetant à nouveau sur celles-ci
					geos::geom::Coordinate P0_P1(P1.x - P0_.x, P1.y - P0_.y, P1.z - P0_.z); //Vecteur P0_P1
					geos::geom::Coordinate P0_P2(P2.x - P0_.x, P2.y - P0_.y, P2.z - P0_.z);
					geos::geom::Coordinate P0_P3(P3.x - P0_.x, P3.y - P0_.y, P3.z - P0_.z);
					double nP0_P1 = sqrt(P0_P1.x * P0_P1.x + P0_P1.y * P0_P1.y + P0_P1.z * P0_P1.z); //Norme du vecteur P0_P1
					double nP0_P2 = sqrt(P0_P2.x * P0_P2.x + P0_P2.y * P0_P2.y + P0_P2.z * P0_P2.z);
                    //double nP0_P3 = sqrt(P0_P3.x * P0_P3.x + P0_P3.y * P0_P3.y + P0_P3.z * P0_P3.z);

					//Sur P1P2 :
					geos::geom::Coordinate Temp(P0_P2.y * P0_P1.z - P0_P2.z * P0_P1.y, P0_P2.z * P0_P1.x - P0_P2.x * P0_P1.z, P0_P2.x * P0_P1.y - P0_P2.y * P0_P1.x);
					geos::geom::Coordinate R(Temp.y * P1P2.z - Temp.z * P1P2.y, Temp.z * P1P2.x - Temp.x * P1P2.z, Temp.x * P1P2.y - Temp.y * P1P2.x); //Direction de P0_ -> P0__
					double nR = sqrt(R.x * R.x + R.y * R.y + R.z * R.z);

					double cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z)/(nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

					double nP0_P0__ = nP0_P1 * cosg;
					geos::geom::Coordinate P0_P0__(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
					geos::geom::Coordinate P0__(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle
					
					double u = (P0__.x - P1.x) / (P1P2.x); //Position relative de P0__ sur le segment P1P2

					if(u <= 0)
						D_2 = nP1P0; //P0 est le plus proche de P1
					else if(u >= 1)
						D_2 = nP2P0; //P0 est le plus proche de P2
					else
						D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

					if(D_1 > D_2)
						D_1 = D_2;

					//Sur P1P3 :
					Temp = geos::geom::Coordinate(P0_P3.y * P0_P1.z - P0_P3.z * P0_P1.y, P0_P3.z * P0_P1.x - P0_P3.x * P0_P1.z, P0_P3.x * P0_P1.y - P0_P3.y * P0_P1.x);
					R = geos::geom::Coordinate(Temp.y * P1P3.z - Temp.z * P1P3.y, Temp.z * P1P3.x - Temp.x * P1P3.z, Temp.x * P1P3.y - Temp.y * P1P3.x); //Direction de P0_ -> P0__
					nR = sqrt(R.x * R.x + R.y * R.y + R.z * R.z);

					cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z)/(nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

					nP0_P0__ = nP0_P1 * cosg;
					P0_P0__ = geos::geom::Coordinate(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
					P0__ = geos::geom::Coordinate(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

					u = (P0__.x - P1.x) / (P1P3.x); //Position relative de P0__ sur le segment P1P3

					if(u <= 0)
						D_2 = nP1P0; //P0 est le plus proche de P1
					else if(u >= 1)
						D_2 = nP3P0; //P0 est le plus proche de P3
					else
						D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

					if(D_1 > D_2)
						D_1 = D_2;

					//Sur P2P3 :
					Temp = geos::geom::Coordinate(P0_P3.y * P0_P2.z - P0_P3.z * P0_P2.y, P0_P3.z * P0_P2.x - P0_P3.x * P0_P2.z, P0_P3.x * P0_P2.y - P0_P3.y * P0_P2.x);
					R = geos::geom::Coordinate(Temp.y * P2P3.z - Temp.z * P2P3.y, Temp.z * P2P3.x - Temp.x * P2P3.z, Temp.x * P2P3.y - Temp.y * P2P3.x); //Direction de P0_ -> P0__
					nR = sqrt(R.x * R.x + R.y * R.y + R.z * R.z);

					cosg = (P0_P2.x * R.x + P0_P2.y * R.y + P0_P2.z * R.z)/(nP0_P2 * nR); //Calcul du cosinus de langle g entre R et P0_P2

					nP0_P0__ = nP0_P2 * cosg;
					P0_P0__ = geos::geom::Coordinate(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
					P0__ = geos::geom::Coordinate(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

					u = (P0__.x - P2.x) / (P2P3.x); //Position relative de P0__ sur le segment P2P3

					if(u <= 0)
						D_2 = nP2P0; //P0 est le plus proche de P2
					else if(u >= 1)
						D_2 = nP3P0; //P0 est le plus proche de P3
					else
						D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

					if(D_1 > D_2)
						D_1 = D_2;
				}
			}
			if(D_1 > D)
				D = D_1;
		}
		return D;
	}

	/**
	* @brief Calcule la distance de Hausdorff entre deux bâtiments composés de triangles : méthode basée sur "3D Distance from a Point to a Triangle", Mark W.Jones, 1995 (DistancePointTriangle.pdf).
	* @param Geo1 Geometrie correspondant au premier bâtiment
	* @param Geo2 Geometrie correspondant au second bâtiment
	*/
	double DistanceHausdorff(const geos::geom::Geometry * Geo1, const geos::geom::Geometry * Geo2)
	{
		double D12 = 0;//Distance de Geo1 à Geo2
		double D21 = 0;//Distance de Geo2 à Geo1

		geos::geom::CoordinateSequence * Points1 = Geo1->getCoordinates();
		geos::geom::CoordinateSequence * Points2 = Geo2->getCoordinates();
		D12 = Hausdorff(Points1, Geo2);
		D21 = Hausdorff(Points2, Geo1);

		//std::cout << std::max(D12, D21) << std::endl;

		return std::max(D12, D21);
	}

	/**
	* @brief Compare deux ensembles de geometries en retournant les liens entre leurs polygones et l'information sur ces liens : si un polygone se retrouve dans les deux ensembles de geometries, dans un seul ou s'il a été modifié
	* @param Geo1 Premier ensemble de geometries qui ont été unies : deux triangles voisins sont réunis en un rectangle par exemple
	* @param Geo2 Second ensemble de geometries qui ont été unies
	* @param Geo1P Premier ensemble de geometries non unies
	* @param Geo2P Second ensemble de geometries non unies
	*/
	std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > CompareGeos(geos::geom::Geometry * Geo1, geos::geom::Geometry * Geo2, geos::geom::Geometry * Geo1P, geos::geom::Geometry * Geo2P)
	{
		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Res; //Enregistre les liens entre les polygones. Pour un polygone donnée de Geo1, si il est en lien avec un de Geo2, l'indice sera précédé de -1 ou -2 pour inchangé/changé

		size_t NbGeo1 = Geo1->getNumGeometries();
		size_t NbGeo2 = Geo2->getNumGeometries();

		Res.first.resize(NbGeo1);
		Res.second.resize(NbGeo2);

		double moyenne = 0;
		int cpt = 0;

		for(size_t i = 0; i < NbGeo1; ++i)
		{
			geos::geom::Geometry * SGeo1 = Geo1->getGeometryN(i)->clone();

			double Zmax1, Zmin1;
			for(size_t j = 0; j < SGeo1->getNumPoints(); ++j)
			{
				geos::geom::CoordinateSequence * TempCoord = SGeo1->getCoordinates();
				double z = TempCoord->getAt(j).z;
				delete TempCoord;
				if(j == 0)
				{
					Zmax1 = z;
					Zmin1 = z;
				}
				else 
				{
					if(Zmax1 < z)
						Zmax1 = z;
					if(Zmin1 > z)
						Zmin1 = z;
				}
			}

			for(size_t j = 0; j < NbGeo2; ++j)
			{
				geos::geom::Geometry * SGeo2 = Geo2->getGeometryN(j)->clone();

				/*double Zmax2, Zmin2;
				for(int k = 0; k < SGeo2->getNumPoints(); ++k)
				{
					geos::geom::CoordinateSequence * TempCoord = SGeo2->getCoordinates();
					double z = TempCoord->getAt(k).z;
					delete TempCoord;
					if(k == 0)
					{
						Zmax2 = z;
						Zmin2 = z;
					}
					else 
					{
						if(Zmax2 < z)
							Zmax2 = z;
						if(Zmin2 > z)
							Zmin2 = z;
					}
				}
				double val3 = std::abs(Zmin1 - Zmin2) + std::abs(Zmax1 - Zmax2);*/

				geos::geom::Geometry * tmp = SGeo1->intersection(SGeo2);
				double Area = tmp->getArea();
				delete tmp;
				double val1 = Area/SGeo1->getArea();
				double val2 = Area/SGeo2->getArea();

				if(val1 > 0.99 && val2 > 0.99 && SGeo1->getArea() - Area < 5 && SGeo2->getArea() - Area < 5)//Les polygons sont identiques
				{
					if(DistanceHausdorff(Geo1P->getGeometryN(i), Geo2P->getGeometryN(j)) < 5)//Si la différence de hauteur est inférieure à 5m, et si la distance de Hausdorff entre les deux bâtimetns est inférieure à 5m.
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
					else//Batiment modifie en "hauteur"
					{
						Res.first[i].push_back(-2);
						Res.second[j].push_back(-2);
						Res.first[i].push_back(j);
						Res.second[j].push_back(i);
						break;
					}
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

			std::cout << "Avancement de CompareGeos : " << i + 1 << " / " << NbGeo1 << "\r" << std::flush;
		}
		std::cout << "\n";
		std::cout << "Moyenne = " << moyenne/cpt << std::endl;

		return Res;
	}

	/**
	* @brief Convertit un polygon avec des trous en un ensemble de geometries distinctes
	* @param Geo Polygone à "éclater"
	*/
	geos::geom::Geometry * ConvertToSimpleGeom(const geos::geom::Geometry * Geo)
	{
		if(Geo->getGeometryType() != "Polygon")
			return Geo->clone();

		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		geos::geom::Polygon * Poly = dynamic_cast<geos::geom::Polygon *>(Geo->clone());

		std::vector<geos::geom::Geometry *> PolyToGeo;
		PolyToGeo.push_back(Poly->getExteriorRing()->clone());
        for(size_t j = 0; j < Poly->getNumInteriorRing(); ++j)
		{
			PolyToGeo.push_back(Poly->getInteriorRingN(j)->clone());
		}

		geos::geom::Geometry * GeoRes = factory->createGeometryCollection(PolyToGeo);
		return GeoRes;
	}

	/**
	* @brief Extrapole les Z de la geometry 1 à partir de la seconde qui lui est coplanaire et dont les Z sont connus
	* @param Geo Geometrie qui contient les points dont on veut calculer la coordonnée z 
	* @param GeoZ Geometrie qui est coplanaire à Geo. Ses points ont des valeurs de coordonnée z connues.
	*/
	geos::geom::Geometry * CalculeZ(const geos::geom::Geometry * Geo, geos::geom::Geometry * GeoZ)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		//On commence par récupérer trois points de GeoZ non alignés pour obtenir l'équation paramétrique du plan formé par cette geometry
		geos::geom::CoordinateSequence * CoordsGeo = Geo->getCoordinates();
		geos::geom::CoordinateSequence * CoordsGeoZ = GeoZ->getCoordinates();
		geos::geom::Coordinate A(CoordsGeoZ->getAt(0));
		geos::geom::Coordinate B;
		geos::geom::Coordinate C;

		TVec3d AB;
		TVec3d AC;
		TVec3d AM;
		int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
        for(size_t i = 1; i < GeoZ->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une répétition du premier
		{
			if(test == 0)
			{				
				B = CoordsGeoZ->getAt(i);
				if(A.x != B.x || A.y != B.y)
				{
					++test;// A est bien différent de B
					AB.x = B.x - A.x;
					AB.y = B.y - A.y;
					AB.z = B.z - A.z;
				}
			}
			else if(test == 1)
			{
				C = CoordsGeoZ->getAt(i);
				if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y))
				{
					++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
					AC.x = C.x - A.x;
					AC.y = C.y - A.y;
					AC.z = C.z - A.z;
					break;
				}
			}
		}
		if(test != 2)
		{
			std::cout << "Erreur lors de la creation du plan. \n";
            delete CoordsGeoZ;
            delete CoordsGeo;
			return nullptr;
		}

		//On va parcourir tous les points M de Geo pour lesquels on cherche le Z. M appartient au plan ABC et vérifie donc : AM = sAB + tAC
		//On va se servir des coordonnées x et y qui sont toutes connues pour déterminer s et t qui nous permettront ensuite de calculer le z du point M

		geos::geom::CoordinateSequence * ResCoords = new geos::geom::CoordinateArraySequence;

        for(size_t i = 0; i < Geo->getNumPoints(); ++i)
		{
			geos::geom::Coordinate M(CoordsGeo->getAt(i));
			double s, t;

			t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
			s = (M.x - A.x - t * AC.x) / AB.x;

			ResCoords->add(geos::geom::Coordinate(M.x, M.y, A.z + s * AB.z + t * AC.z));

			if(i > 0 && i < Geo->getNumPoints()-1 && CoordsGeo->getAt(i).x == CoordsGeo->getAt(0).x && CoordsGeo->getAt(i).y == CoordsGeo->getAt(0).y && CoordsGeo->getAt(i).z == CoordsGeo->getAt(0).z)
			{
				//std::cout << "Holes" << std::endl;
				break; //Pour que les polygones avec des trous ne posent pas de problème, on supprime ces trous en s'arrêtant lorsque la boucle est finie (on retombe sur le premier point).
			}
		}

        delete CoordsGeoZ;
        delete CoordsGeo;

		if(ResCoords->size() > 3)
			return factory->createPolygon(factory->createLinearRing(ResCoords), nullptr);

		//Scale = 10;
		//SaveGeometry("Bati", Geo);
		//std::cout << "ResCoords vide dans CalculeZ" << std::endl;
		//std::cout << ResCoords->size() << "  " << Geo->getNumPoints() << std::endl;

        delete ResCoords;

		return nullptr;
	}

	/**
	* @brief Charge le CityGML et le découpe à l'aide des polygons Geos représentant les bâtiments
	* @param Batiments Contient la liste des geometries représentant les bâtiments à extruder.
	* @param InfoBatiments Contient les informations de ces batiments contenues dans le fichier shape.
	*/
	void ExtruderBatiments(geos::geom::Geometry * Batiments, std::vector<BatimentShape> InfoBatiments)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;

		//citygml::CityModel* model = new citygml::CityModel;

		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		std::vector<geos::geom::Geometry*> VecGeoRes;

		// create citygml exporter to append data into
		citygml::ExporterCityGML exporter("Batiments.citygml");
		exporter.initExport();

		for(size_t j = 0; j < Batiments->getNumGeometries(); ++j)
		{
			if(j%10 == 0)
					std::cout << "Avancement : " << j << "/" << Batiments->getNumGeometries() << " batiments ajoutes au CityGML.\r" << std::flush;

			const geos::geom::Geometry * Bati = Batiments->getGeometryN(j);

			std::vector<geos::geom::Geometry*>* VecGeo = new std::vector<geos::geom::Geometry*>();
			std::vector<double> Hauteurs; //Contiendra les Zmin de toutes les geometry de VecGeo pour savoir jusqu'où descendre les murs

			for(size_t i = 0; i < tiles.size(); i++)//On parcourt les tuiles du CityGML
			{
				citygml::CityModel* model = tiles[i]->getCityModel();
				
				for(citygml::CityObject* Building : model->getCityObjectsRoots())//On parcourt les bâtiments de la tuile
				{
					if(Building->getType() != citygml::COT_Building)
						continue;
					double heightmin = -1;//Contiendra le Zmin du bâtiment courant
					for(citygml::CityObject* object : Building->getChildren())//On parcourt d'abord tous les Wall pour calculer Zmin
					{
						if(object->getType() == citygml::COT_WallSurface)
						{
							std::vector<citygml::Geometry*> geoms = object->getGeometries();
							std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
							for(; itGeom != geoms.end(); ++itGeom) //pour toute les géométries ( /!\ g
							{
								std::vector<citygml::Polygon*> polys = (*itGeom)->getPolygons();
								std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
								for(; itPoly != polys.end(); ++itPoly) //Pour chaque polygone
								{
									citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
									const std::vector<TVec3d> vertices = ring->getVertices();
									std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
									for(; itVertices != vertices.end(); ++itVertices)//pour Chaque som
									{
										TVec3d point = *itVertices;
										if(point.z < heightmin || heightmin == -1)
											heightmin = point.z;
									}
								}
							}
						}
					}
					for(citygml::CityObject* object : Building->getChildren())//On parcourt les objets (Wall et Roof) du bâtiment
					{
						if(object->getType() == citygml::COT_RoofSurface)
						{
							for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
							{
								for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
								{
									if(PolygonCityGML->getExteriorRing()->getVertices().size() < 3)
										continue;
									geos::geom::CoordinateSequence * Coords = new geos::geom::CoordinateArraySequence;

									for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
									{
										Coords->add(geos::geom::Coordinate(Point.x - offset_.x, Point.y - offset_.y, Point.z));
									}
									Coords->add(Coords->getAt(0));

									geos::geom::Geometry * GeoCityGML = factory->createPolygon(factory->createLinearRing(Coords), nullptr);

									geos::geom::Geometry * Inter = GeoCityGML->intersection(Bati);

									for(size_t k = 0; k < Inter->getNumGeometries(); ++k)
									{
										const geos::geom::Geometry * Interpart = Inter->getGeometryN(k);
										if(!Interpart->isValid() || Interpart->isEmpty() || Interpart->getNumPoints() < 4 || Interpart->getArea() == 0.0)
											continue;

										//Save3GeometryRGB("test", Bati, GeoCityGML, Bati);
										//Save3GeometryRGB("test2", Bati, GeoCityGML, Interpart);

										geos::geom::Geometry * Ring = CalculeZ(Interpart, GeoCityGML);
										if(Ring != nullptr)
										{
											VecGeo->push_back(Ring);
											Hauteurs.push_back(heightmin);
										}
									}
									//delete Coords;
									delete GeoCityGML;
									//delete GeoRes;
									delete Inter;
								}
							}
						}
					}
				}
			}
			if(VecGeo->size() > 0)
			{
				//SaveGeometry("Batiment_" + std::to_string(j), factory->createGeometryCollection(VecGeo));

				VecGeoRes.push_back(factory->createGeometryCollection(VecGeo));

				//citygml::CityObject* BuildingCO = new citygml::Building("Building_" + std::to_string(j));
				citygml::CityObject* BuildingCO = new citygml::Building(InfoBatiments[j].ID);

				for(size_t i = 0; i < VecGeo->size(); ++i)
				{
                    citygml::Geometry* Roof = new citygml::Geometry("GeoRoof_Building_" + std::to_string(j)  + "_" + std::to_string(i), citygml::GT_Roof, 2);
					citygml::Polygon * PolyRoof = new citygml::Polygon("PolyRoof");
					citygml::LinearRing * RingRoof = new citygml::LinearRing("RingRoof",true);

                    citygml::Geometry* Ground = new citygml::Geometry("GeoGround_Building_" + std::to_string(j)  + "_" + std::to_string(i), citygml::GT_Ground, 2);
					citygml::Polygon * PolyGround = new citygml::Polygon("PolyGround");
					citygml::LinearRing * RingGround = new citygml::LinearRing("RingGround",true);

					geos::geom::CoordinateSequence * Coords = (*VecGeo)[i]->getCoordinates();

					for(size_t k = 0; k < Coords->size() - 1; ++k)
					{
						RingRoof->addVertex(TVec3d(Coords->getAt(k).x + offset_.x, Coords->getAt(k).y + offset_.y, Coords->getAt(k).z));
						RingGround->addVertex(TVec3d(Coords->getAt(k).x + offset_.x, Coords->getAt(k).y + offset_.y, Hauteurs[i]));

						int BuildWall = 0;//Comptera le nombre de polygones du toit contenant la ligne. S'il est supérieur à 1, cela signifique qu'il ne faut pas construire le mur

						for(size_t z = 0; z < VecGeo->size(); z++)//Pour ne pas construire de murs entre deux polygones voisins partageant une arrête
						{
							if(z == i)
								continue;
							geos::geom::CoordinateSequence * Coords2 = (*VecGeo)[z]->getCoordinates();
							for(size_t c = 0; c < Coords2->size(); ++c)
							{
								if(Coords->getAt(k).x == Coords2->getAt(c).x && Coords->getAt(k).y == Coords2->getAt(c).y && Coords->getAt(k).z == Coords2->getAt(c).z)
								{
									if(c > 0 && Coords->getAt(k+1).x == Coords2->getAt(c-1).x && Coords->getAt(k+1).y == Coords2->getAt(c-1).y && Coords->getAt(k+1).z == Coords2->getAt(c-1).z)
									{
										BuildWall++;
										break;
									}
									else if(c < Coords2->size() - 1 && Coords->getAt(k+1).x == Coords2->getAt(c+1).x && Coords->getAt(k+1).y == Coords2->getAt(c+1).y && Coords->getAt(k+1).z == Coords2->getAt(c+1).z)
									{
										BuildWall++;
										break;
									}
								}
							}
							delete Coords2;
							if(BuildWall > 0)
								break;
						}

						if(BuildWall > 0)
							continue;

                        citygml::Geometry* Wall = new citygml::Geometry("GeoWall_Building_" + std::to_string(j) + "_" + std::to_string(i) + "_" + std::to_string(k), citygml::GT_Wall, 2);
						citygml::Polygon * PolyWall = new citygml::Polygon("PolyWall");
						citygml::LinearRing * RingWall = new citygml::LinearRing("RingWall",true);

						RingWall->addVertex(TVec3d(Coords->getAt(k).x + offset_.x, Coords->getAt(k).y + offset_.y, Coords->getAt(k).z));
						RingWall->addVertex(TVec3d(Coords->getAt(k+1).x + offset_.x, Coords->getAt(k+1).y + offset_.y, Coords->getAt(k+1).z));
						RingWall->addVertex(TVec3d(Coords->getAt(k+1).x + offset_.x, Coords->getAt(k+1).y + offset_.y, Hauteurs[i]));
						RingWall->addVertex(TVec3d(Coords->getAt(k).x + offset_.x, Coords->getAt(k).y + offset_.y, Hauteurs[i]));

						PolyWall->addRing(RingWall);
						Wall->addPolygon(PolyWall);

						citygml::CityObject* WallCO = new citygml::WallSurface("Wall_" + std::to_string(i) + "_" + std::to_string(k));

						WallCO->addGeometry(Wall);
						//model->addCityObject(WallCO);
						BuildingCO->insertNode(WallCO);
					}
					PolyRoof->addRing(RingRoof);
					Roof->addPolygon(PolyRoof);

					citygml::CityObject* RoofCO = new citygml::RoofSurface("Roof_" + std::to_string(i));

					RoofCO->addGeometry(Roof);
					//model->addCityObject(RoofCO);
					BuildingCO->insertNode(RoofCO);

					PolyGround->addRing(RingGround);
					Ground->addPolygon(PolyGround);

					citygml::CityObject* GroundCO = new citygml::GroundSurface("Ground" + std::to_string(i));

					GroundCO->addGeometry(Ground);
					BuildingCO->insertNode(GroundCO);

					delete Coords;
				}
				BuildingCO->setAttribute("ID_shape", InfoBatiments[j].ID);
				exporter.appendCityObject(*BuildingCO);
				delete BuildingCO;

				//model->addCityObject(BuildingCO);
				//model->addCityObjectAsRoot(BuildingCO);
			}
		}
		if(VecGeoRes.size() > 0)
		{
			//SaveGeometry("BatimentsRes" , factory->createGeometryCollection(VecGeoRes));

            for(geos::geom::Geometry* geom: VecGeoRes)
            {
                delete geom;
            }
		}

        //exporter.exportCityModel(*model);
        exporter.endExport();

        std::cout << std::endl << "Fichier CityGML cree." << std::endl;

        //citygml::ParserParams params;
        //model->finish(params);

        //return model;
	}

	/**
	* @brief Génère le LOD0 du bâtiment lié à uri
	* @param uri Représente l'ID du bâtiment à traité.
	*/
	void Algo::generateLOD0(const URI& uri)
	{
		uri.resetCursor();
		std::cout << "URI : " << uri.getStringURI() << std::endl;
		citygml::CityObject* obj = app().getScene().getCityObjectNode(uri);

		if(obj)/////////////////////////////////// Traitement bâtiment par bâtiment 
		{
			std::cout << "Obj : " << obj->getId() << std::endl;

			log() << "GenerateLOD0 on "<< uri.getStringURI() << "\n";

			PolySet roofPoints;
			double heightmax = 0, heightmin = -1;
			projectRoof(obj,roofPoints, &heightmax, &heightmin);
			std::string name = obj->getId();

			//Scale = 10;

			geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
			//SaveGeometry(name + "_MP", GeosObj);
			geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
			
			//SaveGeometry(name + "_Enveloppe", Enveloppe);

			//Pour afficher le ground dans VCity
            citygml::Geometry* geom = ConvertLOD0ToCityGML(name, Enveloppe, heightmin);
			citygml::CityObject* obj2 = new citygml::GroundSurface("tmpObj");
			obj2->addGeometry(geom);
			obj->insertNode(obj2);
			std::cout << "Lod 0 exporte dans " << name << ".shp" << std::endl;

			SaveGeometrytoShape(name, Enveloppe);

			delete GeosObj;
			delete Enveloppe;
		}
#ifdef _WIN32
			_CrtDumpMemoryLeaks();
#endif // _WIN32
	}
	void Algo::generateLOD0(citygml::CityObject* obj)
	{
		std::cout << "TEST1" << std::endl;
		std::cout << obj->getId() << std::endl;
		
		std::cout << "TEST2" << std::endl;
		std::cout << obj->getTypeAsString() << std::endl;
		if(obj)
		{
			std::cout << "TEST3" << std::endl;
			log() << "GenerateLOD0 on "<< obj->getTypeAsString() << "\n";
			std::string name = obj->getId();
			PolySet roofPoints;
			double heightmax = 0, heightmin = -1;

			//projectRoof(obj,roofPoints, &heightmax, &heightmin);
			//geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
			//SaveGeometry(name + "_MP", GeosObj);
			//geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
			//SaveGeometry(name + "_Enveloppe", Enveloppe);
			
			OGRMultiPolygon * Footprint = nullptr;
			GetFootprint(obj, Footprint, &heightmax, &heightmin);
			SaveGeometry(name + "_MP", Footprint);

			OGRMultiPolygon * Enveloppe = GetEnveloppe(Footprint);
			SaveGeometry(name + "_Enveloppe", Enveloppe);

			//Pour afficher le ground dans VCity
            //citygml::Geometry* geom = new citygml::Geometry(obj->getId()+"_lod0", citygml::GT_Ground, 0);
			//geom = ConvertLOD0ToCityGML(name, Enveloppe, heightmin);
			//citygml::CityObject* obj2 = new citygml::GroundSurface("tmpObj");
			//obj2->addGeometry(geom);
			//obj->insertNode(obj2);

			/*citygml::Geometry* geom = new citygml::Geometry(obj->getId()+"_lod0", citygml::GT_Ground, 0);
			geom = ConvertLOD0ToCityGML(name, Enveloppe, heightmin);
			citygml::CityObject* obj2 = new citygml::GroundSurface("Footprint");
			obj2->addGeometry(geom);
			obj->insertNode(obj2);*/
			std::cout << "Lod 0 exporte en cityGML" << std::endl;

			//delete GeosObj;
			//delete Enveloppe;
		}
		std::cout << "TEST4" << std::endl;
	}
	/**
	* @brief Génère le LOD0 du bâtiment contenu dans obj
	* @param obj bâtiment courant
	* @param Enveloppe résultat de la fonction : un multipolygon correspondant au footprint de obj
	* @param heightmax Hauteur max des murs du bâtiment
	* @param heightmin Hauteur min des murs du bâtiment
	*/
	void Algo::generateLOD0(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin)
	{
		*heightmax = 0;
		*heightmin = -1;

		OGRMultiPolygon * Footprint = new OGRMultiPolygon;
		GetFootprint(obj, Footprint, heightmax, heightmin);

		*Enveloppe = GetEnveloppe(Footprint);

        //std::cout << "GDAL" << std::endl;
	}
	void Algo::generateLOD0(citygml::CityObject* obj, geos::geom::Geometry ** Enveloppe, double * heightmax, double * heightmin)
	{
		PolySet roofPoints;
		*heightmax = 0;
		*heightmin = -1;
		
		projectRoof(obj,roofPoints, heightmax, heightmin);

		geos::geom::MultiPolygon * Footprint = ConvertToGeos(roofPoints);

		*Enveloppe = GetEnveloppe(Footprint);

		//SaveGeometry("TEST1", Footprint);
		//SaveGeometry("TEST2", *Enveloppe);
	}

	/**
	* @brief Découpe les bâtiments de la scène qui sont issus d'un fichier CityGML à partie des bâtiments cadastraux contenus dans le fichier shape.
	* @param Shape Contient les bâtiments cadastraux
	* ]param InfoBatiments Contient les informations de ces batiments contenues dans le fichier shape.
	*/
	void Algo::DecoupeCityGML(geos::geom::Geometry * Shape, std::vector<BatimentShape> InfoBatiments)//LOD0 sur toute la scène + Comparaison entre CityGML et Cadastre
	{
        if(Shape == nullptr || Shape->isEmpty())
        {
            std::cout << "Erreur : Aucun fichier Shapefile n'a été trouvé." << std::endl;
            return;
        }
        if(InfoBatiments.size() == 0)
        {
            std::cout << "Erreur : Aucun bâtiment CityGML n'a été trouvé." << std::endl;
            return;
        }
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		geos::geom::Geometry * EnveloppeCity = nullptr;
		std::vector<const geos::geom::Geometry *> VecGeos;

        for(size_t i = 0; i < tiles.size(); i++)//Création de l'enveloppe city à partir des données citygml
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

					geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
					/////////////
                    for(size_t y = 0; y < GeosObj->getNumGeometries(); y++)
					{
						if(GeosObj->getGeometryN(y)->isValid())
							VecGeos.push_back(GeosObj->getGeometryN(y));
					}
					/////////////
					geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
					delete GeosObj;

					if(EnveloppeCity == nullptr)
						EnveloppeCity = Enveloppe;
					else
					{
						geos::geom::Geometry * tmp = EnveloppeCity;
						EnveloppeCity = EnveloppeCity->Union(Enveloppe);
						delete tmp;
						delete Enveloppe;
					}
				}
				cpt++;
				if(cpt%10 == 0)
					std::cout << "Avancement : " << cpt << "/" << objs.size() << " batiments traites.\r" << std::flush;
			}
			std::cout << std::endl;;
		}

		//geos::geom::Geometry * City = factory->createGeometryCollection(VecGeos); //Contient tous les polygons

		//Scale = 4;
		//Save3GeometryRGB("CityGML", EnveloppeCity, factory->createEmptyGeometry(), EnveloppeCity);
		//Save3GeometryRGB("CityGML_Shape", EnveloppeCity, Shape, EnveloppeCity);
		//return;

		////////////////////////////////////////////////////////////////////// Compare le cadastre et le CityGML

		if(Shape == nullptr || Shape->isEmpty())
		{
			std::cout << "Shape NULL. \n";
			return;
		}

		///////////// Relie les polygons du CityGML et du Cadastre
		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Link = LinkGeos(Shape, EnveloppeCity); //Le premier vector contient les polys d'enveloppe pour chaque Shape
		/////////////

        std::vector<geos::geom::Geometry*>* GeoRes = new std::vector<geos::geom::Geometry*>(); //Contiendra les bâtiments de CityGML découpés par le Shape
		std::vector<BatimentShape> InfoBatimentsRes;

        for(size_t i = 0; i < EnveloppeCity->getNumGeometries(); ++i)//On parcourt tous les polygons du CityGML
		{
			if(Link.second[i].size() < 1) // Si == 1, il n'y aurait rien à faire normalement car le bâtiment CityGML correspond à un seul bâtiment cadastral.
				continue;

			const geos::geom::Polygon * CurrPolyE = dynamic_cast<const geos::geom::Polygon*>(EnveloppeCity->getGeometryN(i));

			//Le but de ces lignes est de convertir le polygon avec son exterior ring et ses trous en un ensemble de geometry contenant ceux ci sans qu'ils soient encore liés. On peut ainsi parcourir seulement les arrêtes du polygon sans la notion d'intérieur
			geos::geom::Geometry * CurrGeoE = ConvertToSimpleGeom(CurrPolyE);

			std::vector<geos::geom::Geometry *> Shape1;			//Polygon de base
			std::vector<geos::geom::Geometry *> NewShape;		//Intersection avec le cityGML

			std::vector<geos::geom::Geometry *> GeoS;//Contiendra tous les polygons du shape liés au polygon du CityGML
            for(size_t j = 0; j < Link.second[i].size(); ++j)//Remplissage de GeoS
			{
				geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(Shape->getGeometryN(Link.second[i][j])->clone()); //Polygon du shape courant
				GeoS.push_back(CurrPolyS);
			}

			geos::geom::Geometry* UnionPolyS = geos::operation::geounion::CascadedPolygonUnion::Union(factory->createMultiPolygon(GeoS)); //Enveloppe du shape

			std::vector<geos::geom::Geometry *> PolyToGeo;
            for(size_t k = 0; k < UnionPolyS->getNumGeometries(); ++k)//On parcourt tous les polygons de UnionPolyS pour faire un ensemble de geometry contenant de manière indifférente tous les external ring et les interiors rings.
				//On pourra ainsi calculer l'intersection d'un point avec tous les rings de cette union sans être gêné par le fait qu'un polygon soit "plein".
					//Sinon, un point à l'intérieur du polygon est considéré comme intersect même s'il ne se trouve pas sur les bords.
			{
				geos::geom::Polygon * CurrPolyS = dynamic_cast<geos::geom::Polygon*>(UnionPolyS->getGeometryN(k)->clone());
				if(CurrPolyS == nullptr)
					continue;
				PolyToGeo.push_back(CurrPolyS->getExteriorRing()->clone());
                for(size_t j = 0; j < CurrPolyS->getNumInteriorRing(); ++j)
				{
					PolyToGeo.push_back(CurrPolyS->getInteriorRingN(j)->clone());
				}
			}				
			geos::geom::Geometry * CurrGeoS = factory->createGeometryCollection(PolyToGeo);

            for(size_t j = 0; j < Link.second[i].size(); ++j)//On manipule les polygons EXTERIEURS du shape (ceux qui intersectent CurrGeoS, qui représente l'enveloppe) : on prend un polygon, on le dilate, on lui soustrait tous les autres puis
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

                for(size_t k = 0; k < Link.second[i].size(); ++k)
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
            //geos::geom::Geometry * ShapeRes = factory->createGeometryCollection(NewShape);

			////////////////////////////Eliminer les superpositions des polygons :

            for(size_t j = 0; j < NewShape.size(); ++j)//Pour chaque polygon obtenu après l'intersection ...
			{
				geos::geom::Geometry * Geo = NewShape[j];

                for(size_t k = j+1; k < NewShape.size(); ++k)//On le compare avec les autres pour extraire ceux qui s'intersectent
				{
					const geos::geom::Geometry * Geo2 = NewShape[k];
					geos::geom::Geometry * InterGeo = Geo2->intersection(Geo);

					if(InterGeo->isEmpty())//Si les deux polygons ne s'intersectent pas, on passe aux suivants
                    {
                        delete InterGeo;
						continue;
                    }

					std::vector <geos::geom::Geometry*> InterVec;//Contiendra les polygons (surfaces communes) formés par l'opérateur intersection entre les deux polygons courant

					std::vector <geos::geom::Geometry*> LineVec;//Contiendra les lignes (arrêtes communes) formées par l'opérateur intersection entre les deux polygons courant

                    for(size_t t = 0; t < InterGeo->getNumGeometries(); ++t)//On parcourt les polygons formant l'intersection entre les deux polygons j et k
					{
						const geos::geom::Geometry * GeoTemp = InterGeo->getGeometryN(t);
						
						if(GeoTemp->getGeometryType() == "Polygon" && GeoTemp->getArea() > 0.001) //Seuil pour éliminer les polygons "plats"
							InterVec.push_back(GeoTemp->clone());
						if(GeoTemp->getGeometryType() != "LineString")
							continue;

						//Seules les lignes passeront donc par là : le but est de prolonger la ligne pour couper en deux les polygons qu'elle touche en projetant le point d'intersection de l'autre côté du polygon,
						//sur l'enveloppe du CityGML
						geos::geom::CoordinateSequence * coords = GeoTemp->getCoordinates();
						geos::geom::Point * P1 = factory->createPoint(coords->getAt(0));
						geos::geom::Point * P2 = factory->createPoint(coords->getAt(1));

                        for(size_t p = 0; p < InterGeo->getNumGeometries(); ++p)//On regarde quels sont les polygons qui touchent les linestring en reparcourant la liste des geometry de l'intersection
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

                    //geos::geom::Geometry* GeoLines = factory->createGeometryCollection(LineVec);

                    //geos::geom::Geometry * InterGeo2 = factory->createGeometryCollection(InterVec);

                    std::vector<geos::geom::Geometry*>* VecGeo = new std::vector<geos::geom::Geometry*>(); //Contiendra les geometries à assimiler à Geo (qu'il faudra donc retirer à Geo2)
                    std::vector<geos::geom::Geometry*>* VecGeo2 = new std::vector<geos::geom::Geometry*>();//Contiendra les geometries à assimiler à Geo2 (qu'il faudra donc retirer à Geo)

					std::vector<geos::geom::Geometry*> SplitPoly = SplitPolygon(InterVec, LineVec);

                    for(size_t t = 0; t < SplitPoly.size(); ++t)
					{
						//Save3GeometryRGB("TEST3", SplitPoly[t], Shape->getGeometryN(Link.second[i][j]), Shape->getGeometryN(Link.second[i][k]));
						int n = GetNearestGeo(SplitPoly[t], Shape->getGeometryN(Link.second[i][j]), Shape->getGeometryN(Link.second[i][k]));
						if(n == 1)
                            VecGeo->push_back(SplitPoly[t]->buffer(0.01));//0.01
						else
                            VecGeo2->push_back(SplitPoly[t]->buffer(0.01));//0.01
					}

                    // clean SplitPoly
                    for(geos::geom::Geometry* geom : SplitPoly)
                    {
                        delete geom;
                    }

					NewShape[k] = Geo2->difference(factory->createGeometryCollection(VecGeo)->Union().release());
					NewShape[j] = Geo->difference(factory->createGeometryCollection(VecGeo2)->Union().release());

					/*Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_1_Poly1", NewShape[j], factory->createGeometryCollection(VecGeo2)->Union().release(), NewShape[j]);
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_2_Poly2", NewShape[k], factory->createGeometryCollection(VecGeo)->Union().release(), NewShape[k]);
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_3_Avant" , Geo2, Geo, Geo2);
					Save3GeometryRGB("Inter_" + std::to_string(j) + "_" + std::to_string(k) + "_4_Apres" , NewShape[k], NewShape[j], NewShape[k]);*/

					Geo = NewShape[j];

                    delete InterGeo;
				}
			}

			/*geos::geom::Geometry * ShapeRes2 = factory->createGeometryCollection(NewShape);
			Scale = 10;
			Save3GeometryRGB("Shape_" + std::to_string(i), EnveloppeCity, Shape, EnveloppeCity);
			Save3GeometryRGB("ShapeRes_" + std::to_string(i), EnveloppeCity, ShapeRes, EnveloppeCity);
			Save3GeometryRGB("ShapeRes2_" + std::to_string(i), EnveloppeCity, ShapeRes2, EnveloppeCity);*/

            for(size_t t = 0; t < NewShape.size(); ++t)
			{
                GeoRes->push_back(NewShape[t]);
				InfoBatimentsRes.push_back(InfoBatiments[Link.second[i][t]]);
			}
		}

        delete EnveloppeCity;

		geos::geom::GeometryCollection * Batiments = factory->createGeometryCollection(GeoRes);

		//Scale = 1;
		//SaveGeometry("Batiments", Batiments);

		std::cout << "Lancement de l'extrusion des donnees 3D\n";

        ExtruderBatiments(Batiments, InfoBatimentsRes);

		delete Batiments;

#ifdef _WIN32
        _CrtDumpMemoryLeaks();
#endif // _WIN32
	}

	/**
	* @brief Compare deux fichiers CityGML d'une même zone obtenus à deux dates différentes en donnant les modifications entre leurs bâtiments
	*/
	void Algo::CompareTiles()//Lorsqu'il y a deux tuiles dans VCity, cette fonction crée une image les regroupant pour pouvoir les comparer
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();
		TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		if(tiles.size() != 2)
        {
            std::cout << "Erreur : Il faut ouvrir deux fichiers CityGML de la même zone, en commençant par le plus ancien." << std::endl;
			return;
        }

		geos::geom::Geometry * EnveloppeCity[2];//On part du principe que le plus vieux est dans le 0 et le plus récent dans le 1
		geos::geom::Geometry * EnveloppeCityU[2];//Version avec un seul polygone par bâtiment
		EnveloppeCityU[0] = nullptr;
		EnveloppeCityU[1] = nullptr;

		for(int i = 0; i < 2; ++i)
		{
			citygml::CityModel* model = tiles[i]->getCityModel();
			
			int cpt = 0;

			std::vector<geos::geom::Geometry*> GeoVecAll;

			for(citygml::CityObject* obj : model->getCityObjectsRoots())
			{
				if(obj->getType() == citygml::COT_Building)
				{
					std::vector<geos::geom::Geometry*> GeoVec;
					for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall et Roof) du bâtiment
					{
						if(object->getType() == citygml::COT_RoofSurface)
						{
							for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
							{
								for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
								{
									geos::geom::CoordinateSequence * Coords = new geos::geom::CoordinateArraySequence;
					
									for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
									{
										Coords->add(geos::geom::Coordinate(Point.x - offset_.x, Point.y - offset_.y, Point.z));
									}
									Coords->add(Coords->getAt(0));
									if(Coords->size() > 3)
									{
										GeoVec.push_back(factory->createPolygon(factory->createLinearRing(Coords), nullptr));
										GeoVecAll.push_back(factory->createPolygon(factory->createLinearRing(Coords), nullptr));
									}
									else
										delete Coords;
								}
							}
						}
					}

					geos::geom::MultiPolygon* Building = factory->createMultiPolygon(GeoVec);

					/*PolySet roofPoints;

					double heightmax = 0, heightmin = -1;
					projectRoof(obj,roofPoints, &heightmax, &heightmin);

					geos::geom::MultiPolygon * GeosObj = ConvertToGeos(roofPoints);
					geos::geom::Geometry * Enveloppe = GetEnveloppe(GeosObj);
					delete GeosObj;*/

					geos::geom::Geometry * Enveloppe = GetEnveloppe(Building);
					if(EnveloppeCityU[i] == nullptr)
						EnveloppeCityU[i] = Enveloppe;
					else
					{
						geos::geom::Geometry * tmp = EnveloppeCityU[i];
						EnveloppeCityU[i] = EnveloppeCityU[i]->Union(Enveloppe);
						delete tmp;
					}
				}
				cpt++;
				std::cout << "Avancement tuile " << i+1 << " : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
			}
			std::cout << std::endl;

			//Création de EnveloppeCity : pour chaque bâtiment distinct, il contient la liste des polygones qui le composent
			std::vector<geos::geom::Geometry*> GeoVec1;//Contiendra un geometrycollection par bâtiment
            for(size_t g = 0; g < EnveloppeCityU[i]->getNumGeometries(); ++g)
			{
				std::vector<geos::geom::Geometry*> GeoVec2;//Contiendra liste des polygones pour bâtiment i
				const geos::geom::Geometry * CurrBati = EnveloppeCityU[i]->getGeometryN(g); //On parcourt chaque bâtiment
                for(size_t j = 0; j < GeoVecAll.size(); ++j)
				{
					if(GeoVecAll[j]->intersects(CurrBati))
					{
						GeoVec2.push_back(GeoVecAll[j]);
						GeoVecAll.erase(GeoVecAll.begin() + j);
						j--;
					}
				}
				GeoVec1.push_back(factory->createGeometryCollection(GeoVec2));
			}
			EnveloppeCity[i] = factory->createGeometryCollection(GeoVec1);
		}

		//Save3GeometryRGB("BatiCompare", EnveloppeCityU[0], EnveloppeCityU[1], EnveloppeCityU[0]);
		//Save3GeometryRGB("Bati1", EnveloppeCityU[0], factory->createEmptyGeometry(), EnveloppeCityU[0]);
		//Save3GeometryRGB("Bati2", factory->createEmptyGeometry(), EnveloppeCityU[1], factory->createEmptyGeometry());

		std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Compare = CompareGeos(EnveloppeCityU[0], EnveloppeCityU[1], EnveloppeCity[0], EnveloppeCity[1]);

		std::vector<geos::geom::Geometry *> BatiDetruits;
		std::vector<geos::geom::Geometry *> BatiCrees;
		std::vector<geos::geom::Geometry *> BatiModifies1;
		std::vector<geos::geom::Geometry *> BatiModifies2;
		std::vector<geos::geom::Geometry *> BatiInchanges;

        for(size_t i = 0; i < EnveloppeCityU[0]->getNumGeometries(); ++i)
		{
			if(Compare.first[i].size() == 0)
				BatiDetruits.push_back(EnveloppeCityU[0]->getGeometryN(i)->clone());
			else
			{
				if(Compare.first[i][0] == -1)
					BatiInchanges.push_back(EnveloppeCityU[1]->getGeometryN(Compare.first[i][1])->clone());
				else if(Compare.first[i][0] == -2)
				{
					BatiModifies1.push_back(EnveloppeCityU[0]->getGeometryN(i)->clone());
					BatiModifies2.push_back(EnveloppeCityU[1]->getGeometryN(Compare.first[i][1])->clone());
				}
			}
		}
        for(size_t i = 0; i < EnveloppeCityU[1]->getNumGeometries(); ++i)
		{
			if(Compare.second[i].size() == 0)
				BatiCrees.push_back(EnveloppeCityU[1]->getGeometryN(i)->clone());
		}
		//Scale = 10;
		//Save3GeometryRGB("BatiCrees", EnveloppeCityU[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiCrees));
		//Save3GeometryRGB("BatiDetruits", EnveloppeCityU[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiDetruits));
		//Save3GeometryRGB("BatiModifies", EnveloppeCityU[1], factory->createGeometryCollection(BatiModifies1), factory->createGeometryCollection(BatiModifies2));
		//Save3GeometryRGB("BatiInchanges", EnveloppeCityU[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiInchanges));

		SaveGeometrytoShape("Bati", EnveloppeCityU[1]);
		SaveGeometrytoShape("BatiCrees", factory->createGeometryCollection(BatiCrees));
		SaveGeometrytoShape("BatiDetruits", factory->createGeometryCollection(BatiDetruits));
		SaveGeometrytoShape("BatiModifies", factory->createGeometryCollection(BatiModifies2));
		SaveGeometrytoShape("BatiInchanges", factory->createGeometryCollection(BatiInchanges));

		//Save3GeometryRGB("BatiCrees", EnveloppeCityU[1]->difference(factory->createGeometryCollection(BatiCrees)), factory->createGeometryCollection(BatiCrees), factory->createEmptyGeometry());

		for(auto& it : BatiInchanges) delete it;
		for(auto& it : BatiModifies2) delete it;
		for(auto& it : BatiModifies1) delete it;
		for(auto& it : BatiCrees) delete it;
		for(auto& it : BatiDetruits) delete it;

		delete EnveloppeCityU[1];
		delete EnveloppeCityU[0];
		delete EnveloppeCity[1];
		delete EnveloppeCity[0];

#ifdef _WIN32
        _CrtDumpMemoryLeaks();
#endif // _WIN32
	}

	/**
	* @brief Génère un fichier CityGML en LOD1 à partir d'un fichier shape
	* @param Shape Contient les bâtiments cadastraux
	* @param Hauteurs Contient les Zmin et Zmax des bâtiments issus du fichier shape
	*/
	void Algo::generateLOD1(geos::geom::Geometry * Shape, std::vector<std::pair<double, double>> Hauteurs)//Hauteurs : Liste les hauteurs et Zmin des polygons de ShapeGeo
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

        // clear previous CityGML model
        delete m_model;
        m_model = nullptr;

		//m_model = ConvertShapeToLOD1(Shape, Hauteurs);
		//return; //Pour avoir le LOD1

		if(Shape == nullptr || Shape->isEmpty())
		{
			std::cout << "Shape NULL. \n";
			return;
		}
		//SaveGeometry("Shape", Shape);
		//geos::geom::Geometry * ShapeSimp = geos::simplify::TopologyPreservingSimplifier::simplify(Shape, 2).release();
		//SaveGeometry("Shape_Simplified", ShapeSimp);

		// gen lod1
		m_model = ConvertShapeToLOD1(Shape, Hauteurs);
		return;

		geos::geom::Geometry * ShapeRes = Shape->buffer(4)->buffer(-4);

		//SaveGeometry("Shape_Close", ShapeRes);

		std::vector<std::pair<double, double>> Hauteurs2;
		std::vector<geos::geom::Geometry *> GeosWithoutHoles;

        for(size_t i = 0; i < ShapeRes->getNumGeometries(); ++i)
		{
			const geos::geom::Geometry * CurrGeo = ShapeRes->getGeometryN(i);

			const geos::geom::Polygon *p;
			if(CurrGeo->getGeometryType() != "Polygon")
				continue;

			p = dynamic_cast<const geos::geom::Polygon*>(CurrGeo);

			double H = 0, Zmin = -9999;
			int cpt = 0;
            for(size_t j = 0; j < Shape->getNumGeometries(); ++j)
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

			GeosWithoutHoles.push_back(factory->createPolygon(factory->createLinearRing(p->getExteriorRing()->getCoordinates()), nullptr));
			//GeosWithoutHoles.push_back(CurrGeo->buffer(CurrGeo->getLength()/10)->buffer(-CurrGeo->getLength()/10));
		}
		geos::geom::Geometry * ShapeResWithoutHoles = factory->createGeometryCollection(GeosWithoutHoles);

		//SaveGeometry("Shape_Close_WithoutHoles", ShapeResWithoutHoles);

        //geos::geom::Geometry * ShapeResWithoutHolesSimp = geos::simplify::TopologyPreservingSimplifier::simplify(ShapeResWithoutHoles, 4).release();
		//SaveGeometry("Shape_Close_WithoutHoles_Simplified", ShapeResWithoutHolesSimp);

        // gen lod1
		m_model = ConvertShapeToLOD1(ShapeResWithoutHoles, Hauteurs2);
		//m_model = ConvertShapeToLOD1(ShapeResWithoutHolesSimp, Hauteurs2);

#ifdef _WIN32
        _CrtDumpMemoryLeaks();
#endif // _WIN32
	}

	/**
	* @brief Génère un fichier CityGML en LOD1 à partir d'un fichier CityGML LOD2
	* @param Shape Contient les bâtiments cadastraux
	* @param Hauteurs Contient les Zmin et Zmax des bâtiments issus du fichier shape
	*/

	////////////////////////////////////////////////////////////////////////////////
	/**
	* @brief Récupère m_model
	*/
	citygml::CityModel* Algo::getCitymodel()
	{
		return m_model;
	}
	////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////

