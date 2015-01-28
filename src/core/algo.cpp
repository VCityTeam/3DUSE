// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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

#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/Polygon.h"
#include "geos/geom/Coordinate.h"
#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/LinearRing.h"
#include "geos/geom/Point.h"
#include "geos/operation/union/CascadedPolygonUnion.h"
#include "geos/operation/buffer/BufferOp.h"
#include "geos/operation/distance/DistanceOp.h"

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
    {
    }
    ////////////////////////////////////////////////////////////////////////////////
    Algo::~Algo()
    {
    }
	////////////////////////////////////////////////////////////////////////////////
	/**
	* @brief Sauvegarde la geometry dans un fichier shape
	* @param name Nom du shape à enregistrer
	* @param G Geometry à enregistrer
	*/
    void Algo::SaveGeometrytoShape(std::string name, const OGRMultiPolygon* G)
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
        std::string F = Folder;
        name = name + ".shp";
        if(F != "")
            name = F + "/" + name;

        remove(name.c_str());
        DS = Driver->CreateDataSource(name.c_str(), NULL);

        OGRLayer * Layer = DS->CreateLayer("Layer1");

        for(int i = 0; i < G->getNumGeometries(); ++i)
        {
            if(G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon && G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon25D)
                continue;

            OGRPolygon * Polygon =  dynamic_cast<OGRPolygon*>(G->getGeometryRef(i)->clone());

            OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());
            Feature->SetGeometry(Polygon);
            Layer->CreateFeature(Feature);

            OGRFeature::DestroyFeature(Feature);
        }
        OGRDataSource::DestroyDataSource(DS);

        std::cout << "Fichier " << name << " cree." << std::endl;
    }
    //Pour être appelé n'importe où dans algo.cpp
    void SaveGeometrytoShape(std::string name, const OGRMultiPolygon* G)
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

        for(int i = 0; i < G->getNumGeometries(); ++i)
        {
            if(G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon && G->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon25D)
                continue;

            OGRPolygon * Polygon =  dynamic_cast<OGRPolygon*>(G->getGeometryRef(i)->clone());

            OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());
            Feature->SetGeometry(Polygon);
            Layer->CreateFeature(Feature);

            OGRFeature::DestroyFeature(Feature);
        }
        OGRDataSource::DestroyDataSource(DS);

        std::cout << "Fichier " << name << " cree." << std::endl;
    }
    void SaveGeometrytoShape(std::string name, const OGRGeometry* G)
    {
        OGRMultiPolygon * Temp = new OGRMultiPolygon;
        Temp->addGeometry(G);
        SaveGeometrytoShape(name, Temp);
        delete Temp;
    }

	/**
	* @brief Projette les toits du CityObject sélectionné sur le plan (xy)
	* @param obj CityObject sélectionné
	* @param roofProj un set de Polygon, le résultat de la projection
	* @param heightmax Enregistre le Zmax des murs du bâtiment
    * @param heightmin Enregistre le Zmin des murs du bâtiment
	*/
	void projectRoof(citygml::CityObject* obj, PolySet &roofProj, double * heightmax, double * heightmin)
    {
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
                        poly.push_back(std::make_pair(point.x, point.y)); //on récupere le point
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
	* @brief Projette les toits du CityObject sélectionné sur le plan (xy)
	* @param obj CityObject sélectionné
	* @param FootPrint un multiPolygon, le résultat de la projection
	* @param heightmax Enregistre le Zmax des murs du bâtiment
    * @param heightmin Enregistre le Zmin des murs du bâtiment
	*/
	void GetFootprint(citygml::CityObject* obj, OGRMultiPolygon * FootPrint, double * heightmax, double * heightmin)
    {
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
                        OgrRing->addPoint(Vertices.x, Vertices.y);
						
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
	/**
	* @brief Récupère l'enveloppe d'une geometry en faisant une succession d'unions sur tous les polygones
	* @param MP Ensemble de polygones sur lequel on va générer une enveloppe
	*/
	OGRMultiPolygon * GetEnveloppe(OGRMultiPolygon * MP)
	{
        //std::cout << "Mise en place de l'union des polygons" << std::endl;

        OGRGeometry* ResUnion = new OGRMultiPolygon;

        ResUnion = MP->UnionCascaded();

        //On travaille avec des OGRMultiPolygon pour avoir un format universel, il faut donc transformer la geometry en collection.
        if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon25D)//La geometry est en fait un ensemble de geometry : plusieurs bâitments
        {
            OGRMultiPolygon * GeoCollection = dynamic_cast<OGRMultiPolygon*>(ResUnion->clone());

            OGRMultiPolygon * MultiPolygonRes = new OGRMultiPolygon;

            for(int i = 0; i < GeoCollection->getNumGeometries(); ++i)
            {
                OGRGeometry * Geometry = GeoCollection->getGeometryRef(i);

                if(Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon || Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon25D)
                {
                    OGRPolygon * Poly = dynamic_cast<OGRPolygon*>(Geometry);
                    OGRPolygon * PolyRes = new OGRPolygon;

                    PolyRes->addRing(Poly->getExteriorRing());

                    for(int j = 0; j < Poly->getNumInteriorRings(); ++j)
                    {
                        const OGRLinearRing * IntRing = Poly->getInteriorRing(j);

                        if(IntRing->get_Area() > 0.01) //Pour enlever les arêtes parasites formant des interior ring "plats"
                            PolyRes->addRingDirectly(dynamic_cast<OGRLinearRing*>(IntRing->clone()));
                    }

                    MultiPolygonRes->addGeometryDirectly(PolyRes);
                }
            }

            return MultiPolygonRes;
        }
        else if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon25D)//La geometry est en fait un seul polygon : un seul bâtiment
        {
            OGRMultiPolygon * GeoCollection = new OGRMultiPolygon;
            GeoCollection->addGeometryDirectly(ResUnion);
            return GeoCollection;
        }

        return nullptr;
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
    void ExtruderBatiments(geos::geom::Geometry * Batiments, std::vector<BatimentShape> InfoBatiments, std::string Folder)
	{
		const geos::geom::GeometryFactory * factory = geos::geom::GeometryFactory::getDefaultInstance();

		//citygml::CityModel* model = new citygml::CityModel;
		citygml::Envelope Envelope; //Envelope de l'ensembles des bâtiments qui seront enregistrés dans le fichier CityGML de sortie.

		const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

		std::vector<geos::geom::Geometry*> VecGeoRes;

		// create citygml exporter to append data into
        citygml::ExporterCityGML exporter(Folder + "/BatimentsDecoupes.gml");
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
                                        Coords->add(geos::geom::Coordinate(Point.x, Point.y, Point.z));
									}
									Coords->add(Coords->getAt(0));

									geos::geom::Geometry * GeoCityGML = factory->createPolygon(factory->createLinearRing(Coords), nullptr);

									geos::geom::Geometry * Inter = GeoCityGML->intersection(Bati);

									for(size_t k = 0; k < Inter->getNumGeometries(); ++k)
									{
										const geos::geom::Geometry * Interpart = Inter->getGeometryN(k);
										if(!Interpart->isValid() || Interpart->isEmpty() || Interpart->getNumPoints() < 4 || Interpart->getArea() == 0.0)
											continue;

										geos::geom::Geometry * Ring = CalculeZ(Interpart, GeoCityGML);
										if(Ring != nullptr)
										{
											VecGeo->push_back(Ring);
											Hauteurs.push_back(heightmin);
										}
									}
									delete GeoCityGML;
									delete Inter;
								}
							}
						}
					}
				}
			}
			if(VecGeo->size() > 0)
			{
				VecGeoRes.push_back(factory->createGeometryCollection(VecGeo));

                citygml::CityObject* BuildingCO = new citygml::Building("Building_" + std::to_string(j) + ":" + InfoBatiments[j].ID);

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
                        RingRoof->addVertex(TVec3d(Coords->getAt(k).x, Coords->getAt(k).y, Coords->getAt(k).z));
                        RingGround->addVertex(TVec3d(Coords->getAt(k).x, Coords->getAt(k).y, Hauteurs[i]));

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

                        RingWall->addVertex(TVec3d(Coords->getAt(k).x, Coords->getAt(k).y, Coords->getAt(k).z));
                        RingWall->addVertex(TVec3d(Coords->getAt(k+1).x, Coords->getAt(k+1).y, Coords->getAt(k+1).z));
                        RingWall->addVertex(TVec3d(Coords->getAt(k+1).x, Coords->getAt(k+1).y, Hauteurs[i]));
                        RingWall->addVertex(TVec3d(Coords->getAt(k).x, Coords->getAt(k).y, Hauteurs[i]));

						PolyWall->addRing(RingWall);
						Wall->addPolygon(PolyWall);

						citygml::CityObject* WallCO = new citygml::WallSurface("Wall_" + std::to_string(i) + "_" + std::to_string(k));

						WallCO->addGeometry(Wall);
						//model->addCityObject(WallCO);
						BuildingCO->insertNode(WallCO);
					}
					PolyRoof->addRing(RingRoof);
					PolyRoof->getEnvelope();
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
				BuildingCO->computeEnvelope();
				Envelope.merge(BuildingCO->getEnvelope()); //On remplit l'envelope au fur et à mesure pour l'exporter à la fin dans le fichier CityGML.

				delete BuildingCO;

				//model->addCityObject(BuildingCO);
				//model->addCityObjectAsRoot(BuildingCO);
			}
		}
		if(VecGeoRes.size() > 0)
		{

            for(geos::geom::Geometry* geom: VecGeoRes)
            {
                delete geom;
            }
		}
		exporter.addEnvelope(Envelope);
        exporter.endExport();

        std::cout << std::endl << "Fichier CityGML cree." << std::endl;
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
	* @brief Calcule la distance de Hausdorff unidirectionnelle entre un nuage de points et un triangle
	* @param GeoPoints Correspond à la géométrie contenant le nuage de points et que l'on va projeter sur la seconde géométrie
    * @param Geo Correspond aux triangles sur lesquels seront projetés les points
	*/
    double Hausdorff(OGRMultiPolygon * GeoPoints, OGRMultiPolygon * Geo)
    {
		std::vector<OGRPoint *> Points;

        for(int i = 0; i < GeoPoints->getNumGeometries(); ++i)
        {
            OGRPolygon * Poly = (OGRPolygon *)GeoPoints->getGeometryRef(i);
            OGRLinearRing * Ring = Poly->getExteriorRing();
            for(int j = 0; j < Ring->getNumPoints(); ++j)
            {
                OGRPoint * Point = new OGRPoint;
                Ring->getPoint(j, Point);
                Points.push_back(Point);
            }
        }

        double D = 0;
        for(size_t i = 0; i < Points.size(); ++i)
        {
            double D_1 = 10000;
            OGRPoint * OgrP0 = Points.at(i);
            TVec3d P0(OgrP0->getX(),OgrP0->getY(), OgrP0->getZ());
            for(int j = 0; j < Geo->getNumGeometries(); ++j)
            {
                double D_2;
                OGRPolygon * PolyTriangle = (OGRPolygon *)Geo->getGeometryRef(j);
                OGRLinearRing * Triangle = PolyTriangle->getExteriorRing();
                if(Triangle->getNumPoints() > 4 || Triangle->get_Area() < 0.01)//Si la géometry courante n'est pas un triangle
                {
                    continue;
                }
                OGRPoint * OgrP1 = new OGRPoint; //Point du triangle
                OGRPoint * OgrP2 = new OGRPoint;
                OGRPoint * OgrP3 = new OGRPoint;
                Triangle->getPoint(0, OgrP1);
                Triangle->getPoint(1, OgrP2);
                Triangle->getPoint(2, OgrP3);

                TVec3d P1(OgrP1->getX(),OgrP1->getY(), OgrP1->getZ());
                TVec3d P2(OgrP2->getX(),OgrP2->getY(), OgrP2->getZ());
                TVec3d P3(OgrP3->getX(),OgrP3->getY(), OgrP3->getZ());

                delete OgrP1;
                delete OgrP2;
                delete OgrP3;

                TVec3d P1P0 = P0 - P1; //Vecteur P1P0
                TVec3d P2P0 = P0 - P2;
                TVec3d P3P0 = P0 - P3;

                double nP1P0 = P1P0.length(); //Norme du vecteur P1P0
                double nP2P0 = P2P0.length();
                double nP3P0 = P3P0.length();

                if(nP1P0 == 0 || nP2P0 == 0 || nP3P0 == 0) // Si le point P0 est confondu avec l'un des points du triangle
                {
                    D_1 = 0;
                    break;
                }

                TVec3d P1P2 = P2 - P1;//Vecteur P1P2
                TVec3d P1P3 = P3 - P1;
                TVec3d P2P3 = P3 - P2;

                TVec3d Np(P1P2.y * P1P3.z - P1P2.z * P1P3.y, P1P2.z * P1P3.x - P1P2.x * P1P3.z, P1P2.x * P1P3.y - P1P2.y * P1P3.x); //Normal du triangle
                double nNp = Np.length();

                double cosa = (P1P0.x * Np.x + P1P0.y * Np.y + P1P0.z * Np.z)/(nP1P0 * nNp); //Calcul du cosinus de langle a entre Np et P1P0

                double nP0P0_ = nP1P0 * cosa;
                TVec3d P0P0_(- nP0P0_ * Np.x / nNp, - nP0P0_ * Np.y / nNp, - nP0P0_ * Np.z / nNp); //Vecteur P0P0_, P0_ étant le projeté de P0 sur le plan du triangle

                TVec3d P0_(P0.x + P0P0_.x, P0.y + P0P0_.y, P0.z + P0P0_.z); // Position du projeté de P0 sur le plan du triangle

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
                    TVec3d P0_P1 = P1 - P0_;//Vecteur P0_P1
                    TVec3d P0_P2 = P2 - P0_;
                    TVec3d P0_P3 = P3 - P0_;
                    double nP0_P1 = P0_P1.length();//Norme du vecteur P0_P1
                    double nP0_P2 = P0_P2.length();

                    //Sur P1P2 :
                    TVec3d Temp(P0_P2.y * P0_P1.z - P0_P2.z * P0_P1.y, P0_P2.z * P0_P1.x - P0_P2.x * P0_P1.z, P0_P2.x * P0_P1.y - P0_P2.y * P0_P1.x);
                    TVec3d R(Temp.y * P1P2.z - Temp.z * P1P2.y, Temp.z * P1P2.x - Temp.x * P1P2.z, Temp.x * P1P2.y - Temp.y * P1P2.x); //Direction de P0_ -> P0__
                    double nR = R.length();

                    double cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z)/(nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

                    double nP0_P0__ = nP0_P1 * cosg;
                    TVec3d P0_P0__(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
                    TVec3d P0__(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

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
                    Temp = TVec3d(P0_P3.y * P0_P1.z - P0_P3.z * P0_P1.y, P0_P3.z * P0_P1.x - P0_P3.x * P0_P1.z, P0_P3.x * P0_P1.y - P0_P3.y * P0_P1.x);
                    R = TVec3d(Temp.y * P1P3.z - Temp.z * P1P3.y, Temp.z * P1P3.x - Temp.x * P1P3.z, Temp.x * P1P3.y - Temp.y * P1P3.x); //Direction de P0_ -> P0__
                    nR = R.length();

                    cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z)/(nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

                    nP0_P0__ = nP0_P1 * cosg;
                    P0_P0__ = TVec3d(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
                    P0__ = TVec3d(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

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
                    Temp = TVec3d(P0_P3.y * P0_P2.z - P0_P3.z * P0_P2.y, P0_P3.z * P0_P2.x - P0_P3.x * P0_P2.z, P0_P3.x * P0_P2.y - P0_P3.y * P0_P2.x);
                    R = TVec3d(Temp.y * P2P3.z - Temp.z * P2P3.y, Temp.z * P2P3.x - Temp.x * P2P3.z, Temp.x * P2P3.y - Temp.y * P2P3.x); //Direction de P0_ -> P0__
                    nR = R.length();

                    cosg = (P0_P2.x * R.x + P0_P2.y * R.y + P0_P2.z * R.z)/(nP0_P2 * nR); //Calcul du cosinus de langle g entre R et P0_P2

                    nP0_P0__ = nP0_P2 * cosg;
                    P0_P0__ = TVec3d(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
                    P0__ = TVec3d(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

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
    double DistanceHausdorff(OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2)
	{
		double D12 = 0;//Distance de Geo1 à Geo2
		double D21 = 0;//Distance de Geo2 à Geo1

		D12 = Hausdorff(Geo1, Geo2);
		D21 = Hausdorff(Geo2, Geo1);

		return std::max(D12, D21);
    }

	/**
    * @brief Compare deux ensembles de geometries en retournant les liens entre leurs polygones et l'information sur ces liens : si un polygone se retrouve de manière identique dans les deux ensembles de geometries, dans un seul ou s'il a été modifié
	* @param Geo1 Premier ensemble de geometries qui ont été unies : deux triangles voisins sont réunis en un rectangle par exemple
	* @param Geo2 Second ensemble de geometries qui ont été unies
	* @param Geo1P Premier ensemble de geometries non unies
	* @param Geo2P Second ensemble de geometries non unies
	*/
	std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > CompareBati(OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2, std::vector<OGRMultiPolygon* > Geo1P, std::vector<OGRMultiPolygon *> Geo2P)
    {
        std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Res; //Enregistre les liens entre les polygones. Pour un polygone donnée de Geo1, si il est en lien avec un de Geo2, l'indice sera précédé de -1 ou -2 pour inchangé/changé

        size_t NbGeo1 = Geo1->getNumGeometries(); //Nb de bâtiments de la date1
        size_t NbGeo2 = Geo2->getNumGeometries(); //Nb de bâtiments de la date2

        Res.first.resize(NbGeo1);
        Res.second.resize(NbGeo2);

        for(size_t i = 0; i < NbGeo1; ++i)
        {
            OGRPolygon * Bati1 = (OGRPolygon *)Geo1->getGeometryRef(i)->clone();
            //OGRLinearRing * Ring1 = Bati1->getExteriorRing();

            for(int j = 0; j < NbGeo2; ++j)
            {
                OGRPolygon * Bati2 = (OGRPolygon *)Geo2->getGeometryRef(j)->clone();

                double Area = 0;

                OGRwkbGeometryType Type = Bati1->Intersection(Bati2)->getGeometryType();

                if(Type == OGRwkbGeometryType::wkbPolygon || Type == OGRwkbGeometryType::wkbPolygon25D || Type == OGRwkbGeometryType::wkbMultiPolygon || Type == OGRwkbGeometryType::wkbMultiPolygon25D)
                {
                    OGRPolygon * tmp = (OGRPolygon *)(Bati1->Intersection(Bati2));
                    Area = tmp->get_Area();
                    delete tmp;
                }

				double val1 = (Bati1->get_Area() - Area)/Area;
                double val2 = (Bati2->get_Area() - Area)/Area;

                if(val1 < 0.01 && val2 < 0.01 && Bati1->get_Area() - Area < 5 && Bati2->get_Area() - Area < 5)//Les polygons sont identiques
                {
                    if(DistanceHausdorff(Geo1P.at(i), Geo2P.at(j)) < 5)//Si la différence de hauteur est inférieure à 5m, et si la distance de Hausdorff entre les deux bâtimetns est inférieure à 5m.
                    {
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
                if(val1 < 0.5 && val2 < 0.5)//Le bâtiment a été modifié car les emprises au sol restent suffisament proches
                {
                    Res.first[i].push_back(-2);
                    Res.second[j].push_back(-2);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
                    break;
                }
				/*if(val1 < 0.01 && Hausdorff(Geo1P.at(i), Geo2P.at(j)) < 1 || val2 < 0.01 && Hausdorff(Geo2P.at(j), Geo1P.at(i)))//Le bâtiment a été modifié car une des géométries se retrouvent dans l'autre
				{
					Res.first[i].push_back(-2);
                    Res.second[j].push_back(-2);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
                    break;
				}*/
                delete Bati2;
            }
            delete Bati1;

            std::cout << "Avancement de CompareGeos : " << i + 1 << " / " << NbGeo1 << "\r" << std::flush;
        }
        std::cout << "\n";

        return Res;
    }


	////////////////////////////////////////////////////////////////////////////////

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

        ExtruderBatiments(Batiments, InfoBatimentsRes, Folder);

		delete Batiments;
	}

	/**
	* @brief Convertit un LOD1 contenu dans un MultiPolygon en Cityobject CityGML.
	* @param name Nom du Cityobject CityGML créée
	* @param Geometry MultiPolygon contient les emprises au sol à convertir 
	* @param heightmin Donne la valeur de z minimale pour le LOD1
	* @param heightmax Donne la valeur de z maximale pour le LOD1
	*/
	citygml::CityObject* Algo::ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin)
    {
        citygml::CityObject* BuildingCO = new citygml::Building("LOD1_" + name);
        citygml::CityObject* WallCO = new citygml::WallSurface("LOD1_" + name + "_Wall");
        citygml::CityObject* RoofCO = new citygml::RoofSurface("LOD1_" + name + "_Roof");
        //citygml::CityObject* GroundCO = new citygml::GroundSurface("LOD1_" + name + "_Ground");

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
                double x1 = point->getX();
                double y1 = point->getY();
				delete point;

				RingRoof->addVertex(TVec3d(x1, y1, *heightmax));

                point = new OGRPoint;
				ExtRing->getPoint(j+1, point);
                double x2 = point->getX();
                double y2 = point->getY();
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

	/**
	* @brief Convertit un LOD0 contenu dans un MultiPolygon en geometry CityGML.
	* @param name Nom de la geometry CityGML créée
	* @param Geometry MultiPolygon contient les emprises au sol à convertir en objet CityGML
	* @param heightmin Permet de situer les LOD0 dans l'espace
	*/
	citygml::Geometry* Algo::ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin)
    {
        citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
        for(int i = 0; i < Geometry->getNumGeometries(); ++i)
		{
			citygml::Polygon * Poly = new citygml::Polygon("Polygon");
			citygml::LinearRing * Ring = new citygml::LinearRing("ExteriorRing", true);

            if(Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon && Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon25D)
				continue;

			OGRPolygon * Polygon =  dynamic_cast<OGRPolygon*>(Geometry->getGeometryRef(i)->clone());

			OGRLinearRing * ExtRing = Polygon->getExteriorRing();

			for(int j = 0; j < ExtRing->getNumPoints(); ++j)
			{
				OGRPoint * point = new OGRPoint;
				ExtRing->getPoint(j, point);
                Ring->addVertex(TVec3d(point->getX(), point->getY(), *heightmin));
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
                    IntRingCityGML->addVertex(TVec3d(point->getX(), point->getY(), *heightmin));
				}
				Poly->addRing(IntRingCityGML);
			}
			Geom->addPolygon(Poly);

			delete Polygon;
		}
		return Geom;
	}

	/**
	* @brief Compare deux fichiers CityGML d'une même zone obtenus à deux dates différentes en donnant les modifications entre leurs bâtiments
	*/
    void Algo::CompareTiles(citygml::CityModel* City1, citygml::CityModel* City2)//Version GDAL
    {
        std::vector<citygml::CityModel *> tiles;
        tiles.push_back(City1);
        tiles.push_back(City2);

        //On part du principe que le plus vieux est dans le 0 et le plus récent dans le 1
        std::vector<OGRMultiPolygon *> EnveloppeCity[2];//Version avec un ensemble de polygones pour chaque bâtiment (un bâtiment = un polygone distinct après union)
        OGRMultiPolygon * EnveloppeCityU[2];//Version avec des polygones unis pour représenter le bâtiment (ce n'est pas l'ensemble de polygones bruts)

        EnveloppeCityU[0] = new OGRMultiPolygon;//nullptr;
        EnveloppeCityU[1] = new OGRMultiPolygon;//nullptr;

        for(int i = 0; i < 2; ++i)
		{
            citygml::CityModel* model = tiles[i];
			
			int cpt = 0;

            std::vector<OGRPolygon *> ModelPolygons;//GeoVecAll;//Contient tous les polygones composant le model courant

			for(citygml::CityObject* obj : model->getCityObjectsRoots())
			{
				if(obj->getType() == citygml::COT_Building)
				{
                    OGRMultiPolygon* Building = new OGRMultiPolygon;//Version OGR du bâtiment qui va être remplie

					for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall et Roof) du bâtiment
					{
						if(object->getType() == citygml::COT_RoofSurface)
						{
							for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
							{
								for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
								{
                                    OGRPolygon * OgrPoly = new OGRPolygon;
                                    OGRLinearRing * OgrRing = new OGRLinearRing;
					
									for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
									{
                                        OgrRing->addPoint(Point.x, Point.y, Point.z);
                                    }

                                    OgrRing->closeRings();

                                    if(OgrRing->getNumPoints() > 3)
									{
                                        OgrPoly->addRingDirectly(OgrRing);
                                        if(OgrPoly->IsValid())
                                        {
                                            Building->addGeometryDirectly(OgrPoly);
                                            ModelPolygons.push_back(OgrPoly);
                                        }
									}
									else
                                        delete OgrRing;
								}
							}
						}
                    }

                   // SaveGeometrytoShape(std::to_string(i)+"_Building_", Building);
                    OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);
                    //SaveGeometrytoShape(std::to_string(i)+"_Enveloppe", Enveloppe);

					if(EnveloppeCityU[i] == nullptr)
						EnveloppeCityU[i] = Enveloppe;
					else
					{
                        OGRMultiPolygon * tmp = EnveloppeCityU[i];

                        EnveloppeCityU[i] = (OGRMultiPolygon *)tmp->Union(Enveloppe);
						delete tmp;
					}
				}
				cpt++;
                std::cout << "Avancement tuile " << i+1 << " : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
			}
            std::cout << std::endl;

            //Création de EnveloppeCity : pour chaque bâtiment distinct, il contient la liste des polygones qui le composent. Ces bâtiments ne correspondent pas à ceux du CityGML, mais
            //aux polygones distincts apparus avec les unions successives.

            for(size_t g = 0; g < EnveloppeCityU[i]->getNumGeometries(); ++g)
			{
                OGRMultiPolygon * Bati = new OGRMultiPolygon;//Contiendra liste des polygones pour bâtiment i
                OGRPolygon * CurrBati = (OGRPolygon *)EnveloppeCityU[i]->getGeometryRef(g); //On parcourt chaque bâtiment
                for(size_t j = 0; j < ModelPolygons.size(); ++j)//Pour le bâtiment courant, on va chercher dans ModelPolygons quels sont les polygones qui le composent.
				{
                    if(ModelPolygons[j]->Intersects(CurrBati))//Ce polygone appartient bien à CurrBati
                    {
                        Bati->addGeometryDirectly(ModelPolygons[j]);//Directly donne l'ownership à Bati au lieu de cloner le polygon
                        ModelPolygons.erase(ModelPolygons.begin() + j);//On le retire de ModelPolygons car il est maintenant associé à CurrBati
						j--;
					}
                }
                EnveloppeCity[i].push_back(Bati);
            }
        }

        std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Compare = CompareBati(EnveloppeCityU[0], EnveloppeCityU[1], EnveloppeCity[0], EnveloppeCity[1]);

        OGRMultiPolygon* BatiDetruits = new OGRMultiPolygon;
        OGRMultiPolygon* BatiCrees = new OGRMultiPolygon;
        OGRMultiPolygon* BatiModifies1= new OGRMultiPolygon;
        OGRMultiPolygon* BatiModifies2= new OGRMultiPolygon;
        OGRMultiPolygon* BatiInchanges= new OGRMultiPolygon;

        for(size_t i = 0; i < EnveloppeCityU[0]->getNumGeometries(); ++i)
		{
			if(Compare.first[i].size() == 0)
                BatiDetruits->addGeometry(EnveloppeCityU[0]->getGeometryRef(i));
			else
            {
				if(Compare.first[i][0] == -1)
                    BatiInchanges->addGeometry(EnveloppeCityU[1]->getGeometryRef(Compare.first[i][1]));
				else if(Compare.first[i][0] == -2)
				{
                    BatiModifies1->addGeometry(EnveloppeCityU[0]->getGeometryRef(i));
                    BatiModifies2->addGeometry(EnveloppeCityU[1]->getGeometryRef(Compare.first[i][1]));
				}
			}
		}
        for(size_t i = 0; i < EnveloppeCityU[1]->getNumGeometries(); ++i)
		{
			if(Compare.second[i].size() == 0)
                BatiCrees->addGeometry(EnveloppeCityU[1]->getGeometryRef(i));
		}
		//Scale = 10;
		//Save3GeometryRGB("BatiCrees", EnveloppeCityU[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiCrees));
		//Save3GeometryRGB("BatiDetruits", EnveloppeCityU[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiDetruits));
		//Save3GeometryRGB("BatiModifies", EnveloppeCityU[1], factory->createGeometryCollection(BatiModifies1), factory->createGeometryCollection(BatiModifies2));
		//Save3GeometryRGB("BatiInchanges", EnveloppeCityU[1], factory->createEmptyGeometry(), factory->createGeometryCollection(BatiInchanges));

        SaveGeometrytoShape("Bati", EnveloppeCityU[1]);
        SaveGeometrytoShape("BatiCrees", BatiCrees);
        SaveGeometrytoShape("BatiDetruits", BatiDetruits);
        SaveGeometrytoShape("BatiModifies", BatiModifies2);
        SaveGeometrytoShape("BatiInchanges", BatiInchanges);

		//Save3GeometryRGB("BatiCrees", EnveloppeCityU[1]->difference(factory->createGeometryCollection(BatiCrees)), factory->createGeometryCollection(BatiCrees), factory->createEmptyGeometry());

        delete BatiInchanges;
        delete BatiModifies2;
        delete BatiModifies1;
        delete BatiCrees;
        delete BatiDetruits;

        delete EnveloppeCityU[0];
        delete EnveloppeCityU[1];

        for(auto& it : EnveloppeCity[0]) delete it;
        for(auto& it : EnveloppeCity[1]) delete it;
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
	}
	////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////

