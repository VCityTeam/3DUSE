#include "LinkCityGMLShape.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>

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

#include "export/exportCityGML.hpp"
////////////////////////////////////////////////////////////////////////////////
typedef std::pair<double,double> Point;
typedef std::vector<Point> Polygon2D;
typedef std::set<Polygon2D> PolySet;
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Découpe les bâtiments de la scène qui sont issus d'un fichier CityGML à partie des bâtiments cadastraux contenus dans le fichier shape.
* @param Shape Contient les bâtiments cadastraux
* ]param InfoBatiments Contient les informations de ces batiments contenues dans le fichier shape.
*/
void DecoupeCityGML(std::string Folder, geos::geom::Geometry * Shape, std::vector<BatimentShape> InfoBatiments)//LOD0 sur toute la scène + Comparaison entre CityGML et Cadastre
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

