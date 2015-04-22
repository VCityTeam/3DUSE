#include "LinkCityGMLShape.hpp"
#include "lodsmanagement.hpp"
#include "ExportToShape.hpp"
#include <iomanip>
#include <math.h>

double Precision_Vect = 0.00001; //Précision pour les points des Wall qui ne se superposent pas parfaitement aux emprises au sol issue du Roff
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

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Convertit un OGRPolygon* en citygml::Polygon*
* @param OGRPoly OGRPolygon* à convertir.
* @param Name Nom du Polygon CityGML à retourner.
*/
citygml::Polygon * ConvertOGRPolytoGMLPoly(OGRPolygon* OGRPoly, std::string Name)
{
    citygml::Polygon * Poly = new citygml::Polygon(Name + "_Poly");
    citygml::LinearRing * Ring = new citygml::LinearRing(Name + "_Ring", true);

    const OGRLinearRing * ExtRing = OGRPoly->getExteriorRing();

    for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
    {
        OGRPoint * point = new OGRPoint;
        ExtRing->getPoint(j, point);
        double x = point->getX();
        double y = point->getY();
        double z = point->getZ();
        delete point;

        Ring->addVertex(TVec3d(x, y, z));
    }
    Poly->addRing(Ring);

    return Poly;
}

/**
* @brief Parcourt tous les bâtiments du CityModel, calcule leurs emprises au sol, les stock dans un Vector de Polygon en respectant leur ordre de stockage dans le CityModel. Un seul polygon par emprise au sol : Passage dans SplitBuildingsFromCityGML obligatoire !
* @param model contient les données du CityGML traité
*/
std::vector<OGRPolygon*> GetFootPrintsfromCityGML(citygml::CityModel* Model)
{
	std::vector<OGRPolygon *> ListFootPrints;

	int cpt = 0;
	for(citygml::CityObject* obj : Model->getCityObjectsRoots())
    {
        if(obj->getType() == citygml::COT_Building)
        {
            OGRGeometry* Building = new OGRPolygon;//Version OGR du bâtiment qui va être remplie

            for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
            {
                if(object->getType() == citygml::COT_RoofSurface)
                {
                    for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
                    {
                        for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                        {
                            OGRLinearRing * OgrRing = new OGRLinearRing;

                            for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								OgrRing->addPoint(Point.x, Point.y, Point.z);

                            OgrRing->closeRings();

                            if(OgrRing->getNumPoints() > 3)
                            {
								OGRPolygon * OgrPoly = new OGRPolygon;
                                OgrPoly->addRingDirectly(OgrRing);
                                if(OgrPoly->IsValid())
                                {
									OGRGeometry* tmp = Building;
                                    Building = tmp->Union(OgrPoly);
									delete tmp;
                                }
                                delete OgrPoly;
                            }
                            else
                                delete OgrRing;
                        }
                    }
                }
            }

			if(Building->IsEmpty() || (Building->getGeometryType() != wkbPolygon && Building->getGeometryType() != wkbPolygon25D))
			{
				++cpt;
				std::cout << "Erreur sur GetFootPrintsfromCityGML Batiment " << cpt << std::endl;
				//SaveGeometrytoShape("Building.shp", Building);
				//std::cout << "Avancement etape 1 : " << cpt << "/" << Model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
				ListFootPrints.push_back(nullptr);
				continue;
			}
			OGRPolygon* Envelope = (OGRPolygon*)Building;
			//On retire les InteriorRing avec une aire négligeable qui sont issues des imprécisions des données vectorielles
			if(Envelope->getNumInteriorRings() > 0)
			{
				OGRPolygon* ResBuilding = new OGRPolygon;
				ResBuilding->addRing((OGRLinearRing*)Envelope->getExteriorRing()->clone());
				for(int r = 0; r < Envelope->getNumInteriorRings(); ++r)
				{
					OGRLinearRing* Ring = Envelope->getInteriorRing(r);
					OGRPolygon* tmp = new OGRPolygon;
					tmp->addRing(Ring);
					if(tmp->get_Area() > 10 * Precision_Vect)
						ResBuilding->addRing(Ring);
					delete tmp;
				}
				delete Envelope;
				ListFootPrints.push_back(ResBuilding);
			}
			else
				ListFootPrints.push_back(Envelope);
        }

        ++cpt;
        //std::cout << "Avancement etape 1 : " << cpt << "/" << Model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
    }
	//std::cout << std::endl;

	return ListFootPrints;
}

/**
* @brief Parcourt tous les bâtiments du Layer, récupère leurs emprises au sol, les stock dans un Vector de Polygon en respectant leur ordre de stockage dans le Layer. Un seul polygon par emprise au sol.
* @param Layer contient les données du Shapefile traité
*/
std::vector<OGRPolygon*> GetFootPrintsfromShapeFile(OGRLayer* Layer)
{
	std::vector<OGRPolygon *> ListFootPrints;
	OGRFeature *Feature;
	int cpt = 0;
	while((Feature = Layer->GetNextFeature()) != NULL)
	{
		OGRGeometry* Building = Feature->GetGeometryRef();
		if(Building->getGeometryType() == wkbPolygon || Building->getGeometryType() == wkbPolygon25D)
			ListFootPrints.push_back((OGRPolygon *)Building);
		else
		{
			std::cout << "Erreur sur GetFootPrintsfromShapeFile Batiment " << cpt << std::endl;
			ListFootPrints.push_back(nullptr);
        }
		++cpt;
	}
	return ListFootPrints;
}

/**
* @brief Projette le point désiré sur l'arête du polygone la plus proche et ne contenant pas ce point, retourne le point projeté
* @param Point que l'on veut projeter
* @param Envelope Polygone sur lequel on souhaite projeter le point
*/
OGRPoint* ProjectPointOnEnvelope(OGRPoint* Point, OGRPolygon* Envelope, OGRLineString* CurrLine, OGRLineString* PrevLine, bool* PointIsModified)
{
	OGRLinearRing* ExtRing = Envelope->getExteriorRing();

	OGRPoint* projete = new OGRPoint;

	//On va parcrourir toutes les arêtes de Envelope, on va projeter Point dessus et on va stocker un certains nombre d'informations sur les projections qui semblent intéressantes.
	std::vector<double> Distances; //Contiendra la liste des distances
	std::vector<double> Angles; //Contiendra la liste des Angles
	std::vector<OGRPoint*> Projetes; //Contient la liste des projetés.

	for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j) //Il ne faut pas reparcourir le premier point qui est bouclé à la fin
	{
		OGRPoint * PointCurr = new OGRPoint;
		ExtRing->getPoint(j, PointCurr);
		OGRPoint * PointNext = new OGRPoint;
		ExtRing->getPoint(j + 1, PointNext);
		OGRLineString* LineBC = new OGRLineString;
		LineBC->addPoint(PointCurr);
		LineBC->addPoint(PointNext);

        if(LineBC->Intersects(Point)) //Si l'arête contient Point, inutile de calculer son projeté sur cette même arête.
        {
            delete PointCurr;
            delete PointNext;
            delete LineBC;
			continue;
        }
		//On a trouvé un segment sur lequel sera projeté le point (il ne contient pas Point), il faut maintenant calculer cette projection : H est la projection de A sur [BC] de vecteur directeur v

		double Ax = Point->getX();
		double Ay = Point->getY();
		double Bx = PointCurr->getX();
		double By = PointCurr->getY();
		double Cx = PointNext->getX();
		double Cy = PointNext->getY();
		double vx = Cx-Bx;
		double vy = Cy-By;
		double Normev = sqrt(vx*vx + vy*vy);
		double BH = ((Ax - Bx) * vx + (Ay - By) * vy) / Normev; // <=> BA.v (produit scalaire)
		double Hx;
		double Hy;
		if(BH < 0)
		{
			Hx = Bx;
			Hy = By;
		}
		else if(BH > Normev)
		{
			Hx = Cx;
			Hy = Cy;
		}
		else
		{
			Hx = Bx + BH * vx / Normev;
			Hy = By + BH * vy / Normev;
		}
		double AH = sqrt((Hx-Ax)*(Hx-Ax) + (Hy-Ay)*(Hy-Ay));
		double AHx = Hx - Ax;
		double AHy = Hy - Ay;

		OGRPoint * Proj = new OGRPoint; // Projeté de Point
		Proj->setX(Hx);
		Proj->setY(Hy);

		double PrevLineX = PrevLine->getX(0) - PrevLine->getX(1);
		double PrevLineY = PrevLine->getY(0) - PrevLine->getY(1);
		double AnglePrev = acos((PrevLineX * AHx + PrevLineY * AHy)/(AH*PrevLine->get_Length())); //Mesure de l'angle entre AH et PrevLine
		//AnglePrev = AnglePrev * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

		double CurrLineX = CurrLine->getX(1) - CurrLine->getX(0);
		double CurrLineY = CurrLine->getY(1) - CurrLine->getY(0);
		double AngleCurr = acos((CurrLineX * AHx + CurrLineY * AHy)/(AH*CurrLine->get_Length())); //Mesure de l'angle entre AH et CurrLine
		//AngleCurr = AngleCurr * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

		Distances.push_back(AH);
		double tmp = std::min(AnglePrev / (AnglePrev + AngleCurr), AngleCurr / (AnglePrev + AngleCurr));
		Angles.push_back(tmp); //Permet de repérer angulairement la direction du projeté par rapport aux Lines contenant le point. Si cette valeur vaut 1/2, cela signifie que le projeté forme la bissectrice de l'angle 
							   //formé par PrevLine et CurrLine => Visuellement c'est bien 

		Projetes.push_back(Proj);

        delete PointCurr;
        delete PointNext;
        delete LineBC;
	}
	for(int i = 0; i < Envelope->getNumInteriorRings(); ++i)
	{
		OGRLinearRing* IntRing = Envelope->getInteriorRing(i);
		for(int j = 0; j < IntRing->getNumPoints() - 1; ++j) //Il ne faut pas reparcourir le premier point qui est bouclé à la fin
		{
			OGRPoint * PointCurr = new OGRPoint;
			IntRing->getPoint(j, PointCurr);
			OGRPoint * PointNext = new OGRPoint;
			IntRing->getPoint(j + 1, PointNext);
			OGRLineString* LineBC = new OGRLineString;
			LineBC->addPoint(PointCurr);
			LineBC->addPoint(PointNext);

            if(LineBC->Intersects(Point)) //Si l'arête contient Point, inutile de calculer son projeté sur cette même arête.
            {
                delete PointCurr;
                delete PointNext;
                delete LineBC;
                continue;
            }
			//On a trouvé un segment sur lequel sera projeté le point (il ne contient pas Point), il faut maintenant calculer cette projection : H est la projection de A sur [BC] de vecteur directeur v

			double Ax = Point->getX();
			double Ay = Point->getY();
			double Bx = PointCurr->getX();
			double By = PointCurr->getY();
			double Cx = PointNext->getX();
            double Cy = PointNext->getY();
			double vx = Cx-Bx;
			double vy = Cy-By;
			double Normev = sqrt(vx*vx + vy*vy);
			double BH = ((Ax - Bx) * vx + (Ay - By) * vy) / Normev; // <=> BA.v (produit scalaire)
			double Hx;
			double Hy;
			if(BH < 0)
			{
				Hx = Bx;
				Hy = By;
			}
			else if(BH > Normev)
			{
				Hx = Cx;
				Hy = Cy;
			}
			else
			{
				Hx = Bx + BH * vx / Normev;
				Hy = By + BH * vy / Normev;
			}
			double AH = sqrt((Hx-Ax)*(Hx-Ax) + (Hy-Ay)*(Hy-Ay));
			double AHx = Hx - Ax;
			double AHy = Hy - Ay;

			OGRPoint * Proj = new OGRPoint; // Projeté de Point
			Proj->setX(Hx);
			Proj->setY(Hy);

			double PrevLineX = PrevLine->getX(0) - PrevLine->getX(1);
			double PrevLineY = PrevLine->getY(0) - PrevLine->getY(1);
			double AnglePrev = acos((PrevLineX * AHx + PrevLineY * AHy)/(AH*PrevLine->get_Length())); //Mesure de l'angle entre AH et PrevLine
			//AnglePrev = AnglePrev * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

			double CurrLineX = CurrLine->getX(1) - CurrLine->getX(0);
			double CurrLineY = CurrLine->getY(1) - CurrLine->getY(0);
			double AngleCurr = acos((CurrLineX * AHx + CurrLineY * AHy)/(AH*CurrLine->get_Length())); //Mesure de l'angle entre AH et CurrLine
			//AngleCurr = AngleCurr * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

			Distances.push_back(AH);
			double tmp = std::min(AnglePrev / (AnglePrev + AngleCurr), AngleCurr / (AnglePrev + AngleCurr));
			Angles.push_back(tmp); //Permet de repérer angulairement la direction du projeté par rapport aux Lines contenant le point. Si cette valeur vaut 1/2, cela signifie que le projeté forme la bissectrice de l'angle 
								   //formé par PrevLine et CurrLine => Visuellement c'est bien 

			Projetes.push_back(Proj);
            delete PointCurr;
            delete PointNext;
            delete LineBC;
		}
	}

	double distance = -1;
	double Angle;
	for(int i = 0; i < Distances.size(); ++i)//Parmi toutes les projections possibles, il va falloir choisir celle que l'on retient : pour l'instant c'est celui dont le projeté est le plus proche, si possible ne contenant pas d'arête entre les deux
	{
		if(distance == -1 || distance > Distances.at(i))
		{
            delete projete;
            projete = (OGRPoint*)Projetes.at(i)->clone();
			distance = Distances.at(i);
			Angle = Angles.at(i);
		}
	}

	distance = 2 * distance; //On double cette valeur pour le test sur les angles : si on trouve un meilleur angle, on accepte une distance deux fois supérieure
	if(Angle < 0.25) //Cela signifie que la ligne formée par Point et son projeté est beaucoup plus proche d'une des Line que de l'autre, on veut regarder si on ne peut pas trouver mieux (idéal = bissectrice ?) sans trop perdre en terme de distance
	{
		for(int i = 0; i < Distances.size(); ++i)// Si l'angle est trop faible, on va chercher un éventuel autre projeté avec une distance convenable et un meilleur angle
		{
			if(Distances.at(i) < distance && Angles.at(i) > 0.25) //On juge la distance convenable si elle est inférieure au double de la distance min, et si on en trouve un après on fait récupère une valeur de distance non doublée
			{
				//SaveGeometrytoShape("Point.shp", Point);
				//SaveGeometrytoShape("Polygon.shp", Envelope);
				//SaveGeometrytoShape("Projete1.shp", projete);
				//SaveGeometrytoShape("Projete2.shp", Projetes.at(i));
				//std::cout << Angle << " - " << Angles.at(i) << std::endl;
				//int a;
				//std::cin >>a;
                delete projete;
                projete = (OGRPoint*)Projetes.at(i)->clone();
				distance = Distances.at(i);
			}
		}
	}

    for(OGRPoint* PointTemp : Projetes)
        delete PointTemp;

	//Il faut vérifier que ces deux points ne forment pas une arête déjà existante, car dans ce cas là il ne sert à rien de la recréer. Point appartient à deux arêtes : CurrLine et la PrevLine précédente, il suffit donc de vérifier ces deux là.
    bool LineFound = projete->Intersects(CurrLine) || projete->Intersects(PrevLine);
	//Il faut vérifier que la ligne formée par ces deux points soit bien contenue dans le polygone. On va tester le milieu formé par ce segment.	
	OGRPoint* Milieu = new OGRPoint((Point->getX() + projete->getX())/2, (Point->getY() + projete->getY())/2);
    bool Outside = !Milieu->Intersects(Envelope);
	delete Milieu;

	if(LineFound || Outside) //On veut prolonger pour voir si on peut fixer le projeté plus loin tout en restant dans le PolygonGMLDiffShape, sinon on ne fait rien car cela signifie qu'il n'y a vraisemblablement pas besoin de couper le polygone en deux
	{
		TVec2f Vecteur;
		Vecteur.x = projete->getX() - Point->getX();
		Vecteur.y = projete->getY() - Point->getY();

		OGRLineString* ProlongementProjete = new OGRLineString;
		OGRPoint* Projete2 = new OGRPoint(projete->getX() + Vecteur.x, projete->getY() + Vecteur.y);
		ProlongementProjete->addPoint(projete);//Crée un point basé sur la droite Point-Projeté, mais un peu plus que loin le Projeté. La question est maintenant de savoir si il est en dehors de PolygonGMLDiffShape ou non.
		ProlongementProjete->addPoint(Projete2);
		//Il ne faut pas oublier qu'à ce moment, Projete n'est pas forcément sur PolygonGMLDiffShape à cause des imprécisions. Du coup il faut faire attention avec les tests d'intersection qui peuvent être faussés
        if(!ProlongementProjete->Intersects(Envelope)) //Cela signifie que Projete2 n'est pas dans le polygon (et que l'intersection de Projete avec le polygon ne fonctionne pas, car Projete est très légèrement en dehors) : îl n'y a pas de projete valide
        {
            delete ProlongementProjete;
            delete Projete2;
            delete projete;
            return new OGRPoint;
        }
		OGRGeometry* Inter = ProlongementProjete->Intersection(Envelope);
		if(Inter->getGeometryType() == wkbPoint || Inter->getGeometryType() == wkbPoint25D) //Cela signifie que Projete2 n'est pas dans le polygon (et que l'intersection de Projete avec le polygon a fonctionné)
        {
            delete ProlongementProjete;
            delete Projete2;
            delete projete;
            delete Inter;
            return new OGRPoint;
        }
		OGRLineString* InterLS = (OGRLineString*) Inter; //Arrivé ici, l'intersection est forcément une LineString
		double Length = InterLS->get_Length();
		if(Length < 10 * Precision_Vect) //Cela signifie que Projete2 n'est pas dans le polygon : l'intersection de Projete avec le polygon n'a pas fonctionné correctement car il est très légèrement à l'intérieur
        {                                //et l'intersection donne donc une ligne très courte correspondant à la différence de précision entre Projete et la ligne du Polygon sur laquelle il censé être
            delete ProlongementProjete;
            delete Projete2;
            delete projete;
            delete InterLS;
            return new OGRPoint;
        }
		//On considère qu'il faut prolonger la ligne jusqu'à la prochaine arête afin de couper le polygone en deux

		double Coeff = 1;

		bool Test1, Test2;
        Test1 = Projete2->Intersects(Envelope); //Projete2 est dans le Polygon
		Test2 = (InterLS->getGeometryType() == wkbLineString || InterLS->getGeometryType() == wkbLineString25D); //Projete2 n'a pas traversé d'interioring Ring car l'intersection serait alors un MutliLineString puisqu'elle serait coupée en deux
		while(Test1 && Test2)//Tant que Projete2 est inclus dans le Polygon et que nous n'avons pas traversé d'InteriorRing
		{
			++Coeff;
			delete InterLS;
			delete ProlongementProjete;
			delete Projete2;

			Projete2 = new OGRPoint(projete->getX() + Coeff * Vecteur.x, projete->getY() + Coeff * Vecteur.y);
			ProlongementProjete = new OGRLineString;
			ProlongementProjete->addPoint(projete);
			ProlongementProjete->addPoint(Projete2);
			InterLS = dynamic_cast<OGRLineString*>(ProlongementProjete->Intersection(Envelope));
			if(InterLS == nullptr)
			{
				Test2 = false;
			}
			else
			{
                Test1 = Projete2->Intersects(Envelope);
			}
		}//Si on sort avec !Test1 et Test2 -> On a trouvé le bon ProlongementProjete. Si on sort avec !Test2 -> On a traversé un Interior Ring donc il faut revenir en arrière sur le coeff pour que Projete2 tombe dedans.
		//Si on sort avec !Test1 mais !Test2 -> On a traveré un InteriorRing, puis la portion de Polygon située derrière pour finalement ressortir (dans un autre InterioRing ou en dehors du Polygon). Pas bon non plus car il faut revenir avant l'intRing
		double PrevCoeff = Coeff - 1; //Définit la valeur de Coeff précédente pour définir l'intervalle de recherche
		int compteur = 0;
		//std::cout << "NEW COEFF" << std::endl;
		while(Test1 || !Test2)//Tant que Projete2 est inclus dans le Polygon
		{
			double TempCoeff = Coeff;
			Coeff = (PrevCoeff + Coeff) / 2; //Dichotomie pour trouver le bon Coeff
			//std::cout << "PrefCoeff = " << PrevCoeff << " TempCoeff = " << TempCoeff << " Coeff = " << Coeff << std::endl;
			delete InterLS;
			delete ProlongementProjete;
			delete Projete2;

			Projete2 = new OGRPoint(projete->getX() + Coeff * Vecteur.x, projete->getY() + Coeff * Vecteur.y);
			ProlongementProjete = new OGRLineString;
			ProlongementProjete->addPoint(projete);
			ProlongementProjete->addPoint(Projete2);
			InterLS = dynamic_cast<OGRLineString*>(ProlongementProjete->Intersection(Envelope));
			++ compteur;
			if(compteur > 20)
			{
				//std::cout << "Boucle Infinie." << std::endl;
				/*SaveGeometrytoShape("Polygon.shp", Envelope);
				SaveGeometrytoShape("Point.shp", Point);
				SaveGeometrytoShape("Projete1.shp", projete);
				SaveGeometrytoShape("Prolongement.shp", ProlongementProjete);
				std::cout << "PrefCoeff = " << PrevCoeff << " TempCoeff = " << TempCoeff << " Coeff = " << Coeff << std::endl;
				int a;
				std::cin >> a;*/
				delete InterLS;
				delete ProlongementProjete;
				delete Projete2;
				return new OGRPoint;
			}
			if(InterLS == nullptr)
				Test2 = false;
			else
				Test2 = true;

			if(!Test2)//On doit diminuer le prochain Coeff
			{
				PrevCoeff = std::min(PrevCoeff, TempCoeff);
			}
			else//On doit augmenter le prochain Coeff
			{
				PrevCoeff = std::max(PrevCoeff, TempCoeff);
			}
		}
		delete InterLS;
		delete Projete2;
		///////////////On a suffisament allongé la ligne formée par le point initiale et le projeté pour couper en deux la portion de polygon désirée, il faut maintenant calculer l'intersection entre cette ligne et le polygone pour trouver le projeté à retourner
		
		OGRPoint* OldProjete = (OGRPoint*)projete->clone();
		delete projete;
		projete = new OGRPoint;
		distance = Precision_Vect; //On ne fixe pas à 0, pour que si Line intersecte ProlongementProjete en Point, il ne soit pas retenu comme projeté valide

		for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j) //On va reparcourir toutes les lignes du polygone, regarder celles qui intersectent InterLS, et prendre leurs points d'intersection. On conservera celui qui est le plus éloigne de Point
		{
			OGRPoint * PointCurr = new OGRPoint;
			ExtRing->getPoint(j, PointCurr);
			OGRPoint * PointNext = new OGRPoint;
			ExtRing->getPoint(j + 1, PointNext);
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(PointCurr);
			Line->addPoint(PointNext);
			delete PointCurr;
			delete PointNext;

            if(!Line->Intersects(ProlongementProjete))
            {
                delete Line;
				continue;
            }

			OGRGeometry* PointInter = Line->Intersection(ProlongementProjete);
            delete Line;
			if(PointInter->getGeometryType() != wkbPoint && PointInter->getGeometryType() != wkbPoint25D)
            {
                delete PointInter;
				continue;
            }
			double ecartement = PointInter->Distance(Point);
			if(ecartement > distance)
			{
				delete projete;
				projete = (OGRPoint*)PointInter;
				distance = ecartement;
			}
            else
                delete PointInter;
		}
		for(int k = 0; k < Envelope->getNumInteriorRings(); ++k)
		{
			OGRLinearRing* IntRing = Envelope->getInteriorRing(k);
			for(int j = 0; j < IntRing->getNumPoints() - 1; ++j) //On va reparcourir toutes les lignes du polygone, regarder celles qui intersectent InterLS, et prendre leurs points d'intersection. On conservera celui qui est le plus éloigne de Point
			{
				OGRPoint * PointCurr = new OGRPoint;
				IntRing->getPoint(j, PointCurr);
				OGRPoint * PointNext = new OGRPoint;
				IntRing->getPoint(j + 1, PointNext);
				OGRLineString* Line = new OGRLineString;
				Line->addPoint(PointCurr);
				Line->addPoint(PointNext);
				delete PointCurr;
				delete PointNext;

                if(!Line->Intersects(ProlongementProjete))
                {
                    delete Line;
                    continue;
                }

                OGRGeometry* PointInter = Line->Intersection(ProlongementProjete);
                delete Line;
                if(PointInter->getGeometryType() != wkbPoint && PointInter->getGeometryType() != wkbPoint25D)
                {
                    delete PointInter;
                    continue;
                }
                double ecartement = PointInter->Distance(Point);
                if(ecartement > distance)
                {
                    delete projete;
                    projete = (OGRPoint*)PointInter;
                    distance = ecartement;
                }
                else
                    delete PointInter;
			}
		}
		/*SaveGeometrytoShape("ProlongementProjete.shp", ProlongementProjete);
		SaveGeometrytoShape("Polygon.shp", Envelope);
		SaveGeometrytoShape("Point.shp", Point);
		SaveGeometrytoShape("Projete.shp", projete);
		SaveGeometrytoShape("ProjeteOld.shp", OldProjete);
		int a;
		std::cin >> a;*/

		delete ProlongementProjete;
		*PointIsModified = true;
		delete Point;
		Point = (OGRPoint*)OldProjete->clone();
		delete OldProjete;
	}
	return projete;
}

/**
* @brief Parcourt les points du polygones de Points, regarde s'ils semblent appartenir à une arête de Lines et si c'est le cas, on coupe cette arête en deux afin que ce point existe et que l'intersection fonctionne.
* On retourne une les même polygone que Lines, mais avec des arêtes parfois partagées en deux. Globalement ça ne change rien, mais on pourra calculer l'intersection entre les points de Points et celles ci.
* @param Points : Polygone auquel on va s'intéresser pour ses points
* @param Lines : Polygone auquel on va s'intéresser pour ses arêtes
*/
OGRGeometry* CreatePointsOnLine(OGRGeometry* Points, OGRGeometry* Lines)
{
    if(Lines->getGeometryType() != wkbPolygon && Lines->getGeometryType() != wkbPolygon25D)
        return nullptr;
    if(Points->getGeometryType() != wkbPolygon && Points->getGeometryType() != wkbPolygon25D)
        return nullptr;

	OGRPolygon * PolyP = (OGRPolygon*) Points;
    OGRLinearRing * ExtRingP = PolyP->getExteriorRing();

	std::vector<OGRPoint*> ListPoints; //On stocke la liste des points du polygone Points, le reste ne nous intéresse pas.

	for(int j = 0; j < ExtRingP->getNumPoints() - 1; ++j)
	{
		ListPoints.push_back(new OGRPoint(ExtRingP->getX(j), ExtRingP->getY(j), ExtRingP->getZ(j)));
	}
	for(int i = 0; i < PolyP->getNumInteriorRings(); ++i)
	{
		OGRLinearRing* IntRingP = PolyP->getInteriorRing(i);
		for(int j = 0; j < IntRingP->getNumPoints() - 1; ++j)
		{
			ListPoints.push_back(new OGRPoint(IntRingP->getX(j), IntRingP->getY(j), IntRingP->getZ(j)));
		}
	}

    OGRPolygon * PolyL = (OGRPolygon*) Lines;
    OGRLinearRing * RingL = PolyL->getExteriorRing();

    OGRPolygon * ResPoly = new OGRPolygon;
    OGRLinearRing * ResRing = new OGRLinearRing;

	//// !! On va ajouter les points intermédiaires dans les arêtes concernées, mais on va conserver les coordonnées en Z de Lines. !! ////

    for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
    {
        OGRPoint* P1 = new OGRPoint;
        OGRPoint* P2 = new OGRPoint;
        RingL->getPoint(l, P1);
        RingL->getPoint(l + 1, P2);
        OGRLineString* Line = new OGRLineString;
        Line->addPoint(P1);
        Line->addPoint(P2);

        std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

        for(OGRPoint* P:ListPoints)
        {
            if(P->Intersects(P1) || P->Intersects(P2)) //Si le point existe déjà, inutile de le recréer
                continue;

			if(P->Distance(P1) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P1 (on garde z de P1)
			{
				P1->setX(P->getX());
				P1->setY(P->getY());
				continue;
			}
			if(P->Distance(P2) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P2 (on garde z de P2)
			{
				P2->setX(P->getX());
				P2->setY(P->getY());
				continue;
			}

            if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
            {
                bool AlreadyInVec = false;
                for(OGRPoint* Ptemp:PointsCut)
                {
                    if(P->Intersects(Ptemp))
                    {
                        AlreadyInVec = true;
                        break;
                    }
                }
                if(!AlreadyInVec)
                {
					double zP = P1->getZ() + (P->getX() - P1->getX()) / (P2->getX() - P1->getX()) * (P2->getZ() - P1->getZ());
					P->setZ(zP);
                    PointsCut.push_back((OGRPoint*)P->clone());
                    continue;
                }
            }
        }

        ResRing->addPoint(P1);
        while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
        {
            OGRPoint* P = PointsCut.at(0);
            double Dist = P->Distance(P1);
            int indice = 0;
            for(int j = 1; j < PointsCut.size(); ++j)
            {
                OGRPoint* PointTemp = PointsCut.at(j);
                double DistTemp = PointTemp->Distance(P1);
                if(DistTemp < Dist)
                {
                    P = PointTemp;
                    Dist = DistTemp;
                    indice = j;
                }
            }
            ResRing->addPoint(P);

            delete P;
            PointsCut.erase(PointsCut.begin() + indice);
        }

        delete P1;
        delete P2;
        delete Line;
    }
    ResRing->closeRings();
    ResPoly->addRing(ResRing);

    delete ResRing;

	//// Même traitement sur les InteriorRing du Polygon Lines

    for(int r = 0; r < PolyL->getNumInteriorRings(); ++r)
    {
        ResRing = new OGRLinearRing;
        RingL = PolyL->getInteriorRing(r);

        for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
        {
            OGRPoint* P1 = new OGRPoint;
            OGRPoint* P2 = new OGRPoint;
            RingL->getPoint(l, P1);
            RingL->getPoint(l + 1, P2);
            OGRLineString* Line = new OGRLineString;
            Line->addPoint(P1);
            Line->addPoint(P2);

            std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

            for(OGRPoint* P:ListPoints)
            {
                if(P->Intersects(P1) || P->Intersects(P2))
                    continue;

				if(P->Distance(P1) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P1 (on garde z de P1)
				{
					P1->setX(P->getX());
					P1->setY(P->getY());
					continue;
				}
				if(P->Distance(P2) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P2 (on garde z de P2)
				{
					P2->setX(P->getX());
					P2->setY(P->getY());
					continue;
				}

                if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
                {
                    bool AlreadyInVec = false;
                    for(OGRPoint* Ptemp:PointsCut)
                    {
                        if(P->Intersects(Ptemp))
                        {
                            AlreadyInVec = true;
                            break;
                        }
                    }
                    if(!AlreadyInVec)
                    {
                        double zP = P1->getZ() + (P->getX() - P1->getX()) / (P2->getX() - P1->getX()) * (P2->getZ() - P1->getZ());
						P->setZ(zP);
						PointsCut.push_back((OGRPoint*)P->clone());
						continue;
                    }
                }
            }

            ResRing->addPoint(P1);
            while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
            {
                OGRPoint* P = PointsCut.at(0);
                double Dist = P->Distance(P1);
                int indice = 0;
                for(int j = 1; j < PointsCut.size(); ++j)
                {
                    OGRPoint* PointTemp = PointsCut.at(j);
                    double DistTemp = PointTemp->Distance(P1);
                    if(DistTemp < Dist)
                    {
                        P = PointTemp;
                        Dist = DistTemp;
                        indice = j;
                    }
                }
                ResRing->addPoint(P);

                delete P;
                PointsCut.erase(PointsCut.begin() + indice);
            }

            delete P1;
            delete P2;
            delete Line;
        }
        ResRing->closeRings();
        ResPoly->addRing(ResRing);

        delete ResRing;
    }

	for(OGRPoint* P:ListPoints)
		delete P;

    return ResPoly;
}

/**
* @brief Parcourt les points des polygones de Points, regarde s'ils semblent appartenir à une arête de Lines et si c'est le cas, on coupe cette arête en deux afin que ce point existe et que l'intersection fonctionne.
* On retourne une GeometryCollection qui contient les mêmes polygones que Lines, mais avec des arêtes parfois partagées en deux. Globalement ça ne change rien, mais on pourra calculer l'intersection entre les points de Points et celles ci.
* @param Points Liste de polygones auxquels on va s'intéresser pour leurs points
* @param Lines Liste de polygones auxquels on va s'intéresser pour leurs arêtes
*/
OGRGeometryCollection* CreatePointsOnLine(OGRGeometryCollection* Points, OGRGeometryCollection* Lines)
{
	OGRGeometryCollection* Res = new OGRMultiPolygon;

	for(int i = 0; i < Lines->getNumGeometries(); ++i)
	{
		OGRGeometry * GeoL = Lines->getGeometryRef(i);
		if(GeoL->getGeometryType() != wkbPolygon && GeoL->getGeometryType() != wkbPolygon25D)
			continue;
		OGRPolygon * PolyL = (OGRPolygon*) GeoL;
		OGRLinearRing * RingL = PolyL->getExteriorRing();

		OGRPolygon * ResPoly = new OGRPolygon;
		OGRLinearRing * ResRing = new OGRLinearRing;

		for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
		{
			OGRPoint* P1 = new OGRPoint;
			OGRPoint* P2 = new OGRPoint;
			RingL->getPoint(l, P1);
			RingL->getPoint(l + 1, P2);
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(P1);
			Line->addPoint(P2);

			std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

			for(int j = 0; j < Points->getNumGeometries(); ++j)
			{
				OGRGeometry * GeoP = Points->getGeometryRef(j);
				if(GeoP->getGeometryType() != wkbPolygon && GeoP->getGeometryType() != wkbPolygon25D)
					continue;
				OGRPolygon * PolyP = (OGRPolygon*) GeoP;
				OGRLinearRing * RingP = PolyP->getExteriorRing();
				for(int p = 0; p < RingP->getNumPoints() - 1; ++p)
				{
					OGRPoint* P = new OGRPoint;
					RingP->getPoint(p, P);

                    if(P->Intersects(P1) || P->Intersects(P2))
                    {
                        delete P;
                        continue;
                    }

					if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
					{
						bool AlreadyInVec = false;
						for(OGRPoint* Ptemp:PointsCut)
						{
                            if(P->Intersects(Ptemp))
							{
                                AlreadyInVec = true;
                                delete P;
								break;
							}
						}
                        if(!AlreadyInVec)
                        {
                            PointsCut.push_back(P);
                            continue;
                        }
                    }
                    delete P;
				}
			}

			ResRing->addPoint(P1);
			while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
			{
				OGRPoint* P = PointsCut.at(0);
				double Dist = P->Distance(P1);
				int indice = 0;
				for(int j = 1; j < PointsCut.size(); ++j)
				{
					OGRPoint* PointTemp = PointsCut.at(j);
					double DistTemp = PointTemp->Distance(P1);
					if(DistTemp < Dist)
					{
						P = PointTemp;
						Dist = DistTemp;
						indice = j;
					}
				}
                ResRing->addPoint(P);

				delete P;
				PointsCut.erase(PointsCut.begin() + indice);
            }

            delete P1;
            delete P2;
            delete Line;
		}
		ResRing->closeRings();
		ResPoly->addRing(ResRing);

		delete ResRing;

		for(int r = 0; r < PolyL->getNumInteriorRings(); ++r)
		{
			ResRing = new OGRLinearRing;
			RingL = PolyL->getInteriorRing(r);

			for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
			{
				OGRPoint* P1 = new OGRPoint;
				OGRPoint* P2 = new OGRPoint;
				RingL->getPoint(l, P1);
				RingL->getPoint(l + 1, P2);
				OGRLineString* Line = new OGRLineString;
				Line->addPoint(P1);
				Line->addPoint(P2);
			
				std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing
			
				for(int j = 0; j < Points->getNumGeometries(); ++j)
				{
					OGRGeometry * GeoP = Points->getGeometryRef(j);
					if(GeoP->getGeometryType() != wkbPolygon && GeoP->getGeometryType() != wkbPolygon25D)
						continue;
					OGRPolygon * PolyP = (OGRPolygon*) GeoP;
					OGRLinearRing * RingP = PolyP->getExteriorRing();
					for(int p = 0; p < RingP->getNumPoints() - 1; ++p)
					{
						OGRPoint* P = new OGRPoint;
						RingP->getPoint(p, P);
			
                        if(P->Intersects(P1) || P->Intersects(P2))
                        {
                            delete P;
							continue;
                        }
			
						if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
						{
							bool AlreadyInVec = false;
							for(OGRPoint* Ptemp:PointsCut)
							{
                                if(P->Intersects(Ptemp))
								{
									AlreadyInVec = true;
                                    delete P;
									break;
								}
							}
                            if(!AlreadyInVec)
                            {
                                PointsCut.push_back(P);
                                continue;
                            }
                        }
                        delete P;
					}
				}
			
				ResRing->addPoint(P1);
				while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
				{
					OGRPoint* P = PointsCut.at(0);
					double Dist = P->Distance(P1);
					int indice = 0;
					for(int j = 1; j < PointsCut.size(); ++j)
					{
						OGRPoint* PointTemp = PointsCut.at(j);
						double DistTemp = PointTemp->Distance(P1);
						if(DistTemp < Dist)
						{
							P = PointTemp;
							Dist = DistTemp;
							indice = j;
						}
					}
                    ResRing->addPoint(P);
			
					delete P;
					PointsCut.erase(PointsCut.begin() + indice);
                }

                delete P1;
                delete P2;
                delete Line;
			}
			ResRing->closeRings();
			ResPoly->addRing(ResRing);

			delete ResRing;
		}

        Res->addGeometryDirectly(ResPoly);
	}

	//SaveGeometrytoShape("VecShape1.shp", Lines);
	//SaveGeometrytoShape("VecShape2.shp", Res);

	return Res;
}

/**
* @brief Parcourt tous polygons/bâtiments du cadastre et modifie leurs enveloppes pour correspondre au mieux à celles du CityGML
* @param VecGML contient les emprises au sol issues du CityGML
* @param VecShape contient les emprises au sol issues du Shapefile et que l'on va modifier puis exporter dans un nouveau vector de polygons
* @param Link contient les liens entre les emprises au sol des deux fichiers qui s'intersectent
*/
std::vector<OGRPolygon*> FusionEnvelopes(std::vector<OGRPolygon*>* VecGML, std::vector<OGRPolygon*>* VecShape, std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>>* Link)
{
	////////////////////////A vérifier si le cas plusieurs Bati CityGML pour un seul Bati Shape fonctionne ! //////////////////////////////

	std::vector<OGRPolygon*> NewVecShape;//Vector résultat
	for(OGRPolygon* Poly:*VecShape)
		NewVecShape.push_back((OGRPolygon*)Poly->clone());

	//OGRMultiPolygon* CutPolygons = new OGRMultiPolygon;
	//OGRMultiLineString* CutLines = new OGRMultiLineString;

	for(int i = 0; i < VecGML->size(); ++i) //Parcourt les bâtiments du CityGML
	{
		//if(i%10 == 0)
			std::cout << "Batiment CityGML : " << i << " / " << VecGML->size() << std::endl;

		/////////////////////////// Pour chaque bâtiment CityGML, on soustrait à son emprise au sol celles des bâtiments cadastraux qui lui sont liés. Cette soustraction créé des points intermédiaires sur des arêtes droites
		/////////////////////////// qui correspondent aux endroits où deux bâtiments cadastraux se touchent. ->GMLDiffShape
		if(VecGML->at(i) == nullptr)
			continue;
		if(Link->first.at(i).size() == 0) // Le bâtiment CityGML n'est lié à aucun bâtiment cadastral, on passe donc au suivante
			continue;

		OGRGeometry* GMLDiffShape = VecGML->at(i)->clone();
		for(int j : Link->first.at(i)) //On parcourt tous les polygones du Shape liés au bâtiment CityGML courant
		{
			OGRGeometry* tmp = GMLDiffShape;
			GMLDiffShape = tmp->Difference(VecShape->at(j));
			delete tmp;
		}
		if(GMLDiffShape->IsEmpty()) //Cela signifie que l'emprise au sol du Bâtiment CityGML est complètement incluse dans les polygones du cadastre, il n'y a donc rien à faire à cette étape
			continue;
		
		/////////////////////////// Transformation de GMLDiffShape en GeometryCollection pour pouvoir parcrourir ses éléments un par un -> GC_GMLDiffShape

		OGRGeometryCollection* GC_GMLDiffShape = new OGRGeometryCollection;
		if(GMLDiffShape->getGeometryType() == wkbPolygon || GMLDiffShape->getGeometryType() == wkbPolygon25D)
			GC_GMLDiffShape->addGeometryDirectly(GMLDiffShape);
		else if(GMLDiffShape->getGeometryType() == wkbGeometryCollection || GMLDiffShape->getGeometryType() == wkbGeometryCollection25D || GMLDiffShape->getGeometryType() == wkbMultiPolygon || GMLDiffShape->getGeometryType() == wkbMultiPolygon25D)
			GC_GMLDiffShape = (OGRGeometryCollection*) GMLDiffShape;

		/////////////////////////// L'objectif est d'assigner chaque polygone de GC_GMLDiffShape à des emprises au sol du cadastre pour que celles ci recouvrent la surface totalement du bâtiment CityGML. Cependant, certains polygones de
		/////////////////////////// GC_GMLDiffShape peuvent sembler pouvoir être assignés à deux (ou plus) polygones du cadastre, il faut donc les partager en plus petite parties qui seront chacune assignées à un bâtiment cadastral
		/////////////////////////// On va parcourir chaque polygone de GC_GMLDiffShape, regarder s'il partage plusieurs bâtiments cadastraux et si c'est le cas, le découper en plusieurs petits polygones. On assigne ensuite chaque polygone
		/////////////////////////// à un bâtiment cadastral et on unit les emprises au sol afin d'en générer une de plus en plus grosse, jusqu'à ce que la somme des emprises au sol du cadastre recouvre entièrement le bâtiment CityGML

		for(int g = 0; g < GC_GMLDiffShape->getNumGeometries(); ++g)
		{
			OGRGeometry* GeometryGMLDiffShape = GC_GMLDiffShape->getGeometryRef(g);
			if(GeometryGMLDiffShape->getGeometryType() != wkbPolygon && GeometryGMLDiffShape->getGeometryType() != wkbPolygon25D) //Si ce n'est pas un polygon, on passe au suivant
				continue;

			OGRPolygon* PolygonGMLDiffShape = (OGRPolygon*) GeometryGMLDiffShape;
			if(PolygonGMLDiffShape->get_Area() < 10*Precision_Vect)//Filtre sur l'aire des polygons pour enlever ceux issus des imprécisions des fichiers vectoriels (lorsqu'un point ne colle pas parfaitement à la ligne d'un polygone voisin)
				continue;

			/////////////////////////// On va parcourir tous les points du Ring exterior du polygon courant de GC_GMLDiffShape, et compter le nombre de bâtiments cadastraux sur lequel chaque point est également situé. Si ce nombre
			/////////////////////////// est d'au moins 2, alors ce point fait partie de la frontière entre deux bâtiments cadastraux, cela signifie qu'il faut se servir de ce point pour découper PolygonGMLDiffShape en deux. Ces deux polygones
			/////////////////////////// seront ensuite assignés aux deux bâtiments cadastraux concernés. Pour l'instant, on se contente de projeter ce point pour trouver le second avec lequel il formera la ligne qui coupera le polygon en deux.

			OGRLinearRing* RingGMLDiffShape = PolygonGMLDiffShape->getExteriorRing();

			std::vector<OGRPoint*> ListAProjetes; //Contiendra tous les points que l'on aura projetés 
			std::vector<OGRPoint*> ListProjetes; //Contiendra tous les points projetés dont on se servira afin de partager des LineString en deux
			std::vector<OGRPoint*> ListAProjetesModifies; //Contiendra les points projetés "modifiés" : qui ne correspondent pas forcément à des points du Polygone et qu'il faudra donc créer sur le MultiLS
			OGRMultiLineString* MultiLS = new OGRMultiLineString; //Contiendra les LineString qui serviront à générer les polygones désirés

			for(int j = 0; j < RingGMLDiffShape->getNumPoints() - 1; ++j) // -1 car il ne faut pas reparcourir le premier point qui est bouclé à la fin
			{
				OGRPoint * Point = new OGRPoint;
				RingGMLDiffShape->getPoint(j, Point);

				OGRPoint * PointNext = new OGRPoint;
				RingGMLDiffShape->getPoint(j + 1, PointNext);
				OGRLineString* LS = new OGRLineString;
				LS->addPoint(Point);
				LS->addPoint(PointNext);
				delete PointNext;

				MultiLS->addGeometryDirectly(LS); //Pour remplir MultiLS qui servira dans l'étape suivante

				int ShapesByPoint = 0; //Compte le nombre de polygons du Shape qui contiennent également le point courant
				for(int k : Link->first.at(i)) //On parcourt tous les polygones du Shape liés au bâtiment CityGML courant
				{
                    if(Point->Intersects(VecShape->at(k)))
						++ShapesByPoint;
					if(ShapesByPoint > 1) //Cela signifie que ce point est partagé par plusieurs emprises au sol de cadastre -> point que l'on doit projeter sur l'enveloppe du CityGML pour diviser le polygon g de GMLDiffShape
					{
						OGRLineString* PrevLine; //On calcule l'arête précédente car elle contient également Point
						if(j == 0) //On est obligé d'aller chercher l'arête précédente à la fin du LinearRing
						{
							PrevLine = new OGRLineString;
							OGRPoint* PrevPoint = new OGRPoint;
							RingGMLDiffShape->getPoint(RingGMLDiffShape->getNumPoints() - 2, PrevPoint); //Il faut aller le chercher le -2 car le -1 est le même que Point;
							PrevLine->addPoint(PrevPoint);
							PrevLine->addPoint(Point);
						}
						else
							PrevLine = (OGRLineString*)MultiLS->getGeometryRef(MultiLS->getNumGeometries() - 2); //On vient d'ajouter le LS courant, donc il faut aller chercher le -2

						bool PointIsModified = false;

						OGRPoint* Projete = ProjectPointOnEnvelope(Point, PolygonGMLDiffShape, LS, PrevLine, &PointIsModified);

						if(Projete->getX() == 0.0 && Projete->getY() == 0.0) //La projection n'est pas nécessaire (si c'est sur un bord du polygon)
							break;

						if(PointIsModified)
							ListAProjetesModifies.push_back((OGRPoint*)Point->clone());

						ListAProjetes.push_back(Point);
						ListProjetes.push_back(Projete);

						break;//Inutile de continuer à parcourir les autres polygons du Shape, cela ne changera rien au résultat puisque la projection sera le même qu'il y ait 2 ou + polygones associés à un point donné
					}
				}
			}

			for(int r = 0; r < PolygonGMLDiffShape->getNumInteriorRings(); ++r) //Il faut faire pareil sur les Interior Ring, au cas où le CityGML enveloppe entièrement un bâtiment shape
			{
				OGRLinearRing* IntRingGMLDiffShape = PolygonGMLDiffShape->getInteriorRing(r);

				for(int j = 0; j < IntRingGMLDiffShape->getNumPoints() - 1; ++j) // -1 car il ne faut pas reparcourir le premier point qui est bouclé à la fin
				{
					OGRPoint * Point = new OGRPoint;
					IntRingGMLDiffShape->getPoint(j, Point);

					OGRPoint * PointNext = new OGRPoint;
					IntRingGMLDiffShape->getPoint(j + 1, PointNext);
					OGRLineString* LS = new OGRLineString;
					LS->addPoint(Point);
					LS->addPoint(PointNext);
					delete PointNext;

					MultiLS->addGeometryDirectly(LS); //Pour remplir MultiLS qui servira dans l'étape suivante

					int ShapesByPoint = 0; //Compte le nombre de polygons du Shape qui contiennent également le point courant
					for(int k : Link->first.at(i)) //On parcourt tous les polygones du Shape liés au bâtiment CityGML courant
					{
						if(Point->Intersects(VecShape->at(k)))
							++ShapesByPoint;
						if(ShapesByPoint > 1) //Cela signifie que ce point est partagé par plusieurs emprises au sol de cadastre -> point que l'on doit projeter sur l'enveloppe du CityGML pour diviser le polygon g de GMLDiffShape
						{
							OGRLineString* PrevLine; //On calcule l'arête précédente car elle contient également Point
							if(j == 0) //On est obligé d'aller chercher l'arête précédente à la fin du LinearRing
							{
								PrevLine = new OGRLineString;
								OGRPoint* PrevPoint = new OGRPoint;
								IntRingGMLDiffShape->getPoint(IntRingGMLDiffShape->getNumPoints() - 2, PrevPoint); //Il faut aller le chercher le -2 car le -1 est le même que Point;
								PrevLine->addPoint(PrevPoint);
								PrevLine->addPoint(Point);
							}
							else
								PrevLine = (OGRLineString*)MultiLS->getGeometryRef(MultiLS->getNumGeometries() - 2); //On vient d'ajouter le LS courant, donc il faut aller chercher le -2

							bool PointIsModified = false;

							OGRPoint* Projete = ProjectPointOnEnvelope(Point, PolygonGMLDiffShape, LS, PrevLine, &PointIsModified);

							if(Projete->getX() == 0.0 && Projete->getY() == 0.0) //La projection n'est pas nécessaire (si c'est sur un bord du polygon)
								break;

							if(PointIsModified)
								ListAProjetesModifies.push_back((OGRPoint*)Point->clone());

							ListAProjetes.push_back(Point);
							ListProjetes.push_back(Projete);

							break;//Inutile de continuer à parcourir les autres polygons du Shape, cela ne changera rien au résultat puisque la projection sera le même qu'il y ait 2 ou + polygones associés à un point donné
						}
					}
				}
			}

			//////// !!!!! : Traiter le cas où un Polygon possède un InteriorRing et une seule projection qui le découpe : Risque de ne pas découper le polygon en deux car l'arête coupante s'arrêtera à l'InteriorRing.

			/////////////////////////// On a une liste de couples de points (en deux vector remplis simultanément) qui doivent partager PolygonGMLDiffShape. Pour mettre en place cette découpe, on récupère la liste LS des arêtes du polygone,
			/////////////////////////// on la parcourt en testant chaque ligne pour voir si elle contient (l'intersection ne fonctionne pas, test sur la distance) un des points projetés précédemment calculés. 
			/////////////////////////// Si c'est le cas, on partage cette ligne en deux afin d'intégrer ce point dans LS, et on ajoute également la ligne formée par ce point et celui qui l'a généré au départ, celui dont il est le projeté.
			
			/*OGRGeometry* tmpMultiLS = MultiLS->clone(); ///////////////////////// DEBUG
			
			OGRMultiPoint* LAP = new OGRMultiPoint; ////// DEBUG
			OGRMultiPoint* LAPM = new OGRMultiPoint; ////// DEBUG
			OGRMultiPoint* LP = new OGRMultiPoint; ////// DEBUG
			for(OGRPoint* PointTemp : ListAProjetes)
			{
				LAP->addGeometry(PointTemp);
			}
            for(OGRPoint* PointTemp : ListAProjetesModifies)
			{
				LAPM->addGeometry(PointTemp);
			}
            for(OGRPoint* PointTemp : ListProjetes)
			{
				LP->addGeometry(PointTemp);
			}*/


			for(int j = 0; j < MultiLS->getNumGeometries(); ++j)
			{
				OGRLineString* LS = (OGRLineString*)MultiLS->getGeometryRef(j);
				OGRPoint* Point1 = new OGRPoint;
				LS->getPoint(0, Point1);
				OGRPoint* Point2 = new OGRPoint;
				LS->getPoint(1, Point2);

				bool test = false;
				int l = 0;
				for(OGRPoint* Projete:ListProjetes)
				{
					if(Projete->Distance(LS) < Precision_Vect) //Utilisation de Distance avec un seuil très faible car l'intersection ne fonctionne pas entre un point et un segment qui est censé passer par ce point (à part si c'est un des deux points du segment)
					{
						OGRLineString* LS1 = new OGRLineString;
						LS1->addPoint(Point1);
						LS1->addPoint(Projete);
						OGRLineString* LS2 = new OGRLineString;
						LS2->addPoint(Projete);
						LS2->addPoint(Point2);

						OGRLineString* LS3 = new OGRLineString; //Ligne qui "coupe en deux" les polygones
						LS3->addPoint(Projete);
						LS3->addPoint(ListAProjetes.at(l));

						delete ListProjetes.at(l);
						delete ListAProjetes.at(l);
						ListProjetes.erase(ListProjetes.begin() + l);
						ListAProjetes.erase(ListAProjetes.begin() + l);
						MultiLS->removeGeometry(j);
						MultiLS->addGeometryDirectly(LS1);
						MultiLS->addGeometryDirectly(LS2);
						MultiLS->addGeometryDirectly(LS3);
						--j;
						test = true;
						break;
					}
					++l;
				}

				if(test) //Si on a déjà ajouté des points dans MultiLS via la boucle précédente, on sort.
					continue;

				l = 0;
				
				for(OGRPoint* AProjete:ListAProjetesModifies) //Il faut s'assurer que les points à projeter qui ont changé, donc qui ne sont plus forcément directement sur le polygon, soient bien présents dans MultiLS
				{
					if(AProjete->Distance(LS) < Precision_Vect && AProjete->Distance(Point1) > Precision_Vect && AProjete->Distance(Point2) > Precision_Vect)
					{
						OGRLineString* LS1 = new OGRLineString;
						LS1->addPoint(Point1);
						LS1->addPoint(AProjete);
						OGRLineString* LS2 = new OGRLineString;
						LS2->addPoint(AProjete);
						LS2->addPoint(Point2);

						delete ListAProjetesModifies.at(l);
						ListAProjetesModifies.erase(ListAProjetesModifies.begin() + l);

						MultiLS->removeGeometry(j);
						MultiLS->addGeometryDirectly(LS1);
						MultiLS->addGeometryDirectly(LS2);
						--j;
						break;
					}
					++l;
				}
			}

            for(OGRPoint* PointTemp : ListAProjetes)
				delete PointTemp;
            for(OGRPoint* PointTemp : ListAProjetesModifies)
                delete PointTemp;
            for(OGRPoint* PointTemp : ListProjetes)
                delete PointTemp;

			/////////////////////////// Grâce aux nouvelles lignes insérées, on utilise la fonction Polygonize qui va les prendre en compte pour générer un nouvel ensemble de polygone. Si la ligne coupe PolygonGMLDiffShape en deux, Polygonize va 
			/////////////////////////// ressortir les deux polygons désirés.

			//for(int  j = 0; j < MultiLS->getNumGeometries(); ++j)
			//	CutLines->addGeometry(MultiLS->getGeometryRef(j)->clone());

			OGRGeometryCollection* SplitPolygon = (OGRGeometryCollection*) MultiLS->Polygonize();

			if(SplitPolygon->IsEmpty())
			{
				std::cout << "ERREUR : SPLIT POLYGON VIDE !!" << std::endl;
				SaveGeometrytoShape("GeometryGMLDiffShape.shp", GeometryGMLDiffShape);
				SaveGeometrytoShape("MultiLS.shp", MultiLS);
				/*SaveGeometrytoShape("MultiLS_Old.shp", tmpMultiLS);
				SaveGeometrytoShape("ListAProjetes.shp", LAP);
				SaveGeometrytoShape("ListAProjetesModifies.shp", LAPM);
				SaveGeometrytoShape("ListProjetes.shp", LP);*/
				delete MultiLS;
				int a;
				std::cin >> a;
				continue;
			}

			//delete MultiLS;

            /// On parcourt les polygones de SplitPolygon nouvellement créés pour mettre à jour les Polygon du Shape auxquels ils sont liés en ajoutant les éventuels points nécessaires
            /// A une intersection/union fonctionnelle. On profite également de ce parcourt pour repérer à quel bâtiment du Shape chaque polygon de SplitPolygon va devoir être ajouté.

			for(int j = 0; j < SplitPolygon->getNumGeometries(); ++j)
            {
                //CutPolygons->addGeometry(Geo->clone());
				OGRGeometry* Geo = SplitPolygon->getGeometryRef(j); //C'est forcément un Polygon

				//Il faut retirer les Interior Ring de PolygonGMLDiffShape qui ont créé des polygons à cause de Polygonize : on calcule l'intersection et si l'intersection n'est pas un Polygon, c'est que Geo est un InteriorRing
				OGRGeometry* InterGEO_GMLDiffShape = Geo->Intersection(PolygonGMLDiffShape);
				if(InterGEO_GMLDiffShape->getGeometryType() != wkbPolygon && InterGEO_GMLDiffShape->getGeometryType() != wkbPolygon25D)
					continue;

				if(((OGRPolygon*)Geo)->get_Area() < 10 * Precision_Vect)
					continue;

                double distance = 0;
                double area = 0;
                int indice = -1;
                int indice2 = -1;

                for(int k : Link->first.at(i))
                {
                    if(Geo->Distance(VecShape->at(k)) > Precision_Vect) //Le polygon de SplitPolygon n'est pas lié au bâtiment shape courant, on passe au suivant
                        continue;

					/////////////////////////// Certains points de Geo qui sont censés s'intersecter avec les polygones cadastraux (qui ont servi à leur génération) ne correspondent pas à des points sur ceux ci (par exemple
					/////////////////////////// ils se retrouvent au milieu d'une ligne d'un polygone cadastral). Du coup l'intersection ne fonctionne pas. Il faut donc créer ces points sur les lignes concernées car l'intersection Point-Point fonctionne.

                    OGRGeometry* ShapeWithPoints = CreatePointsOnLine(Geo, VecShape->at(k)); //On considère que ces deux polygons s'intersectent, on va donc créer les points de Geo sur VecShape pour que ce soit effectivement le cas pour GDAL

                    OGRGeometry* Inter = Geo->Intersection(ShapeWithPoints); //Inter peut être Point, LineString, MultiLineString ou GeometryCollection (Point + Line ?)

                    if(Inter == nullptr)
                    {
                        indice = -1; //Voir slide 7 du Pwp CutCityGMLShape pour cas d'application où on passe ici.
                        indice2 = -1;

						//SaveGeometrytoShape("Geo1.shp", Geo);
						//SaveGeometrytoShape("Geo2.shp", VecShape->at(k));
						//SaveGeometrytoShape("Geo3.shp", ShapeWithPoints);
						//int a;
						//std::cin >> a;
						delete ShapeWithPoints;

                        break;//Pour pas que Geo soit assigné à un autre k car il y aurait alors du recouvrement entre les deux, on n'associe Geo à aucun polygon du Shape.
                    }
					delete ShapeWithPoints;
                    OGRGeometryCollection* InterGC = dynamic_cast<OGRGeometryCollection*>(Inter);

                    if(InterGC == nullptr) //Si Inter est un  Point ou un LineString, il ne pourra être converti en GC donc on va l'ajouter pour en faire un GC avec un seul élément, ce qui n'est pas gênant et évite trop de if
                    {
                        InterGC = new OGRGeometryCollection;
                        InterGC->addGeometry(Inter);
                    }
                    double distanceTemp = 0;
                    double areaTemp = 0;
                    for(int z = 0; z < InterGC->getNumGeometries(); ++z)
                    {
                        OGRGeometry* CurrGeo = InterGC->getGeometryRef(z);
                        if(CurrGeo->getGeometryType() == wkbLineString || CurrGeo->getGeometryType() == wkbLineString25D)
                            distanceTemp += ((OGRLineString*)CurrGeo)->get_Length();
                        else if(CurrGeo->getGeometryType() == wkbPolygon || CurrGeo->getGeometryType() == wkbPolygon25D)
                            areaTemp += ((OGRPolygon*)CurrGeo)->get_Area();
                    }
                    if(distanceTemp > distance)
                    {
                        distance = distanceTemp;
                        indice = k;
                    }
                    if(areaTemp > area)
                    {
                        area = areaTemp;
                        indice2 = k;
                    }
					if(indice == -1)
						indice = k; //Si aucun indice n'a été relevé et qu'on est arrivé jusqu'ici, cela signifie qu'il y a une intersection (Point) et qu'il faut donc l'ajouter au cas où on en trouve pas d'autre. On laisse distance à 0 
									//Pour qu'il soit facilement changé si une intersection LineString ou Polygon est trouvée
                    delete Inter;
                }

                if(indice2 != -1) //On privilégie les intersections avec des aires, dû à des imprécisions mais qui témoignent d'une certaine proximité.
                {
					/*SaveGeometrytoShape("Geo.shp", Geo);
					SaveGeometrytoShape("SplitPolygon.shp", SplitPolygon);
					SaveGeometrytoShape("MultiLS.shp", MultiLS);
					SaveGeometrytoShape("Geo1.shp", NewVecShape.at(indice));
					SaveGeometrytoShape("Geo2.shp", NewVecShape.at(indice2));
					std::cout << "AIRE " << area << "  " << distance << std::endl;
					int a;
					std::cin >> a;*/
                    indice = indice2;
                }
                if(indice == -1) //Le polygon découpé n'a pas réussi à se lier à un bati Shape : A DEBUG
                {
                    std::cout << "INDICE -1" << std::endl;

					SaveGeometrytoShape("Geo1.shp", Geo);
					int a;
					std::cin >> a;
                    continue;
                }

                OGRGeometry* ShapeWithPoints = CreatePointsOnLine(Geo, NewVecShape.at(indice));

                OGRGeometry* ShapeUnion = ShapeWithPoints->Union(Geo);

                delete ShapeWithPoints;

				if(ShapeUnion->getGeometryType() != wkbPolygon && ShapeUnion->getGeometryType() != wkbPolygon25D) //A cause des imprécisions, on peut avoir un MultiPolygon ... Il faut combler les trous pour obtenir un Polygon
                {
                    OGRGeometryCollection* GC = (OGRGeometryCollection*)ShapeUnion;
					OGRGeometry* Test = new OGRPolygon;
                    for(int m = 0; m < GC->getNumGeometries(); ++m)
                    {
                        //SaveGeometrytoShape("Geo" + std::to_string(m) + ".shp", GC->getGeometryRef(m));
                        //std::cout << GC->getGeometryRef(m)->getGeometryName() << std::endl;
						if(GC->getGeometryRef(m)->getGeometryType() == wkbPolygon || GC->getGeometryRef(m)->getGeometryType() == wkbPolygon25D)
						{
							OGRGeometry* tmp = Test;
							Test = tmp->Union(GC->getGeometryRef(m));
							delete tmp;
						}
                    }
					delete NewVecShape.at(indice);
					NewVecShape.at(indice) = (OGRPolygon*) Test;
				}
				else
                {
                    delete NewVecShape.at(indice);
                    NewVecShape.at(indice) = (OGRPolygon*) ShapeUnion;
                }
            }
			delete SplitPolygon;
		}
		delete GC_GMLDiffShape;
    }

	//SaveGeometrytoShape("CutPolygons.shp", CutPolygons); //TODO : faire une fonction qui soustrait le cadastre au CityGML et ressort les différences (calcul d'air, shapefile des différences, ...)
	//SaveGeometrytoShape("CutLines.shp", CutLines);

    OGRMultiPolygon* NewShape = new OGRMultiPolygon;

    for(OGRPolygon* Poly:NewVecShape)
	{
        NewShape->addGeometry(Poly);
	}

    SaveGeometrytoShape("NewShape.shp", NewShape);

    delete NewShape;

    return NewVecShape;
}

/**
* @brief Compare une par une les emprises au sol des deux vector afin de savoir quelles sont celles qui s'intersectent. Pour chaque bâtiment d'un vector, on connait la liste de ceux de l'autre vector qui l'intersectent (avec les indices)
* @param Vec1 contient les emprises au sol de la première base de données (CityGML)
* @param Vec2 contient les emprises au sol de la seconde base de données (Shape)
*/
std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> LinkCityGMLandShapefilesFootprints(std::vector<OGRPolygon*> Vec1, std::vector<OGRPolygon*> Vec2)
{
	std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> Link;

	Link.first.resize(Vec1.size());
	Link.second.resize(Vec2.size());

	for(int i = 0; i < Vec1.size(); ++i)
	{
		if(Vec1.at(i) == nullptr)
			continue;
		OGRPolygon* Poly1 = Vec1.at(i);
		for(int j = 0; j < Vec2.size(); ++j)
		{
			if(Vec2.at(j) == nullptr)
				continue;
			OGRPolygon* Poly2 = Vec2.at(j);
            if(!Poly1->Intersects(Poly2))
				continue;

			OGRGeometry* Inter = Poly1->Intersection(Poly2);
			if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D || Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D)
			{
				Link.first[i].push_back(j);
				Link.second[j].push_back(i);
			}
            delete Inter;
		}
	}
	return Link;
}

/**
* @brief Découpe un polygon 3D GMLPoly suivant un second polygon 2D BuildingShp. Il faut que le polygon découpé soit encore en 3d.
* @param GMLPoly représente le premier polygon 3D (issu du fichier CityGML)
* @param BuildingShp représente le second polygon 2D (issu du fichier Shape et modifié pour coller aux emprises du CityGML)
*/
OGRGeometry * CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp, std::vector<TVec2f> *TexUV, std::vector<std::vector<TVec2f>>* TexUVout)
{
    //SaveGeometrytoShape("A_GMLPoly.shp", GMLPoly);
    //SaveGeometrytoShape("A_BuildingShp.shp", BuildingShp);

    OGRGeometry* Inter = GMLPoly->Intersection(BuildingShp);

    //On va parcourir chaque point P de Inter et calculer sa position dans GMLPoly afin de calculer sa coordonnée z (+ TODO : texture)
    //On commence par récupérer trois points A, B et C de GMLPoly non alignés pour obtenir l'équation paramétrique du plan formé par ce polygon
    OGRLinearRing* GMLRing = GMLPoly->getExteriorRing();

    TVec3d A;
    TVec3d B;
    TVec3d C;
	TVec2f uvA;
	TVec2f uvB;
	TVec2f uvC;
	TVec2f uvAB;
	TVec2f uvAC;

    A.x = GMLRing->getX(0);
    A.y = GMLRing->getY(0);
    A.z = GMLRing->getZ(0);

	uvA = TexUV->at(0);

    TVec3d AB;
    TVec3d AC;

    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
    for(int i = 1; i < GMLRing->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une répétition du premier
    {
        if(test == 0)
        {
            B.x = GMLRing->getX(i);
            B.y = GMLRing->getY(i);
            B.z = GMLRing->getZ(i);

            if(A.x != B.x || A.y != B.y)
            {
                ++test;// A est bien différent de B
                AB = B - A;
				uvB = TexUV->at(i);
				uvAB = uvB - uvA;
            }
        }
        else if(test == 1)
        {
            C.x = GMLRing->getX(i);
            C.y = GMLRing->getY(i);
            C.z = GMLRing->getZ(i);

            if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y))
            {
                ++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
                AC = C - A;
				uvC = TexUV->at(i);
				uvAC = uvC - uvA;
                break;
            }
        }
    }

    if(test != 2)
    {
        std::cout << "Erreur lors de la creation du plan. \n";
        delete Inter;
        return nullptr;
    }

    OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(Inter);
    if(PolyInter != nullptr)
    {
        //SaveGeometrytoShape("A_PolyInter.shp", PolyInter);
        if(PolyInter->get_Area() == GMLPoly->get_Area())
        {
            delete Inter;
			TexUVout->push_back(*TexUV);
            return GMLPoly->clone(); //GMLPoly est inclu dans BuildingShp, il n'y a pas besoin de le modifier
        }
        OGRPolygon* ResPoly = new OGRPolygon;
        OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
        OGRLinearRing* ResExtRing = new OGRLinearRing;

		std::vector<TVec2f> uvPolyInter;

        for(int i = 0; i < InterExtRing->getNumPoints(); ++i)
        {
            OGRPoint* P = new OGRPoint;
            InterExtRing->getPoint(i, P);

            TVec3d M; // <=> P
            M.x = P->getX();
            M.y = P->getY();

            double s, t;

            t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
            s = (M.x - A.x - t * AC.x) / AB.x;

            M.z = A.z + s * AB.z + t * AC.z;

            ResExtRing->addPoint(M.x, M.y, M.z);

			uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquées sans déformation.

            delete P;
        }

        ResPoly->addRingDirectly(ResExtRing);

        /*for(int r = 0; r < PolyInter->getNumInteriorRings(); ++r)
        {
			std::cout << "INTERIOR RING" << std::endl;

            OGRLinearRing* InterIntRing = PolyInter->getInteriorRing(r);
            OGRLinearRing* ResIntRing = new OGRLinearRing;

            for(int i = 0; i < InterIntRing->getNumPoints(); ++i)
            {
                OGRPoint* P = new OGRPoint;
                InterIntRing->getPoint(i, P);

                TVec3d M;
                M.x = P->getX();
                M.y = P->getY();

                double s, t;

                t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
                s = (M.x - A.x - t * AC.x) / AB.x;

                M.z = A.z + s * AB.z + t * AC.z;

                ResIntRing->addPoint(M.x, M.y, M.z);

				uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y));

                delete P;
            }
            ResPoly->addRingDirectly(ResIntRing);
        }*/

		TexUVout->push_back(uvPolyInter);

        return ResPoly;
    }
    else //Si l'intersection ne représente pas un simple Polygon, il faut rechercher si c'est une GeometryCollection qui en contient, afin de pouvoir ensuite les récupérer.
    {
        //std::cout << "Pas Polygon !! " << Inter->getGeometryName() << std::endl;

        OGRGeometryCollection* GC_Inter = dynamic_cast<OGRGeometryCollection*>(Inter);
        if(GC_Inter != nullptr)
        {
            OGRMultiPolygon* ResMultiPoly = new OGRMultiPolygon;
            for(int j = 0; j < GC_Inter->getNumGeometries(); ++j)
            {
                OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(GC_Inter->getGeometryRef(j));
                if(PolyInter != nullptr)
                {
                    OGRPolygon* ResPoly = new OGRPolygon;
                    OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
                    OGRLinearRing* ResExtRing = new OGRLinearRing;

					std::vector<TVec2f> uvPolyInter;

                    for(int i = 0; i < InterExtRing->getNumPoints(); ++i)
                    {
                        OGRPoint* P = new OGRPoint;
                        InterExtRing->getPoint(i, P);

                        TVec3d M; // <=> P
                        M.x = P->getX();
                        M.y = P->getY();

                        double s, t;

                        t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
                        s = (M.x - A.x - t * AC.x) / AB.x;

                        M.z = A.z + s * AB.z + t * AC.z;

                        ResExtRing->addPoint(M.x, M.y, M.z);

						uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquées sans déformation.

                        delete P;
                    }

                    ResPoly->addRingDirectly(ResExtRing);

                    /*for(int r = 0; r < PolyInter->getNumInteriorRings(); ++r)
                    {
						std::cout << "INTERIOR RING2" << std::endl;

                        OGRLinearRing* InterIntRing = PolyInter->getInteriorRing(r);
                        OGRLinearRing* ResIntRing = new OGRLinearRing;

                        for(int i = 0; i < InterIntRing->getNumPoints(); ++i)
                        {
                            OGRPoint* P = new OGRPoint;
                            InterIntRing->getPoint(i, P);

                            TVec3d M;
                            M.x = P->getX();
                            M.y = P->getY();

                            double s, t;

                            t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
                            s = (M.x - A.x - t * AC.x) / AB.x;

                            M.z = A.z + s * AB.z + t * AC.z;

                            ResIntRing->addPoint(M.x, M.y, M.z);

							uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquées sans déformation.

                            delete P;
                        }
                        ResPoly->addRingDirectly(ResIntRing);
                    }*/
                    ResMultiPoly->addGeometryDirectly(ResPoly);

					TexUVout->push_back(uvPolyInter);
                }
            }
            return ResMultiPoly;
        }
        return nullptr;
    }
}

/**
* @brief Calcul les coordonnées de texture UV d'un point par rapport à un polygone qui lui est coplanaire.
* @param Poly Contient les coordonnées des points du polygone
* @param UVs Contient les coordonnées de texture des points du polygone
* @param Point Contient le point dont on désire calculer les coordonnées de texture
*/
TVec2f CalculUV(std::vector<TVec3d>* Poly, std::vector<TVec2f>* UVs, TVec3d Point)
{
	TVec3d A;
    TVec3d B;
    TVec3d C;
	TVec2f uvA;
	TVec2f uvB;
	TVec2f uvC;
	TVec2f uvAB;
	TVec2f uvAC;

    A = Poly->at(0);

	uvA = UVs->at(0);

    TVec3d AB;
    TVec3d AC;

    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
    for(int i = 1; i < Poly->size(); ++i) //Le premier point n'est pas répété à la fin
    {
        if(test == 0)
        {
            B = Poly->at(i);

            if(A != B)
            {
                ++test;// A est bien différent de B
                AB = B - A;
				uvB = UVs->at(i);
				uvAB = uvB - uvA;
            }
        }
        else if(test == 1)
        {
            C = Poly->at(i);

            if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y) || (C.z - A.z)/(B.z - A.z) != (C.y - A.y)/(B.y - A.y) || (C.x - A.x)/(B.x - A.x) != (C.z - A.z)/(B.z - A.z))
            {
                ++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
                AC = C - A;
				uvC = UVs->at(i);
				uvAC = uvC - uvA;
                break;
            }
        }
    }

	double s, t;

	if(AB.x != 0)
	{
		t = (A.z * AB.x - A.x * AB.z + AB.z * Point.x - AB.x * Point.z) / (AB.z * AC.x - AB.x * AC.z);
		s = (Point.x - A.x - t * AC.x) / AB.x;
	}
	else if(AB.y != 0)
	{
		t = (A.z * AB.y - A.y * AB.z + AB.z * Point.y - AB.y * Point.z) / (AB.z * AC.y - AB.y * AC.z);
		s = (Point.y - A.y - t * AC.y) / AB.y;
	}
	else if(AB.z != 0)
	{
		t = (A.x * AB.z - A.z * AB.x + AB.x * Point.z - AB.z * Point.x) / (AB.x * AC.z - AB.z * AC.x);
		s = (Point.z - A.z - t * AC.z) / AB.z;
	}
	else
	{
		t = (A.y * AB.z - A.z * AB.y + AB.y * Point.z - AB.z * Point.y) / (AB.y * AC.z - AB.z * AC.y);
		s = (Point.z - A.z - t * AC.z) / AB.z;
	}

	TVec2f Res = TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y);

	return Res;
}

/**
* @brief Parcourt les polygones des bâtiments d'un fichier CityGML et les assigne, selon leurs positions, à des emprises au sol générées à partir d'un fichier ShapeFile afin de créer un nouveau fichier CityGML.
* @param FootprintsShape Contient les emprises au sol des bâtiments ShapeFile que l'on souhaite extruder en 3D grâce aux polygones CityGML.
* @param LayerShape Contient toutes les données du ShapeFile afin de transférer les éventuelles informations sémantiques dans le CityGML créé.
* @param ModelGML Contient toutes les données du CityGML que l'on va parcourir.
* @param Link Contient les liens entre les bâtiments CityGML et Shape afin de savoir lesquels sont à mettre en relation.
*/
citygml::CityModel* AssignPolygonGMLtoShapeBuildings(std::vector<OGRPolygon*>* FootprintsShape, std::vector<OGRPolygon*>* FootprintsGML, OGRLayer* LayerShape, citygml::CityModel* ModelGML, std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>>* Link, std::vector<TextureCityGML>* TexturesList)
{
    citygml::CityModel* ModelOut = new citygml::CityModel;

    OGRFeature *Feature;
    int cpt = -1;

    const std::string NameModel = ModelGML->getId();
    LayerShape->ResetReading();
    while((Feature = LayerShape->GetNextFeature()) != NULL)
    {
        ++cpt;
		std::cout << cpt << " batiments generes en CityGML sur " << LayerShape->GetFeatureCount() << std::endl;

        std::string Name = NameModel + "_" + std::to_string(cpt);
        citygml::CityObject* BuildingCO = new citygml::Building(Name);
        citygml::CityObject* RoofCO = new citygml::RoofSurface(Name+"_Roof");
        citygml::Geometry* Roof = new citygml::Geometry(Name+"_RoofGeometry", citygml::GT_Roof, 2);
        citygml::CityObject* WallCO = new citygml::WallSurface(Name+"_Wall");
        citygml::Geometry* Wall = new citygml::Geometry(Name+"_WallGeometry", citygml::GT_Wall, 2);

        OGRPolygon* BuildingShp = FootprintsShape->at(cpt);
		OGRPolygon* BuildingShp2 = (OGRPolygon*) BuildingShp->clone(); //Contiendra le polygon Shape mais avec les points intermédiaires ajoutés à partir des Polygons du Wall pour que les intersections fonctionnent.
		OGRMultiPolygon* PolygonsRoofBuildingShp = new OGRMultiPolygon; //Contiendra la liste des Polygons de toit ajouté.

		std::vector<OGRPolygon*> ListPolygonsWall; //On va stocker tous les polygons de Wall qui sont liés à BuildingShp
		
		int cptPolyRoof = 0;
        int cpt2 = - 1;

        for(citygml::CityObject* obj : ModelGML->getCityObjectsRoots())
        {
            ++cpt2;
            bool IsLinked = false;
            for(int i : Link->second.at(cpt))
            {
                if(cpt2 == i)
                {
                    IsLinked = true;
                    break;
                }
            }
            if(!IsLinked) //On regarde si le bâtiment CityGML courant est en relation avec le bâtiment ShapeFile. Si ce n'est pas le cas, on passe au suivant.
            {
                continue;
            }

            if(obj->getType() == citygml::COT_Building)
            {
                for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
                {
                    if(object->getType() == citygml::COT_RoofSurface)
                    {
                        for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
                        {
                            for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                            {
                                OGRLinearRing * OgrRing = new OGRLinearRing;
                                for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
                                    OgrRing->addPoint(Point.x, Point.y, Point.z);

								std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

                                OgrRing->closeRings();

                                if(OgrRing->getNumPoints() > 3)
                                {
                                    OGRPolygon * OgrPoly = new OGRPolygon;
                                    OgrPoly->addRingDirectly(OgrRing);
                                    if(OgrPoly->IsValid() && OgrPoly->Intersects(BuildingShp))
                                    {
										bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

										std::string Url;
										citygml::Texture::WrapMode WrapMode;
										std::vector<std::vector<TVec2f>> TexUVout;
										if(HasTexture)
										{
											Url = PolygonCityGML->getTexture()->getUrl();
											WrapMode = PolygonCityGML->getTexture()->getWrapMode();
										}

                                        OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, BuildingShp, &TexUV, &TexUVout);

                                        if(CutPoly != nullptr)
                                        {
                                            if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
                                            {
												citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "Roof_" + std::to_string(cptPolyRoof));
                                                Roof->addPolygon(GMLPoly);
												PolygonsRoofBuildingShp->addGeometry(CutPoly);
												if(HasTexture)
												{
													TextureCityGML Texture;
													Texture.Id = Name + "Roof_" + std::to_string(cptPolyRoof) + "_Poly";
													Texture.IdRing = Name + "Roof_" + std::to_string(cptPolyRoof) + "_Ring";
													Texture.Wrap = WrapMode;
													Texture.Url = Url;
													Texture.TexUV = TexUVout.at(0);
													TexturesList->push_back(Texture);
												}
												++cptPolyRoof;
                                            }
                                            else
                                            {
                                                OGRMultiPolygon* CutMultiPoly = dynamic_cast<OGRMultiPolygon*>(CutPoly);
                                                if(CutMultiPoly != nullptr)
                                                {
                                                    for(int i = 0; i < CutMultiPoly->getNumGeometries(); ++i)
                                                    {
														citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutMultiPoly->getGeometryRef(i), Name + "Roof_" + std::to_string(cptPolyRoof));
                                                        Roof->addPolygon(GMLPoly);
														PolygonsRoofBuildingShp->addGeometry(CutMultiPoly->getGeometryRef(i));
														if(HasTexture)
														{
															TextureCityGML Texture;
															Texture.Id = Name + "Roof_" + std::to_string(cptPolyRoof) + "_Poly";
															Texture.IdRing = Name + "Roof_" + std::to_string(cptPolyRoof) + "_Ring";
															Texture.Wrap = WrapMode;
															Texture.Url = Url;
															Texture.TexUV = TexUVout.at(i);
															TexturesList->push_back(Texture);
														}
														++cptPolyRoof;
                                                    }
                                                }
                                            }
                                        }
                                        delete CutPoly;
                                    }
                                    delete OgrPoly;
                                }
                                else
                                    delete OgrRing;
                            }
                        }
                    }
				}
			}
		}

		cpt2 = - 1;
		int cptPolyWall = 0;
		double Zmin = -1; //Stock la valeur minimale en Z des murs afin d'avoir une valeur à appliquer pour les murs créés à partir de rien.

        for(citygml::CityObject* obj : ModelGML->getCityObjectsRoots())
        {
			
            ++cpt2;
            bool IsLinked = false;
            for(int i : Link->second.at(cpt))
            {
                if(cpt2 == i)
                {
                    IsLinked = true;
                    break;
                }
            }
            if(!IsLinked) //On regarde si le bâtiment CityGML courant est en relation avec le bâtiment ShapeFile. Si ce n'est pas le cas, on passe au suivant.
            {
                continue;
            }

            if(obj->getType() == citygml::COT_Building)
            {
                for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
                {
					if(object->getType() == citygml::COT_WallSurface)
                    {
						for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
                        {
                            for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                            {
								OGRLinearRing * WallRing = new OGRLinearRing;

								std::vector<TVec3d> PointsWall = PolygonCityGML->getExteriorRing()->getVertices();
                                for(int i = 0; i < PointsWall.size(); ++i) //Le premier point n'est pas répété à la fin
								{
									TVec3d Point = PointsWall.at(i);
                                    WallRing->addPoint(Point.x, Point.y, Point.z);
									if(Zmin == -1 || Zmin > Point.z)
										Zmin = Point.z;
								}
								WallRing->closeRings();

								bool HasTexture = (PolygonCityGML->getTexture() != nullptr);
								
								std::string Url;
								citygml::Texture::WrapMode WrapMode;
								std::vector<TVec2f> TexUV;
								
								if(HasTexture)
								{
									 Url = PolygonCityGML->getTexture()->getUrl();
									 WrapMode = PolygonCityGML->getTexture()->getWrapMode();
									 TexUV = PolygonCityGML->getTexCoords();
								}

                                if(WallRing->getNumPoints() > 3)
                                {
									OGRPolygon * WallPoly = new OGRPolygon;
                                    WallPoly->addRingDirectly(WallRing);
									if(WallPoly->Distance(BuildingShp) < Precision_Vect) //Le polygon du Wall semble intersecter le Roof du Shape, on va donc l'ajouter à ce bâtiment.
									{
										//Wall->addPolygon(ConvertOGRPolytoGMLPoly(WallPoly, Name)); // Pour simplement ajouter les Polygons de Mur qui touchent un Polygon Shp -> Il y aura des doublons
										//continue;

										OGRPolygon* tmp = BuildingShp2;
										BuildingShp2 = (OGRPolygon*) CreatePointsOnLine(WallPoly, BuildingShp2); //On crée les points du mur sur l'enveloppe du bâtiment pour la génération des murs non présents dans le CityGML.
										delete tmp;

										for(int i = 0; i < PolygonsRoofBuildingShp->getNumGeometries(); ++i)
										{
											OGRGeometry* PolyRoof = PolygonsRoofBuildingShp->getGeometryRef(i);

											std::vector<TVec2f> TexUVWall;

											if(PolyRoof->Distance(WallPoly) < Precision_Vect) //On recherche quels sont les polygons du Roof qui intersectent le polygon du Wall afin de créer les points de l'un sur l'autre.
											{
												OGRGeometry* PolyRoofwithWallPoints = CreatePointsOnLine(WallPoly, PolyRoof);
												OGRGeometry* tmp = CreatePointsOnLine(PolyRoof, WallPoly);

												delete WallPoly;
												WallPoly = (OGRPolygon*) tmp;

												OGRLinearRing* ExtRingWall = WallPoly->getExteriorRing();
												OGRPolygon* WallPolyRes = new OGRPolygon; //Contiendra le polygon du Wall que l'on aura limité à la zone 2D formée par le polygon BuildingShp
												OGRLinearRing* WallRingRes = new OGRLinearRing;

												for(int i = 0; i < ExtRingWall->getNumPoints() - 1; ++i)
												{
													OGRPoint* WallPoint1 = new OGRPoint(ExtRingWall->getX(i), ExtRingWall->getY(i), ExtRingWall->getZ(i));
													OGRPoint* WallPoint2 = new OGRPoint(ExtRingWall->getX(i+1), ExtRingWall->getY(i+1), ExtRingWall->getZ(i+1));

													if(WallPoint1->Distance(PolyRoofwithWallPoints) == 0 && WallPoint2->Distance(PolyRoofwithWallPoints) == 0)
													{
														WallRingRes->addPoint(WallPoint1);
														WallRingRes->addPoint(WallPoint2);
													}
													delete WallPoint1;
													delete WallPoint2;
												}

												if(WallRingRes != nullptr && WallRingRes->getNumPoints() > 0)
												{
													WallRingRes->closeRings();
													OGRLinearRing* CleanWallRingRes = new OGRLinearRing; //Des points dans WallRingRes sont potentiellement en double, il faut les retirer avant de créer le Polygon, on va mettre le LinearRing modifié ici.
													double prevX = -1, prevY = -1, prevZ = -1;
													for(int i = 0; i < WallRingRes->getNumPoints(); ++i)
													{
														if(WallRingRes->getX(i) != prevX || WallRingRes->getY(i) != prevY || WallRingRes->getZ(i) != prevZ)
														{
															prevX = WallRingRes->getX(i);
															prevY = WallRingRes->getY(i);
															prevZ = WallRingRes->getZ(i);
															if(HasTexture)
																TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(prevX, prevY, prevZ)));
															
															CleanWallRingRes->addPoint(prevX, prevY, prevZ);
														}
													}
													
													delete WallRingRes;
													if(CleanWallRingRes->getNumPoints() < 4)
													{
														delete CleanWallRingRes;
														continue;
													}
													WallPolyRes->addRingDirectly(CleanWallRingRes);
													citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)WallPolyRes, Name + "Wall_" + std::to_string(cptPolyWall));
													
													Wall->addPolygon(GMLPoly);
													delete WallPolyRes;
													if(HasTexture)
													{
														TextureCityGML Texture;
														Texture.Id = Name + "Wall_" + std::to_string(cptPolyWall) + "_Poly";
														Texture.IdRing = Name + "Wall_" + std::to_string(cptPolyWall) + "_Ring";
														Texture.Wrap = WrapMode;
														Texture.Url = Url;
														Texture.TexUV = TexUVWall;
														TexturesList->push_back(Texture);
													}
													++cptPolyWall;
												}
											}
										}
										//On convertit les polygones de WallPoly (qui sont en 2D) en 3D grâce à PolygonsRoofBuildingShpwithWallPoints

										ListPolygonsWall.push_back(WallPoly);
									}
									else
										delete WallPoly;
								}
								else
									delete WallRing;
							}
						}
					}
                }
            }
        }

		//On veut générer des murs sur chaque facade du bâtiment, car les murs du CityGML ne couvrent pas tous les cas.
		//Il faut commencer par stocker toutes les LineString de BuildingShp2 (provenant d'exterior Ring et d'interior Rings) car ces lignes doivent être la base de murs, qu'ils existent déjà dans le CityGML ou qu'il faille les créer.
		std::vector<OGRLineString*> ListLinesShp;
		OGRLinearRing* ExtRingShp = BuildingShp2->getExteriorRing();
		for(int i = 0; i < ExtRingShp->getNumPoints() - 1; ++i)
		{
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(ExtRingShp->getX(i), ExtRingShp->getY(i), ExtRingShp->getZ(i));
			Line->addPoint(ExtRingShp->getX(i+1), ExtRingShp->getY(i+1), ExtRingShp->getZ(i+1));
			ListLinesShp.push_back(Line);
		}
		
		for(int j = 0; j < BuildingShp2->getNumInteriorRings(); ++j)
		{
			OGRLinearRing* IntRingShp = BuildingShp2->getInteriorRing(j);
			for(int i = 0; i < IntRingShp->getNumPoints() - 1; ++i)
			{
				OGRLineString* Line = new OGRLineString;
				Line->addPoint(IntRingShp->getX(i), IntRingShp->getY(i), IntRingShp->getZ(i));
				Line->addPoint(IntRingShp->getX(i+1), IntRingShp->getY(i+1), IntRingShp->getZ(i+1));
				ListLinesShp.push_back(Line);
			}
		}

		//SaveGeometrytoShape("A_BuildingShp2.shp", BuildingShp2);
		//SaveGeometrytoShape("A_PolygonsRoofBuildingShp.shp", PolygonsRoofBuildingShp);

		//On va parcourir toutes les LineString composant les polygones du toit du Building Shp. Pour chacune de ces arêtes, on va regarder si elle est sur sa frontière, et si elle n'a pas déjà de mur CityGML qui lui est lié. 
		//Si ce n'est pas le cas, on va construire un mur générique afin de construire des objets fermés.
		for(int i = 0; i < PolygonsRoofBuildingShp->getNumGeometries(); ++i)
		{
			OGRPolygon* Poly = (OGRPolygon*) PolygonsRoofBuildingShp->getGeometryRef(i);

			OGRLinearRing* Ring = Poly->getExteriorRing();

			for(int j = 0; j < Ring->getNumPoints() - 1; ++j)
			{
				OGRPoint* Point1R = new OGRPoint;
				OGRPoint* Point2R = new OGRPoint;
				OGRLineString* Line = new OGRLineString;

				Ring->getPoint(j, Point1R);
				Ring->getPoint(j+1, Point2R);
				Line->addPoint(Point1R);
				Line->addPoint(Point2R);
				bool WallIsAlreadyBuilt = false;

				for(int p = 0; p < ListPolygonsWall.size(); ++p)
				{
					OGRPolygon* WallPoly = ListPolygonsWall.at(p);

					if(Line->Distance(WallPoly) > Precision_Vect)
						continue;

					if(Point1R->Distance(WallPoly) < Precision_Vect && Point2R->Distance(WallPoly) < Precision_Vect)
					{
						WallIsAlreadyBuilt = true;
						break;
					}
				}
				if(WallIsAlreadyBuilt) //Cette arête est déjà reliée à un mur CityGML
				{
					delete Point1R;
					delete Point2R;
					delete Line;
					continue;
				}
				
				bool LineIsOnEnvelope = false;
				for(int l = 0; l < ListLinesShp.size(); ++l)
				{
					OGRLineString* ShpLine = ListLinesShp.at(l);
					if(Point1R->Distance(ShpLine) < Precision_Vect && Point2R->Distance(ShpLine) < Precision_Vect)
					{
						LineIsOnEnvelope = true;
						break;
					}
				}
				if(!LineIsOnEnvelope) //Cette arête n'est pas sur l'envelope du bâtiment Shp
				{
					delete Point1R;
					delete Point2R;
					delete Line;
					continue;
				}
				OGRLinearRing* WallLine = new OGRLinearRing;
				WallLine->addPoint(Point1R);
				WallLine->addPoint(Point1R->getX(), Point1R->getY(), Zmin);
				WallLine->addPoint(Point2R->getX(), Point2R->getY(), Zmin);
				WallLine->addPoint(Point2R);
				WallLine->addPoint(Point1R);
				OGRPolygon* NewWallPoly = new OGRPolygon;
				NewWallPoly->addRingDirectly(WallLine);
				Wall->addPolygon(ConvertOGRPolytoGMLPoly(NewWallPoly, Name));
				delete NewWallPoly;
				delete Point1R;
				delete Point2R;
				delete Line;
			}
		}

		delete PolygonsRoofBuildingShp;
		delete BuildingShp2;

		for(OGRLineString* Line:ListLinesShp)
			delete Line;

		for(OGRPolygon* Poly:ListPolygonsWall)
			delete Poly;

		RoofCO->addGeometry(Roof);
        ModelOut->addCityObject(RoofCO);
        BuildingCO->insertNode(RoofCO);
        WallCO->addGeometry(Wall);
        ModelOut->addCityObject(WallCO);
        BuildingCO->insertNode(WallCO);

        ModelOut->addCityObject(BuildingCO);
        ModelOut->addCityObjectAsRoot(BuildingCO);
    }

    return ModelOut;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Découpe les bâtiments du fichier CityGML ouvert dans Tile, à partie des bâtiments cadastraux contenus dans le fichier shapefile chargé dans DataSource.
* @param Tile Contient les données du fichier CityGML ouvert : il doit contenir un ensemble de bâtiments LOD2
* @param DataSource Contient les données du fichier Shapefile ouvert : il doit contenir un ensemble d'emprises au sol définissant les bâtiments de la zone étudiée
*/
citygml::CityModel* CutCityGMLwithShapefile(vcity::Tile* Tile, OGRDataSource* DataSource, std::vector<TextureCityGML>* TexturesList)
{
    citygml::CityModel* Model = Tile->getCityModel();
	OGRLayer* Layer = DataSource->GetLayer(0);

	//Créer 2 tableaux de polygons correspondant aux emprises au sol des bâtiments du CityGML et à celles du cadastre. Conserver un lien vers le CityModel et le Layer.
	std::cout << "Generation des emprises au sol du CityGML." << std::endl;
	std::vector<OGRPolygon*> FootprintsCityGML = GetFootPrintsfromCityGML(Model); //On part du principe que chaque bâtiment CityGML correspond à une emprise au sol Polygon (et non MultiPolygon !). Passage dans SplitBuildingsFromCityGML obligatoire !
	/*OGRMultiPolygon* FootPrintsBuildingsCityGML = new OGRMultiPolygon;
	for(OGRPolygon* Poly:FootprintsCityGML)
		FootPrintsBuildingsCityGML->addGeometry(Poly);
	SaveGeometrytoShape("FootPrintsBuildingsCityGML.shp", FootPrintsBuildingsCityGML);*/
	std::cout << "Generation des emprises au sol du ShapeFile." << std::endl;
	std::vector<OGRPolygon*> FootprintsShapefile = GetFootPrintsfromShapeFile(Layer); //On part du principe que chaque featur du shapefile contient un unique polygon en geometry
	//Lier Bâtiments CityGML <-> Bâtiments Cadastraux qui s'intersectent pour orienter les étapes suivantes.
	std::cout << "Liaison des emprises au sol du CityGML avec celles du ShapeFile." << std::endl;
	std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> Link = LinkCityGMLandShapefilesFootprints(FootprintsCityGML, FootprintsShapefile);
	//Modifier chaque emprise au sol du Shapefile afin que ses arrêtes correspondent à celles des bâtiments du CityGML
	std::cout << "Modification des polygones du cadastre." << std::endl;
	std::vector<OGRPolygon*> NewFootprintsShapefile = FusionEnvelopes(&FootprintsCityGML, &FootprintsShapefile, &Link);
	for(OGRPolygon* Poly:FootprintsShapefile)
		delete Poly;
    //On a maintenant un ensemble d'emprises au sol (uniquement des Polygons, ce sont juste des zones d'influences non intersectées avec le CityGML, donc pas de MultiPolygon) contenues dans le vecteur
    //NewFootprintsShapefile. Chaque polygon correspond à un bâtiment cadastral et son emprise au sol recouvre une partie de l'emprise au sol d'un bâtiment CityGML lié. L'ensemble des emprises au sol
    //de NewFootprintsShapefile doit recouvrir les emprises au sol de tous les bâtiments CityGML. Il nous reste à parcourir les polygons du CityGML et à les assigner à ces différents bâtiments Shape.
	std::cout << "Decoupe des Polygons Wall et Roof du CityGML selon les emprises au sol generees a partir du cadastre." << std::endl;
    citygml::CityModel* ModelOut = AssignPolygonGMLtoShapeBuildings(&NewFootprintsShapefile, &FootprintsCityGML, Layer, Model, &Link, TexturesList);

	for(OGRPolygon* Poly:FootprintsCityGML)
		delete Poly;
	for(OGRPolygon* Poly:NewFootprintsShapefile)
		delete Poly;

	return ModelOut;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Parcourt tous les bâtiments du model, récupère leurs polygones de toit, et les unis afin d'obtenir une liste de polygones disjoints
* @param model : Contient les bâtiments que l'on souhaite traiter
* @param BuildingsFootprints : résultat
* @param Names : contient la liste de noms associés à la liste de Polygons/Bâtiments contenus dans BuildingsFootprints
*/
OGRMultiPolygon* GetBuildingsFootprintsFromCityModel(citygml::CityModel* model, std::vector<std::string>* Names)
{
	OGRMultiPolygon* BuildingsFootprints = new OGRMultiPolygon;
	int cpt = 0;

	std::vector<OGRMultiPolygon *> ListEnveloppes;
	std::vector<std::string> NameBuildings;
	
    for(citygml::CityObject* obj : model->getCityObjectsRoots())
    {
        if(obj->getType() == citygml::COT_Building)
        {
            OGRMultiPolygon* Building = new OGRMultiPolygon;//Version OGR du bâtiment qui va être remplie

            for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
            {
                if(object->getType() == citygml::COT_RoofSurface)
                {
                    for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
                    {
                        for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                        {
                            OGRLinearRing * OgrRing = new OGRLinearRing;

                            for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								OgrRing->addPoint(Point.x, Point.y, Point.z);

                            OgrRing->closeRings();

                            if(OgrRing->getNumPoints() > 3)
                            {
								OGRPolygon * OgrPoly = new OGRPolygon;
                                OgrPoly->addRingDirectly(OgrRing);
                                if(OgrPoly->IsValid())
                                {
                                    Building->addGeometry(OgrPoly);
                                }
                            }
                            else
                                delete OgrRing;
                        }
                    }
                }
            }

			if(Building->IsEmpty())
			{
				cpt++;
				std::cout << "Avancement etape 1 : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
				continue;
			}

            OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);
			ListEnveloppes.push_back(Enveloppe);
			NameBuildings.push_back(obj->getId());

            if(BuildingsFootprints == nullptr)
                BuildingsFootprints = Enveloppe;
            else
            {
                OGRMultiPolygon * tmp = BuildingsFootprints;
                BuildingsFootprints = (OGRMultiPolygon *)tmp->Union(Enveloppe);
                delete tmp;
            }
        }

        cpt++;
        std::cout << "Avancement etape 1 : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
    }
	std::cout << std::endl;
	
	Names->resize(BuildingsFootprints->getNumGeometries(), "null");

	//On veut assigner le nom de bâtiment initial à chaque sous bâtiment que l'on vient de créer, avec une numérotation permettant de les distinguer les uns des autres
	for(int i = 0; i < ListEnveloppes.size(); ++i)
	{
		cpt = 0;
		OGRMultiPolygon * CurrentMP = ListEnveloppes.at(i);
		for(int j = 0; j < BuildingsFootprints->getNumGeometries(); ++j)
		{
			if(Names->at(j) != "null")//Le nom est déjà rempli, inutile de faire les tests suivants
				continue;

			OGRPolygon* CurrentGeo = (OGRPolygon*) BuildingsFootprints->getGeometryRef(j);
            if(CurrentMP->Intersects(CurrentGeo))
			{
				++ cpt;
				Names->at(j) = NameBuildings.at(i) + "_" + std::to_string(cpt);
			}
		}
	}

	return BuildingsFootprints;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Traite un fichier CityGML afin de définir chaque objet 3D isolé comme étant un bâtiment : ressort le CityModel contenant tous ces nouveaux bâtiments
* @param Tile : Contient les données du CityGML ouvert
*/
//TODO : GESTION TEXTURES
citygml::CityModel* SplitBuildingsFromCityGML(vcity::Tile* Tile)
{
	citygml::CityModel* model = Tile->getCityModel();
	citygml::CityModel* ModelOut = new citygml::CityModel;

	std::vector<std::string> Names;

	OGRMultiPolygon * BuildingsFootprints = GetBuildingsFootprintsFromCityModel(model, &Names); //Permet de définir l'existence des bâtiments recherchés, il faut maintenant leur assigner des CityObject complets (Wall + Roof + ...)
	 //Maintenant qu'on a la liste des bâtiments désirés, il va falloir assigner à chacun les Wall et Roof qui correspondent en reparcourant
	// tous les bâtiments et en regardant quels sont les polygons se situant au même niveau que les emprises au sol sauvegardées dans BuildingsFootprints

	for(int i = 0; i < BuildingsFootprints->getNumGeometries(); ++i)
	{
		OGRGeometry * Building = BuildingsFootprints->getGeometryRef(i); //Batiment de référence que l'on cherche à remplir d'objets CityGML

		std::string Name = Names.at(i);
		citygml::CityObject* BuildingCO = new citygml::Building(Name);
		citygml::CityObject* RoofCO = new citygml::RoofSurface(Name+"_Roof");
		citygml::Geometry* Roof = new citygml::Geometry(Name+"_RoofGeometry", citygml::GT_Roof, 2);
		citygml::CityObject* WallCO = new citygml::WallSurface(Name+"_Wall");
		citygml::Geometry* Wall = new citygml::Geometry(Name+"_WallGeometry", citygml::GT_Wall, 2);

		for(citygml::CityObject* obj : model->getCityObjectsRoots())
		{
			if(obj->getType() == citygml::COT_Building)
			{
				for(citygml::CityObject* object : obj->getChildren())
				{
					if(object->getType() == citygml::COT_RoofSurface)
					{
						for(citygml::Geometry* Geometry : object->getGeometries())
						{
							for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
							{
								bool PolyIsInBati = true;

								for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								{
									OGRPoint* P = new OGRPoint(Point.x, Point.y);
									
                                    if(!P->Intersects(Building)) //Si un point ne se retrouve pas dans Building, alors le polygon correspondant ne doit pas lui être associé.
									{
										PolyIsInBati = false;
										break;
									}
								}
								if(PolyIsInBati)
                                    Roof->addPolygon(PolygonCityGML);
							}
						}
					}
					else if(object->getType() == citygml::COT_WallSurface)
					{
						for(citygml::Geometry* Geometry : object->getGeometries())
						{
							for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
							{
								bool PolyIsInBati = true;
								for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								{
									OGRPoint* P = new OGRPoint(Point.x, Point.y);
									if(P->Distance(Building)> Precision_Vect) //Si un point ne se retrouve pas dans Building, alors le polygon correspondant ne doit pas lui être associé. Distance au lieu de intersection à cause des imprécisions 
										//entre certains points de Wall par rapport à l'emprise au sol définie par les Roof
									{
										PolyIsInBati = false;
										delete P;
										break;
									}
									delete P;
								}
								if(PolyIsInBati) //Si tous les points du polygone sont dans Building, il faut l'ajouter à celui ci
								{
									Wall->addPolygon(PolygonCityGML);
								}
							}
						}
					}
				}
			}
		}
		RoofCO->addGeometry(Roof);
        ModelOut->addCityObject(RoofCO);
        BuildingCO->insertNode(RoofCO);
		WallCO->addGeometry(Wall);
        ModelOut->addCityObject(WallCO);
        BuildingCO->insertNode(WallCO);

		ModelOut->addCityObject(BuildingCO);
        ModelOut->addCityObjectAsRoot(BuildingCO);

		std::cout << "Avancement etape 2 : " << i+1 << "/" << BuildingsFootprints->getNumGeometries() << " batiments crees.\r" << std::flush;
	}
	std::cout << std::endl;
	
	return ModelOut;
}
