#include "ChangeDetection.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
#include "ExportToShape.hpp"
#include "lodsmanagement.hpp"
////////////////////////////////////////////////////////////////////////////////
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
* @param Geo1P Premier ensemble de geometries non unies : pour un polygone de Geo1, il donne la liste des polygones non unis qui le composent
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
					delete Bati2;
                    break;
                }
                else//Batiment modifie en "hauteur"
                {
                    Res.first[i].push_back(-2);
                    Res.second[j].push_back(-2);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
					delete Bati2;
                    break;
                }
            }
            if(val1 < 0.5 && val2 < 0.5)//Le bâtiment a été modifié car les emprises au sol restent suffisament proches
            {
                Res.first[i].push_back(-2);
                Res.second[j].push_back(-2);
                Res.first[i].push_back(j);
                Res.second[j].push_back(i);
				delete Bati2;
                break;
            }
			//// Si on arrive jusqu'ici, les premiers tests disent que les deux bâtiments sont respectivement détruit/construit. Dernier test pour extraire les bâtiments qui ont des parties identiques => bâtiment modifié
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
* @brief Compare deux fichiers CityGML d'une même zone obtenus à deux dates différentes en donnant les modifications entre leurs bâtiments
*/
void CompareTiles(std::string Folder, citygml::CityModel* City1, citygml::CityModel* City2)//Version GDAL
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

		OGRMultiPolygon * ModelPolygons = new OGRMultiPolygon;//Contient tous les polygones composant le model courant

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
										ModelPolygons->addGeometryDirectly(OgrPoly);
                                    }
                                }
                                else
                                    delete OgrRing;
                            }
                        }
                    }
                }
				
                OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);

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

			std::cout << "G = " << g << "/" << EnveloppeCityU[i]->getNumGeometries() << std::endl;
			for(size_t j = 0; j < ModelPolygons->getNumGeometries(); ++j)//Pour le bâtiment courant, on va chercher dans ModelPolygons quels sont les polygones qui le composent.
            {
				if(ModelPolygons->getGeometryRef(j)->Intersect(CurrBati))//Ce polygone appartient bien à CurrBati
				{
					OGRGeometry * Inter = ModelPolygons->getGeometryRef(j)->Intersection(CurrBati);
					if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D || Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D) //L'intersection n'est pas un simple point
					{
						Bati->addGeometry(ModelPolygons->getGeometryRef(j));
						ModelPolygons->removeGeometry(j);//On le retire de ModelPolygons car il est maintenant associé à CurrBati
						j--;
					}
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

    SaveGeometrytoShape(Folder + "/Bati.shp", EnveloppeCityU[1]);
    SaveGeometrytoShape(Folder + "/BatiCrees.shp", BatiCrees);
    SaveGeometrytoShape(Folder + "/BatiDetruits.shp", BatiDetruits);
    SaveGeometrytoShape(Folder + "/BatiModifies.shp", BatiModifies2);
    SaveGeometrytoShape(Folder + "/BatiInchanges.shp", BatiInchanges);

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
