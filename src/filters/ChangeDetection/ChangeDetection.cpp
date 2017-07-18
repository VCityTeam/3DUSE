// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "ChangeDetection.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtools.hpp"

/**
* @brief Compare deux ensembles de geometries en retournant les liens entre leurs polygones et l'information sur ces liens : si un polygone se retrouve de maniere identique dans les deux ensembles de geometries, dans un seul ou s'il a ete modifie
* @param Geo1 Premier ensemble de geometries qui ont ete unies : deux triangles voisins sont reunis en un rectangle par exemple
* @param Geo2 Second ensemble de geometries qui ont ete unies
* @param Geo1P Premier ensemble de geometries non unies : pour un polygone de Geo1, il donne la liste des polygones non unis qui le composent
* @param Geo2P Second ensemble de geometries non unies
*/
std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > >
  CompareBati( std::string Folder,
               OGRMultiPolygon * Geo1,
               OGRMultiPolygon * Geo2,
               std::vector<OGRMultiPolygon* > Geo1P,
               std::vector<OGRMultiPolygon *> Geo2P )
{
    std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Res; //Enregistre les liens entre les polygones. Pour un polygone donnee de Geo1, si il est en lien avec un de Geo2, l'indice sera precede de -1 ou -2 pour inchange/change

    int NbGeo1 = Geo1->getNumGeometries(); //Nb de batiments de la date1
    int NbGeo2 = Geo2->getNumGeometries(); //Nb de batiments de la date2

    Res.first.resize(NbGeo1);
    Res.second.resize(NbGeo2);

    //OGRMultiPolygon* PolyZonesCommunes = new OGRMultiPolygon;

    for (int i = 0; i < NbGeo1; ++i)
    {
        OGRPolygon * Bati1 = (OGRPolygon *)Geo1->getGeometryRef(i)->clone();

        for (int j = 0; j < NbGeo2; ++j)
        {
            OGRPolygon * Bati2 = (OGRPolygon *)Geo2->getGeometryRef(j)->clone();

            if (!Bati1->Intersects(Bati2))
                continue;

            double Area = 0;

            OGRGeometry* Intersection = Bati1->Intersection(Bati2);

            OGRwkbGeometryType Type = Intersection->getGeometryType();

            if (Type == OGRwkbGeometryType::wkbPolygon || Type == OGRwkbGeometryType::wkbPolygon25D)
            {
                OGRPolygon * tmp = (OGRPolygon *)(Intersection);
                Area = tmp->get_Area();
                delete tmp;
            }
            else if (Type == OGRwkbGeometryType::wkbMultiPolygon || Type == OGRwkbGeometryType::wkbMultiPolygon25D)
            {
                OGRMultiPolygon * tmp = (OGRMultiPolygon *)(Intersection);
                Area = tmp->get_Area();
                delete tmp;
            }
            else if (Type == OGRwkbGeometryType::wkbGeometryCollection || Type == OGRwkbGeometryType::wkbGeometryCollection25D)
            {
                OGRGeometryCollection * tmp = (OGRGeometryCollection *)(Intersection);
                Area = tmp->get_Area();
                delete tmp;
            }

            double val1 = (Bati1->get_Area() - Area) / Area;
            double val2 = (Bati2->get_Area() - Area) / Area;

            if (val1 < 0.01 && val2 < 0.01 && Bati1->get_Area() - Area < 5 && Bati2->get_Area() - Area < 5)//Les polygons sont identiques
            {
                if (DistanceHausdorff(Geo1P.at(i), Geo2P.at(j)) < 1)//Si la distance de Hausdorff entre les deux batiments est inferieure a 5m.
                {
                    Res.first[i].push_back(-1);
                    Res.second[j].push_back(-1);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
                    delete Bati2;
                    continue;
                }
                else//Batiment modifie en "hauteur"
                {
                    Res.first[i].push_back(-2);
                    Res.second[j].push_back(-2);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
                    delete Bati2;
                    continue;
                }
            }

            //// Si on arrive jusqu'ici, les premiers tests disent que les deux batiments sont respectivement detruit/construit. Dernier test pour extraire les batiments qui ont des parties identiques => batiment modifie
            OGRMultiPolygon* ZonesCommunes = new OGRMultiPolygon;

            for (int u = 0; u < Geo1P.at(i)->getNumGeometries(); ++u)
            {
                OGRPolygon* Poly1 = (OGRPolygon*)Geo1P.at(i)->getGeometryRef(u);
                if (Poly1 == NULL)
                    continue;
                for (int v = 0; v < Geo2P.at(j)->getNumGeometries(); ++v)
                {
                    OGRPolygon* Poly2 = (OGRPolygon*)Geo2P.at(j)->getGeometryRef(v);
                    if (Poly2 == NULL)
                        continue;
                    if (!Poly1->Intersects(Poly2))
                        continue;

                    OGRGeometry* Inter = Poly1->Intersection(Poly2);
                    if (Inter->getGeometryType() != wkbPolygon && Inter->getGeometryType() != wkbPolygon25D && Inter->getGeometryType() != wkbMultiPolygon && Inter->getGeometryType() != wkbMultiPolygon25D)//Il faut que l'intersection existe et soit au moins un polygon
                    {
                        delete Inter;
                        continue;
                    }

                    OGRMultiPolygon* InterMP = new OGRMultiPolygon; //Intersection entre les deux polygones courants sous la forme d'un multipolygon meme si il n'y qu'un seul polygone pour eviter de traiter les deux cas par la suite

                    if (Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D)
                        InterMP->addGeometry(Inter);
                    else if (Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D)
                        InterMP = (OGRMultiPolygon*)Inter;

                    ////// TODO : CES POLYGONES SONT A TRIANGULER POUR AVOIR UNE DISTANCE DE HAUSDORFF VIABLE !!!! //////

                    OGRMultiPolygon* InterOnPoly1 = ProjectPolyOn3DPlane(InterMP, Poly1);
                    OGRMultiPolygon* InterOnPoly2 = ProjectPolyOn3DPlane(InterMP, Poly2);

                    OGRMultiPolygon* MP1 = new OGRMultiPolygon;
                    MP1->addGeometry(Poly1);
                    OGRMultiPolygon* MP2 = new OGRMultiPolygon;
                    MP2->addGeometry(Poly2);

                    double d1 = Hausdorff(InterOnPoly1, MP2);
                    double d2 = Hausdorff(InterOnPoly2, MP1);

                    if (d1 < 0.5 && d2 < 0.5)
                    {
                        for (int k = 0; k < InterMP->getNumGeometries(); ++k)
                        {
                            OGRGeometry* InterGeo = InterMP->getGeometryRef(k);

                            OGRGeometry* tmp = ZonesCommunes->Union(InterGeo);

                            if (!tmp->IsValid())
                            {
                                //std::cout << "UNION NON VALIDE " << std::endl;
                                continue;
                            }

                            if (tmp->getGeometryType() == wkbPolygon || tmp->getGeometryType() == wkbPolygon25D)
                            {
                                delete ZonesCommunes;
                                ZonesCommunes = new OGRMultiPolygon;
                                ZonesCommunes->addGeometry((OGRPolygon*)tmp);
                            }
                            else if (tmp->getGeometryType() == wkbMultiPolygon || tmp->getGeometryType() == wkbMultiPolygon25D)
                            {
                                delete ZonesCommunes;
                                ZonesCommunes = (OGRMultiPolygon*)tmp;
                            }
                        }
                    }
                    delete MP1;
                    delete MP2;
                    delete InterMP;
                }
            }

            if (ZonesCommunes->IsEmpty() || !ZonesCommunes->IsValid()) //Si il n'y a pas d'intersection, les lignes suivantes bug donc il faut sortir
                continue;

            if (ZonesCommunes->getGeometryType() == wkbPolygon || ZonesCommunes->getGeometryType() == wkbPolygon25D)
            {
                OGRPolygon* Poly = (OGRPolygon*)ZonesCommunes;
                if (Poly->get_Area() > 10) //La zone commune est un polygone d'aire superieure a 10m² -> Batiment modifie car une zone est restee identique
                {
                    Res.first[i].push_back(-2);
                    Res.second[j].push_back(-2);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
                    //PolyZonesCommunes->addGeometry(Poly);
                }
            }
            else if (ZonesCommunes->getGeometryType() == wkbMultiPolygon || ZonesCommunes->getGeometryType() == wkbMultiPolygon25D)
            {
                OGRMultiPolygon* MultiPoly = (OGRMultiPolygon*)ZonesCommunes;
                bool Modified = false; //Passe a true des qu'un polygon a une aire superieure au seuil pour seulement remplir PolyZonesCommunes
                for (int k = 0; k < MultiPoly->getNumGeometries(); ++k)
                {
                    OGRPolygon* Poly = (OGRPolygon*)MultiPoly->getGeometryRef(k);
                    if (Poly->get_Area() > 10) //La zone commune comporte au moins un polygone d'aire superieure a 10m² -> Batiment modifie car une zone est restee identique
                    {
                        //if(!Modified) //Seulement pour avoir un PolyZonesCommunes complet, inutile pour l'algo : a retirer ce test et remettre break pour gain de temps
                        //{
                        Res.first[i].push_back(-2);
                        Res.second[j].push_back(-2);
                        Res.first[i].push_back(j);
                        Res.second[j].push_back(i);
                        Modified = true;
                        //}
                        //PolyZonesCommunes->addGeometry(Poly);
                        break;
                    }
                }
            }
            delete ZonesCommunes;
            delete Bati2;
        }
        delete Bati1;
        std::cout << "Avancement de CompareGeos : " << i + 1 << " / " << NbGeo1 << "\r" << std::flush;
    }
    std::cout << "\n";

    //SaveGeometrytoShape(Folder + "/ZonesCommunes.shp", PolyZonesCommunes);

    return Res;
}
////////////////////////////////////////////////////////////////////////////////
/**
* @brief Compare deux fichiers CityGML d'une meme zone obtenus a deux dates differentes en donnant les modifications entre leurs batiments
*/
ChangeDetectionRes CompareTiles(std::string Folder, citygml::CityModel* City1, citygml::CityModel* City2)//Version GDAL
{
    std::vector<citygml::CityModel *> tiles;
    tiles.push_back(City1);
    tiles.push_back(City2);

    //On part du principe que le plus vieux est dans le 0 et le plus recent dans le 1
    std::vector<OGRMultiPolygon *> EnveloppeCity[2];//Version avec un ensemble de polygones pour chaque batiment (un batiment = un polygone distinct apres union)
    OGRMultiPolygon * EnveloppeCityU[2];//Version avec des polygones unis pour representer le batiment (ce n'est pas l'ensemble de polygones bruts)

    EnveloppeCityU[0] = new OGRMultiPolygon;//nullptr;
    EnveloppeCityU[1] = new OGRMultiPolygon;//nullptr;

    for (int i = 0; i < 2; ++i)
    {
        citygml::CityModel* model = tiles[i];

        int cpt = 0;

        OGRMultiPolygon * ModelPolygons = new OGRMultiPolygon;//Contient tous les polygones composant le model courant

        for (citygml::CityObject* obj : model->getCityObjectsRoots())
        {
            if (obj->getType() == citygml::COT_Building)
            {
                OGRMultiPolygon* Building = new OGRMultiPolygon;//Version OGR du batiment qui va etre remplie

                for (citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du batiment
                {
                    if (object->getType() == citygml::COT_RoofSurface)
                    {
                        for (citygml::Geometry* Geometry : object->getGeometries()) //pour chaque geometrie
                        {
                            for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                            {
                                OGRPolygon * OgrPoly = new OGRPolygon;
                                OGRLinearRing * OgrRing = new OGRLinearRing;

                                for (TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
                                {
                                    OgrRing->addPoint(Point.x, Point.y, Point.z);
                                }

                                OgrRing->closeRings();

                                if (OgrRing->getNumPoints() > 3)
                                {
                                    OgrPoly->addRingDirectly(OgrRing);
                                    if (OgrPoly->IsValid())
                                    {
                                        Building->addGeometry(OgrPoly);
                                        ModelPolygons->addGeometryDirectly(OgrPoly);
                                    }
                                }
                                else
                                    delete OgrRing;
                            }
                        }
                    }
                }

                if (Building->IsEmpty())
                {
                    cpt++;
                    std::cout << "Avancement tuile " << i + 1 << " : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
                    continue;
                }

                OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);
                if (EnveloppeCityU[i] == nullptr)
                    EnveloppeCityU[i] = Enveloppe;
                else
                {
                    OGRMultiPolygon * tmp = EnveloppeCityU[i];
                    EnveloppeCityU[i] = (OGRMultiPolygon *)tmp->Union(Enveloppe);
                    delete tmp;
                }
            }

            cpt++;
            std::cout << "Avancement tuile " << i + 1 << " : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
        }
        std::cout << std::endl;

        //SaveGeometrytoShape("ModelInit.shp", ModelPolygons);

        //Creation de EnveloppeCity : pour chaque batiment distinct, il contient la liste des polygones qui le composent. Ces batiments ne correspondent pas a ceux du CityGML, mais
        //aux polygones distincts apparus avec les unions successives.

        for (int g = 0; g < EnveloppeCityU[i]->getNumGeometries(); ++g)
        {
            OGRPolygon * CurrBati = (OGRPolygon *)EnveloppeCityU[i]->getGeometryRef(g); //On parcourt chaque "batiment" (batiment = un polygon isole apres l'union)

            if (!CurrBati->IsValid())
            {
                EnveloppeCityU[i]->removeGeometry(g);
                g--;
                continue;
            }

            OGRMultiPolygon * Bati = new OGRMultiPolygon;//Contiendra liste des polygones pour batiment i

            //std::cout << "G = " << g << "/" << EnveloppeCityU[i]->getNumGeometries() << std::endl;
            for (int j = 0; j < ModelPolygons->getNumGeometries(); ++j)//Pour le batiment courant, on va chercher dans ModelPolygons quels sont les polygones qui le composent.
            {
                if (ModelPolygons->getGeometryRef(j)->Intersects(CurrBati))//Ce polygone appartient bien a CurrBati
                {
                    OGRGeometry * Inter = ModelPolygons->getGeometryRef(j)->Intersection(CurrBati);
                    if (Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D || Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D) //L'intersection n'est pas un simple point
                    {
                        Bati->addGeometry(ModelPolygons->getGeometryRef(j));
                        ModelPolygons->removeGeometry(j);//On le retire de ModelPolygons car il est maintenant associe a CurrBati
                        j--;
                    }
                }
            }
            EnveloppeCity[i].push_back(Bati);
        }
    }

    std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Compare = CompareBati(Folder, EnveloppeCityU[0], EnveloppeCityU[1], EnveloppeCity[0], EnveloppeCity[1]);

    for (auto& it : EnveloppeCity[0]) delete it;
    for (auto& it : EnveloppeCity[1]) delete it;

    OGRMultiPolygon* BatiDetruits = new OGRMultiPolygon;
    OGRMultiPolygon* BatiCrees = new OGRMultiPolygon;
    OGRMultiPolygon* BatiModifies1 = new OGRMultiPolygon;
    OGRMultiPolygon* BatiModifies2 = new OGRMultiPolygon;
    OGRMultiPolygon* BatiInchanges = new OGRMultiPolygon;

    for (int i = 0; i < EnveloppeCityU[0]->getNumGeometries(); ++i)
    {
        if (Compare.first[i].size() == 0)
            BatiDetruits->addGeometry(EnveloppeCityU[0]->getGeometryRef(i));
        else
        {
            bool modifie = false;
            for (std::size_t j = 0; j < Compare.first[i].size(); j += 2)
            {
                if (Compare.first[i][j] == -2)
                {
                    modifie = true;
                    BatiModifies2->addGeometry(EnveloppeCityU[1]->getGeometryRef(Compare.first[i][j + 1]));
                }
            }
            if (modifie)
                BatiModifies1->addGeometry(EnveloppeCityU[0]->getGeometryRef(i));
            else
                BatiInchanges->addGeometry(EnveloppeCityU[1]->getGeometryRef(Compare.first[i][1]));
        }
    }
    for (int i = 0; i < EnveloppeCityU[1]->getNumGeometries(); ++i)
    {
        if (Compare.second[i].size() == 0)
            BatiCrees->addGeometry(EnveloppeCityU[1]->getGeometryRef(i));
    }

    ChangeDetectionRes Res;

    Res.EnveloppeCityU1 = EnveloppeCityU[0];
    Res.EnveloppeCityU2 = EnveloppeCityU[1];
    Res.BatiCrees = BatiCrees;
    Res.BatiDetruits = BatiDetruits;
    Res.BatiInchanges = BatiInchanges;
    Res.BatiModifies1 = BatiModifies1;
    Res.BatiModifies2 = BatiModifies2;

    return Res;
}
