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
    //   [    // The first vector encodes what has become of "old" buildings
    //  first[0] = [ -1, 0]   // Building 0 keeps same index and unchanged
    //  first[1] = [ -1, 2]   // Building 1 relabeled to become building 2
    //  first[2] = [ -1, 1]   // Building 2 relabeled to become building 1
    //  first[3] = [ -2, 4]   // Building 3 changed height and relabeled to 4
    //  first[4] = []         // Building 4 disappeared
    //  first[5] = [ -2, 6, -2, 7] // Building 5 was split into 6 and 7
    //  ,    // The second vector encodes where the "new" buildings come from
    //  second[0]= [ -1, 0]   // Building 0 is unchanged
    //  second[1]= [ -1, 2]   // Building 1 was known as building 2
    //  second[2]= [ -1, 1]   // Building 2 was known as building 1
    //  second[3]= []         // Building 3 is a totally new building
    //  second[4]= [ -2, 3]   // Was previously known as 3 and changed height
    //  second[5]= []         // Another entirely new building (just as 3)
    //  second[6]= [ -2, 5]   // Was previously known as 3 a part of 5
    //  second[7]= [ -2, 5]   // Just as 6, 7 was previously 3 a part of 5
    //   ]

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
* @brief Compare two CityGML descrition of a same geographical zone
*        corresponding to two different dates (moments) in order to retrieve
*        the difference (modifications) of the geometries of the Buildings.
* @note  This function uses the OGR (provided by GDAL) based representation
*        for the geometries (polygons) it manipulates.
* @warning The reconstructed buildings, resulting from this algorithm, do NOT
*        necessarily correspond to the "original" CityGML (semantic) buildings.
*        Such a difference will happen when the original CityGML model
*        erroneously lumped many semantical buildings into a single cityGML
*        building (for example all the buildings of a block are described, at
*        the CityGML as a single building). When such an erroneous input occurs
*        the algorithm will set appart buildings that are geometrical
*        disconnected (not sharing a vertex, an edge or a face and of course
*        not spatially intersecting) although they were described within the
*        input as a single building.
* @param[in] City1 The "oldest" of the two modesl to be compared
* @param[in] City2 The "youngest" of the two modesl to be compared
*/
ChangeDetectionRes CompareTiles(std::string Folder,
                                citygml::CityModel* City1,
                                citygml::CityModel* City2)
{
    // Keep in mind the strong made assumption: 0 is the index of the oldest
    // model while 1 is the thus the index of the "youngest" model.
    std::vector<citygml::CityModel *> tiles;
    tiles.push_back(City1);
    tiles.push_back(City2);

    // The version of the building envelopes each of which described as
    // a set of its original/raw polygons of its constituting CityGML::Objects
    // (wall, roof..). This representation holds an OGR convertion of the
    // original CityGML building constituting objects prior to any other
    // treatment.
    std::vector<OGRMultiPolygon *> EnveloppeCity[2];

    // Each of the above building envelopes has its own corresponding original
    // CityGML identifier.
    // Note that std::string stands here as the type representing a CityGML
    // object Id (refer to cityGML::object.getId() method).
    std::vector<std::string> BuildingID[2];

    // United version of the building envelopes: each building is thus here
    // represented as a single United polygone (i.e. the result of the union
    // of the original/raw polygons of its constituting CityGML::Objects.
    // This United polygonal representation of a building is better adapted
    // when wishing to identify each building with its single polygon envelope.
    OGRMultiPolygon * EnveloppeCityU[2];
    EnveloppeCityU[0] = new OGRMultiPolygon;//nullptr;
    EnveloppeCityU[1] = new OGRMultiPolygon;//nullptr;

    // Sets of geometries (used as auxillary within the scope of this
    // algorithm) holding all the polygons constituting the buildings of
    // the respective models.
    OGRMultiPolygon * ModelPolygons[2];
    ModelPolygons[0] = new OGRMultiPolygon;
    ModelPolygons[1] = new OGRMultiPolygon;

    // The CityGML building Id orginally associated to a considered Geometry
    // (of a building). The index of this vector correspond to the index
    // of the corresponding geometry within ModelPolygons.
    // Note that std::string stands here as the type representing a CityGML
    // object Id (refer to cityGML::object.getId() method).
    std::vector<std::string> PolygonBuildingID[2];

    // For each of the two models convert the CityGML geometries of all of
    // their respective buildings into a set (OGRMultiPolygon) of polygons
    // (OGRPolygon).
    for (int i = 0; i < 2; ++i)
    {
        std::cout << "Model " << i + 1 << ": " << std::endl << std::flush;
        citygml::CityModel* model = tiles[i];
        int buildingNumber = 0;

        for (citygml::CityObject* obj : model->getCityObjectsRoots())
        {
            if (obj->getType() != citygml::COT_Building)
            {
                continue;
            }

            // A building is here reduced to its geometry represented
            // as an OGRMultiPolygon
            OGRMultiPolygon* Building = new OGRMultiPolygon;

            // Iteration on all the (cityGML) objects (Wall, Roof, ...)
            // that belong to the considered building:
            for (citygml::CityObject* object : obj->getChildren())
            {
                if (object->getType() != citygml::COT_RoofSurface)
                {
                    continue;
                }

                std::string buildingID = object->getId();

                // Iteration on each geometry of the constituting object
                for (citygml::Geometry* Geometry : object->getGeometries())
                {
                    // Iteration on each polygone
                    for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                    {
                        OGRPolygon    * OgrPoly = new OGRPolygon;
                        OGRLinearRing * OgrRing = new OGRLinearRing;

                        for (TVec3d Point :
                              PolygonCityGML->getExteriorRing()->getVertices())
                        {
                          OgrRing->addPoint(Point.x, Point.y, Point.z);
                        }

                        OgrRing->closeRings();

                        if (OgrRing->getNumPoints() > 3)
                        {
                          OgrPoly->addRingDirectly(OgrRing);
                          if (OgrPoly->IsValid())
                          {
                            // Use the newly constructed OGR polygon:
                            Building->addGeometry(OgrPoly);
                            ModelPolygons[i]->addGeometryDirectly(OgrPoly);
                            PolygonBuildingID[i].push_back( buildingID );
                          }
                        }
                        else
                        {
                          delete OgrRing;
                        }
                    }
                }
            }

            // Displaying of algorithm progress
            if (!Building->IsEmpty())
            {
                // Construction of the United representation of the building
                // envelopes (refer above on the distinction between
                // EnveloppeCity and EnveloppeCityU).
                OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);
                if (EnveloppeCityU[i] == nullptr)
                {
                  EnveloppeCityU[i] = Enveloppe;
                }
                else
                {
                  OGRMultiPolygon * tmp = EnveloppeCityU[i];
                  EnveloppeCityU[i] = (OGRMultiPolygon *)tmp->Union(Enveloppe);
                  delete tmp;
                }
            }

            // Displaying the algorithm progress
            buildingNumber++;
            std::cout << "    Converting building: "
            << buildingNumber << "/" << model->getCityObjectsRoots().size()
            << " \r" << std::flush;

        } // for loop on CityGML objects
        std::cout << std::endl << "Done." << std::endl;
    }

    // For each of the two models, proceed with the creation de EnveloppeCity.
    for (int model = 0; model < 2; ++model)
    {
      // Reminder: the reconstructed buildings might NOT correspond to the
      // CityGML original (semantic) buildings: refer to the above warning
      // about erroneously lumped building (successive geometrical unions)
      // as a pr-etreatment of data realized prior to the usage of the present
      // algorithm. In other terms the provided CityGML already contains
      // "corrupted data" (e.g. the fusion of spatially separated buildings
      // within a single CityGML "logical building).
      //
      // The algorithm (below) reconstructs each independent building of the
      // resulting EnveloppeCity by spatially disassociating the "original"
      // polygons that composed the lumped (United) building.
      // Note that we are constrained to reconstruct such a constituting
      // list precisely because the considered United buildings were
      // previously lumped/fusioned/merged through logical lumping.
      // Otherwise it would be possible to construct such lists directly
      // (in the above construction of EnveloppeCityU[i]) only if we
      // considered the CityGML "semantical" buildings.
      //
      // Iterate on the buildings each of which is represented as a single
      // isolated polygon (since we consider EnveloppeCityU i.e. the United
      // geometrical representation)
      for (int g = 0; g < EnveloppeCityU[model]->getNumGeometries(); ++g)
      {
        OGRPolygon * originalBuilding =
        (OGRPolygon*)EnveloppeCityU[model]->getGeometryRef(g);

        // Disregard the geometries that are not valid (United) building
        // representations by dropping them...
        if (!originalBuilding->IsValid())
        {
          EnveloppeCityU[model]->removeGeometry(g);
          g--;
          continue;
        }

        // The target set of the original polygons constituting the
        // ith United Building:
        OGRMultiPolygon * finalBuilding = new OGRMultiPolygon;
        std::string finalBuildingID = "_UNSET_";

        // We look within ModelPolygons[i] in order to retrieve the polygons
        // matching (through intersection) the Current Building
        for (int j = 0; j < ModelPolygons[model]->getNumGeometries(); ++j)
        {
          auto geomRef = ModelPolygons[model]->getGeometryRef(j);
          // When this polygon matches/belongs to originalBuilding
          if(geomRef->Intersects(originalBuilding))
          {
            OGRGeometry * Inter = geomRef->Intersection(originalBuilding);
            // When the intersection (between the current original
            // polygon and the current United single building polygon)
            // is not reduced to a single point, THEN we associate it
            // to the finalBuilding polygon list and thus remove
            // it from the ModelPolygons[i] list:
            if (   Inter->getGeometryType() == wkbPolygon
            || Inter->getGeometryType() == wkbPolygon25D
            || Inter->getGeometryType() == wkbMultiPolygon
            || Inter->getGeometryType() == wkbMultiPolygon25D)
            {
              finalBuilding->addGeometry(geomRef);
              ModelPolygons[model]->removeGeometry(j);

              if( finalBuildingID == "_UNSET_" )
              {
                finalBuildingID = PolygonBuildingID[model][j];
              }
              else
              {
                // Asserting that all is well
                if( finalBuildingID != PolygonBuildingID[model][j] )
                {
                   std::cout << "WARNING"
                             << "   All polygon ID of this building shoud be "
                             << std::endl
                             << "   "
                             << finalBuildingID
                             << std::endl
                             << "   but instead this identifier was found "
                             << std::endl
                             << "   "
                             << PolygonBuildingID[model][j]
                             << std::endl;
                }
              }
              // Purge the associated BuildingID in order to keep
              // ModelPolygons[i] and PolygonBuildingID[i] aligned.
              PolygonBuildingID[model].erase(
                                       PolygonBuildingID[model].begin() + j );
              j--;
            }
          }
        }
        EnveloppeCity[model].push_back(finalBuilding);
        BuildingID[model].push_back(finalBuildingID);
      }
    } // for model

    /////////////
    // Based on the above convertions, the following part of the algorithm
    // can now trace the becoming of a building between its "oldest"
    // version to its "newest" version (or possibly versions).
    std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Compare = CompareBati(Folder,
                          EnveloppeCityU[0],
                          EnveloppeCityU[1],
                          EnveloppeCity[0],
                          EnveloppeCity[1]);

    for (auto& it : EnveloppeCity[0]) delete it;
    for (auto& it : EnveloppeCity[1]) delete it;

    OGRMultiPolygon* BatiDetruits  = new OGRMultiPolygon;
    OGRMultiPolygon* BatiCrees     = new OGRMultiPolygon;
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

    // Kludgy temporary exploratory code
    std::pair< std::vector<std::vector<std::string> >,
               std::vector<std::vector<std::string> > > BuildingCorrespondencies;
    int nbrInitialBuilding = EnveloppeCityU[0]->getNumGeometries();
    int   nbrFinalBuilding = EnveloppeCityU[1]->getNumGeometries();

    BuildingCorrespondencies.first.resize( nbrInitialBuilding );
    BuildingCorrespondencies.second.resize( nbrFinalBuilding );

    for (int building = 0;
             building < nbrInitialBuilding;
             building++)
    {
      std::cout << PolygonBuildingID[0][building];
      // The encoded vector of encoded correspondences
      auto correspondence = Compare.first[building];
      std::size_t nbrCorrespondents = correspondence.size();
      if ( nbrCorrespondents == 0 )
      {
          BuildingCorrespondencies.first[building].push_back("Destroyed");
          std::cout << ": destroyed." << std::endl;
          continue;
      }
      // The building has one or possibly many associated future Buildings
      if( ! nbrCorrespondents %2 )
      {
        std::cout << "ERROR: The encoding of Compare should always be even!"
                  << std::endl;
      }

      int changed =  Compare.first[building][0];
      if (changed == -1)
      {
         std::cout << ": unchanged.";
      }
      else if (changed == -2)
      {
         std::cout << ": changed to ";
      }
      else
      {
        std::cout << "ERROR: change status must be -1 or -2"
                  << std::endl;
      }
      for (std::size_t j = 0; j < nbrCorrespondents; j += 2)
      {
          // Test wheter all the nature of changes (-1 or -2) are coherent
          // (they should all be the same). Not that when then is only
          // single corresponding building this is a dummy comparison...
          if (Compare.first[building][j] != changed)
          {
             std::cout << "ERROR: change status are not homogeneous..."
                       << std::endl;
          }
          if (Compare.first[building][j] == -1)
          {
              if( PolygonBuildingID[0][building] != PolygonBuildingID[1][Compare.first[building][j + 1]] )
              std::cout << "renumbered to "
                        << PolygonBuildingID[1][Compare.first[building][j + 1]]
                        << " (oldindex: " << Compare.first[building][j + 1]
                        << "), ";
              continue;
          }
          if (Compare.first[building][j] == -2)
          {
              std::cout << PolygonBuildingID[1][Compare.first[building][j + 1]]
                        << " (oldindex: " << Compare.first[building][j + 1]
                        << "), ";
              continue;
          }
          std::cout << "ERROR" << std::endl;
      }
      std::cout << std::endl;
    }

    for (int newBuilding = 0;
             newBuilding < nbrFinalBuilding;
             newBuilding++)
    {
      // The encoded vector of encoded possible correspondences
      auto correspondence = Compare.second[newBuilding];
      if ( correspondence.size() == 0 )
      {
          std::cout << PolygonBuildingID[1][newBuilding]
                    << " : Created."
                    << std::endl;
          continue;
      }
    }

    return Res;
}
