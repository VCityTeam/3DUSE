// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
////////////////////////////////////////////////////////////////////////////////
#include "algo2.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "application.hpp"
#include "gui/applicationGui.hpp"

////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
    float Algo2::sign(TVec3d p1, TVec3d p2, TVec3d p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

////////////////////////////////////////////////////////////////////////////////
    bool Algo2::pointDansTriangle(TVec3d pt, TVec3d v1, TVec3d v2, TVec3d v3)
    {
        bool b1, b2, b3;
        
        b1 = sign(pt, v1, v2) < 0.0f;
        b2 = sign(pt, v2, v3) < 0.0f;
        b3 = sign(pt, v3, v1) < 0.0f;
        
        return ((b1 == b2) && (b2 == b3));
    }

////////////////////////////////////////////////////////////////////////////////
    void Algo2::stockeForme(citygml::CityObject* noeud1, int val)
    {
        if(noeud1->getType() == citygml::COT_WallSurface)
        {
//            log() << "MUR \n";
        }

        std::vector<citygml::Polygon*> polyBat;

        // parse geometry
        std::vector<citygml::Geometry*>& geoms = noeud1->getGeometries();
        std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
        for(; itGeom != geoms.end(); ++itGeom)
        {
            // parse polygons
            std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
            std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
            for(; itPoly != polys.end(); ++itPoly)
            {
                // get linear ring
                citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                citygml::LinearRing* ringOffset = new citygml::LinearRing(ring->getId()+"_offset", true);
                std::vector<TVec3d>& vertices = ring->getVertices();
                std::vector<TVec3d>::iterator itVertices = vertices.begin();
                for(; itVertices != vertices.end(); ++itVertices)
                {
                    // get Points
                    TVec3d point = *itVertices;
                    point.z = val;
                    ringOffset->addVertex(point);
                }

                citygml::Polygon* poly = new citygml::Polygon((*itPoly)->getId());
                poly->addRing(ringOffset);
                polyBat.push_back(poly);
            }

            for(std::vector<citygml::Polygon*>::iterator it = polyBat.begin(); it < polyBat.end(); ++it)
            {
                (*itGeom)->addPolygon(*it);
            }
        }

        citygml::CityObjects& cityObjects = noeud1->getChildren();
        citygml::CityObjects::iterator itObj = cityObjects.begin();
        for(; itObj != cityObjects.end(); ++itObj)
        {
            stockeForme(*itObj,val); //Appel recursif
        }
    }


////////////////////////////////////////////////////////////////////////////////
    void Algo2::recupTerrainBat(citygml::CityObject* noeud2, int s)
    {
        std::vector<citygml::Polygon*> polyTer;

        // parse geometry
        std::vector<citygml::Geometry*>& geoms = noeud2->getGeometries();
        std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
        for(; itGeom != geoms.end(); ++itGeom)
        {
            // parse polygons
            std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
            std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
            for(; itPoly != polys.end(); ++itPoly)
            {
                // get LinearRing
                citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                citygml::LinearRing* ringOffset2 = new citygml::LinearRing(ring->getId()+"_offset2", true);
                std::vector<TVec3d>& vertices = ring->getVertices();
                std::vector<TVec3d>::iterator itVertices = vertices.begin();
                std::vector<TVec3d>& verticesContour = contour->getVertices();

                std::vector<TVec3d>::iterator itVerticesContour = verticesContour.begin();

                for(; itVerticesContour != verticesContour.end(); ++itVerticesContour)
                {
                    TVec3d pointSol = *itVertices;
                    TVec3d pointContour = *itVerticesContour;
                    if(pointDansTriangle(pointContour,*(vertices[0]),*(vertices[1]),*(vertices[2])))
                    {
                        pointSol.z += s;
                        ringOffset2->addVertex(pointSol);
                    }
                }

                citygml::Polygon* poly = new citygml::Polygon((*itPoly)->getId());
                poly->addRing(ringOffset2);
                polyTer.push_back(poly);
            }
            for(std::vector<citygml::Polygon*>::iterator it = polyTer.begin(); it < polyTer.end(); ++it)
            {
                (*itGeom)->addPolygon(*it);
            }

        }

        citygml::CityObjects& cityObjects2 = noeud2->getChildren();
        citygml::CityObjects::iterator itObj = cityObjects2.begin();
        for(; itObj != cityObjects2.end(); ++itObj)
        {
            recupTerrainBat(*itObj, s); //Appel recursif
        }
    }


////////////////////////////////////////////////////////////////////////////////
    void Algo2::recup(citygml::CityObject* noeud3, int i)
    {
        // parse geometry
        std::vector<citygml::Geometry*>& geoms = noeud3->getGeometries();
        std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
        for(; itGeom != geoms.end(); ++itGeom)
        {
            // parse polygons
            std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
            std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
            for(; itPoly != polys.end(); ++itPoly)
            {
                // get linear ring
                citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                std::vector<TVec3d>& vertices = ring->getVertices();
                std::vector<TVec3d>::iterator itVertices = vertices.begin();
                for(; itVertices != vertices.end(); ++itVertices)
                {
                    // do stuff with points...
                    TVec3d point = *itVertices;
                    if(i==1)
                    {
                        hauteurTerrainSousBat.push_back(point.z);
                    }
                    else
                    {
                        solBat.push_back(point.z);
                    }
                }
            }
        }

        citygml::CityObjects& fils = noeud3->getChildren();
        citygml::CityObjects::iterator itObj = fils.begin();
        for(; itObj != fils.end(); ++itObj)
        {
            recup(*itObj, i); //Appel recursif
        }

    }


////////////////////////////////////////////////////////////////////////////////
    void Algo2::fixBuilding(const std::vector<URI>& uris)
    {

        URI uriBuilding = uris[0];
        URI uriTerrain = uris[1];

        //S'il y a au moins deux objets selectionnes
        if(uris.size() >= 2)
        {
            citygml::CityObject* building = nullptr;
            citygml::CityObject* terrain = nullptr;

            building = app().getScene().getCityObjectNode(uris[0]);
            terrain = app().getScene().getCityObjectNode(uris[1]);

            //Si le type n'est pas bon, on permute les 2
            if(building->getType() != citygml::COT_Building)
            {
                citygml::CityObject* tmp = building;
                building = terrain;
                terrain = tmp;

                URI uriTmp = uriBuilding;
                uriBuilding = uriTerrain;
                uriTerrain = uriTmp;
            }

//            log() << "building : " << building->getId() << "\n";
//            log() << "terrain : " << terrain->getId() << "\n";


            /**********************************************************************************
                                     On recupere la forme du batiment
            **********************************************************************************/
            contour = new citygml::LinearRing("linearRingContour",true);

            stockeForme(building,100);
/*
            citygml::Geometry * geom = new citygml::Geometry("fixBuildingGeom",citygml::GT_Floor,2);
            citygml::Polygon * poly = new citygml::Polygon("fixBuildingPoly");

            poly->addRing(contour);
            geom->addPolygon(poly);
*/
            appGui().getControllerGui().update(uriBuilding);


            /**********************************************************************************
                      On teste chaque triangle du terrain avec chaque point du contour
            **********************************************************************************/
            sol = new citygml::LinearRing("linearRingBat",true);

            recupTerrainBat(terrain, 100);

            citygml::Geometry * geom2 = new citygml::Geometry("fixBuildingGeom2",citygml::GT_Ground,2);
            citygml::Polygon * poly2 = new citygml::Polygon("fixBuildingPoly2");

            poly2->addRing(sol);
            geom2->addPolygon(poly2);
            terrain->addGeometry(geom2);

            appGui().getControllerGui().update(uriTerrain);


            /**********************************************************************************
                              Si besoin, on deplace le terrain et le batiment
            **********************************************************************************/
            //Si le terrain est bien sous le batiment
            if(sol->size() != 0)
            {
                // On recupere la hauteur du point le plus haut du terrain
                recup(terrain, 1);
                int max = *std::max_element(hauteurTerrainSousBat.begin(),hauteurTerrainSousBat.end());

                /* On deplace les points du sol situes sous le batiment
                 * au niveau du point le plus haut du sol
                 */
                stockeForme(terrain,max);
                appGui().getControllerGui().update(uriTerrain);

                // On calcule la difference de hauteur entre le point le plus bas du batiment et le terrain
                recup(building, 2);
                int min = *std::min_element(solBat.begin(),solBat.end());
                int diff = min - max;

                /* On deplace le batiment de façon a ce que son sol
                 * soit a la meme hauteur que le terrain
                 */
                recupTerrainBat(building, diff);
                appGui().getControllerGui().update(uriBuilding);

            }
            else
            {
                log() << "Le terrain n'est pas sous le batiment\n";
            }
        }

    }
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
