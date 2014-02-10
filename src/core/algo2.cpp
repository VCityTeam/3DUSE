////////////////////////////////////////////////////////////////////////////////
#include "algo2.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
    void Algo2::processRec(citygml::CityObject* node)
    {
        log() << "processRec \n";

        if(node->getType() == citygml::COT_WallSurface)
        {
            log() << "MUR \n";
        }

        // parse geometry
        std::vector<citygml::Geometry*>& geoms = node->getGeometries();
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
                    // Points
                    TVec3d point = *itVertices;
                    log() << "X:" << point.x << "\n";
                    log() << "Y:" << point.y << "\n";
                    log() << "Z:" << point.z << "\n";
                }
            }
        }

        citygml::CityObjects& cityObjects = node->getChildren();
        citygml::CityObjects::iterator itObj = cityObjects.begin();
        for(; itObj != cityObjects.end(); ++itObj)
        {
            processRec(*itObj); //Appel récursif
        }
    }

////////////////////////////////////////////////////////////////////////////////
    void Algo2::fixBuilding(const std::vector<URI>& uris)
    {

        log() << "fixBuilding \n";


        const std::vector<vcity::URI>& uris2 = vcity::app().getSelectedNodes();

        vcity::URI uri = uris[0];
        citygml::CityObject* obj = app().getScene().getNode(uri);

        log() << "test : " << obj->getId() << "\n";


        if(uris.size() >= 2)
        {
            citygml::CityObject* building = nullptr;
            citygml::CityObject* terrain = nullptr;

            building = app().getScene().getNode(uris[0]);
            terrain = app().getScene().getNode(uris[1]);

            if(building->getType() != citygml::COT_Building)
            {
                citygml::CityObject* tmp = building;
                building = terrain;
                terrain = tmp;
            }

            log() << "building : " << building->getId() << "\n";
            log() << "terrain : " << terrain->getId() << "\n";

            /***************************** Parcourt les murs ******************************/
            processRec(building);

            /************************** Trouve les points du bas **************************/


            /*************************** Teste les intersections **************************/
            processRec(terrain);


            /************************* Bouge les points si besoin *************************/


        }
    }
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
