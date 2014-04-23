////////////////////////////////////////////////////////////////////////////////
#include "exportOBJ.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterOBJ::ExporterOBJ()
{

}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityObject(CityObject* obj, const std::string& fileName)
{
    //if(obj->getType() == citygml::COT_RoofSurface)
    { //Si surface de toit
        //std::cout << "Nouveau Toit trouvé" << std::endl;
        std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
        std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
        for(; itGeom != geoms.end(); ++itGeom){ //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)

            std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
            std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
            for(; itPoly != polys.end(); ++itPoly){ //Pour chaque polygone

                citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                const std::vector<TVec3d>& vertices = ring->getVertices();
                std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
                for(; itVertices != vertices.end(); ++itVertices){//pour Chaque sommet

                    TVec3d point = *itVertices;
                }
            }
        }
    }

    citygml::CityObjects& cityObjects = obj->getChildren();
    citygml::CityObjects::iterator itObj = cityObjects.begin();
    for(; itObj != cityObjects.end(); ++itObj){
        exportCityObject(*itObj, fileName);
    }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
