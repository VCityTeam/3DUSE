////////////////////////////////////////////////////////////////////////////////
#include "algo2.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
void Algo2::fixBuilding(const std::vector<URI>& uris)
{
    if(uris.size() >= 2)
    {
        citygml::CityObject* building = nullptr;
        citygml::CityObject* terrain = nullptr;

        building = app().getScene().getCityObjectNode(uris[0]);
        terrain = app().getScene().getCityObjectNode(uris[1]);

        if(building->getType() != citygml::COT_Building)
        {
            citygml::CityObject* tmp = building;
            building = terrain;
            terrain = tmp;
        }

        log() << "building : " << building->getId() << "\n";
        log() << "terrain : " << terrain->getId() << "\n";

        // parcourir les murs
        // trouver les ponits du bas

        // tester les intersections

        // bouger les points si il faut
    }
}
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
