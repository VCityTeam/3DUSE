# How to parse a CityGML native file from C++ (Developer's guide)

C'est un guide rapide pour accéder aux données dans le code.

Le format CityGML est en xml, donc hiérarchique. Tout en haut, on a le CityModel. Celui ci contient une liste de CityObject. Et ensuite CityObject contient des Geometry et peut aussi contenir d'autre CityObject. Enfin, Geometry contient des Polygon. Les Polygon sont en fait des listes de points.

Il faut savoir que CityObject est une classe de base et est spécialisée en de nombeaux types :
  * WallSurface
  * GroundSurface
  * RoofSurface
  * ...

Exemple de code qui parse un CityGML et descend jusqu'aux polygones :

```c++
#include "core/application.hpp"

void processRec(citygml::CityObject* node)
{
    // access to semantic type
    if(node->getType() == citygml::COT_Building)
    {
        // do stuffs...
    }
    else if(node->getType() == citygml::COT_WallSurface) 
	     // or COT_RoofSurface, COT_GroundSurface, ...
    {
        // do some stuff...
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
                // do stuff with points...
                TVec3d point = *itVertices;
                point.x = ...
                point.y = ...
                point.z = ...
            }
        }
    }

    citygml::CityObjects& cityObjects = node->getChildren();
    citygml::CityObjects::iterator itObj = cityObjects.begin();
    for(; itObj != cityObjects.end(); ++itObj)
    {
        processRec(*itObj); // recursive call
    }
}

void process()
{
    // get first tile - ugly (crash if no tiles loaded)
    vcity::Tile* tile =
vcity::app().getScene().getDefaultLayer()->getTiles()[0];

    // get citymodel
    citygml::CityModel* model = tile->getCityModel();

    // parse the cityobjects, recursive
    citygml::CityObjects& cityObjects = model->getCityObjectsRoots();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        processRec(*it); // recursive call
    }
}
```

Dans le cas du TINRelief, il faut procéder de la même manière pour récupérer les triangles. Ils sont stockés dans des LinearRing (un triangle par LinearRing, 3 points).