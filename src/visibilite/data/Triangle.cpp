#include "Triangle.hpp"

/**
*	@brief A random number between low and high
*/
int RandInt(int low, int high)
{
	// Random number between low and high
	return qrand() % ((high + 1) - low) + low;
}

TriangleList* BuildTriangleList(std::string tilefilename, citygml::CityObjectsType objectType)
{
	std::vector<Triangle*> triangles;

	vcity::Tile* tile = new vcity::Tile(tilefilename);

	citygml::CityModel * model = tile->getCityModel();

	for(citygml::CityObject* obj : model->getCityObjectsRoots()) //For each city object
	{
		if(obj->getType() == citygml::COT_Building && objectType == citygml::COT_Building) //We only take building or terrain
		{
			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
				for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
					{
						//Get triangle list
						const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
						const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

						for(unsigned int i = 0 ; i < ind.size() / 3; i++)//Push all triangle of the polygon in our list
						{
							TVec3d a = vert[ind[ i * 3 + 0 ]];
							TVec3d b = vert[ind[ i * 3 + 1 ]];
							TVec3d c = vert[ind[ i * 3 + 2 ]];

							Triangle* t = new Triangle(a,b,c);
							t->subObjectType = object->getType();
							t->objectType = obj->getType();
							t->objectId = obj->getId();
							t->polygonId = PolygonCityGML->getId();
							t->tileFile = tilefilename;

							triangles.push_back(t);
						}
					}
		}
		// #CityObjectType
		// We check if the current cityobject is the same type of the wanted type of cityobject given in parameter
		// Exemple : (obj->getType() == citygml::COT_<MyType> && objectType == citygml::COT_<MyType>
		else if((obj->getType() == citygml::COT_SolitaryVegetationObject  && objectType == citygml::COT_SolitaryVegetationObject) || 
			(obj->getType() == citygml::COT_TINRelief  && objectType == citygml::COT_TINRelief) || 
			(obj->getType() == citygml::COT_WaterBody  && objectType == citygml::COT_WaterBody))
		{

			for(citygml::Geometry* Geometry : obj->getGeometries()) //pour chaque géométrie
				for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
				{
					//Get triangle list
					const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
					const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

					for(unsigned int i = 0 ; i < ind.size() / 3; i++)//Push all triangle of the polygon in our list
					{
						TVec3d a = vert[ind[ i * 3 + 0 ]];
						TVec3d b = vert[ind[ i * 3 + 1 ]];
						TVec3d c = vert[ind[ i * 3 + 2 ]];

						Triangle* t = new Triangle(a,b,c);
						t->objectType = obj->getType();
						t->objectId = obj->getId();
						t->polygonId = PolygonCityGML->getId();
						t->tileFile = tilefilename;

						triangles.push_back(t);
					}
				}
		}
	}

	delete tile;

	return new TriangleList(triangles);
}