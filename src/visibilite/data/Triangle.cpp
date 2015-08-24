#include "Triangle.hpp"

#include "ViewPoint.h"

/**
*	@brief A random number between low and high
*/
int RandInt(int low, int high)
{
	// Random number between low and high
	return qrand() % ((high + 1) - low) + low;
}

TriangleList* BuildTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint, citygml::CityObjectsType objectType)
{
	std::vector<Triangle*> triangles;

	citygml::CityModel * model = tile->getCityModel();

	for(citygml::CityObject* obj : model->getCityObjectsRoots()) //For each city object
	{
		if(obj->getType() == citygml::COT_Building && objectType == citygml::COT_Building) //We only take building or terrain
		{
			QColor objectColor = qRgba(RandInt(0,255),RandInt(0,255),RandInt(0,255),255);
			std::pair<std::string,QColor> pair(obj->getId(),objectColor);
			if(viewpoint != nullptr)
				viewpoint->objectToColor.insert(pair);

			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
				for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
					{
						//Get triangle list
						const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
						const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

						for(unsigned int i = 0 ; i < ind.size() / 3; i++)//Push all triangle of the polygon in our list
						{
							TVec3d a = vert[ind[ i * 3 + 0 ]] - offset;
							TVec3d b = vert[ind[ i * 3 + 1 ]] - offset;
							TVec3d c = vert[ind[ i * 3 + 2 ]] - offset;

							Triangle* t = new Triangle(a,b,c);
							t->subObjectType = object->getType();
							t->objectType = obj->getType();
							t->objectId = obj->getId();

							triangles.push_back(t);
						}
					}
		}
		else if((obj->getType() == citygml::COT_SolitaryVegetationObject  && objectType == citygml::COT_SolitaryVegetationObject) || (obj->getType() == citygml::COT_TINRelief  && objectType == citygml::COT_TINRelief) || (obj->getType() == citygml::COT_WaterBody  && objectType == citygml::COT_WaterBody)) //We only take building or terrain
		{
			QColor objectColor = qRgba(RandInt(0,255),RandInt(0,255),RandInt(0,255),255);
			std::pair<std::string,QColor> pair(obj->getId(),objectColor);
			if(viewpoint != nullptr)
				viewpoint->objectToColor.insert(pair);

			for(citygml::Geometry* Geometry : obj->getGeometries()) //pour chaque géométrie
				for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
				{
					//Get triangle list
					const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
					const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

					for(unsigned int i = 0 ; i < ind.size() / 3; i++)//Push all triangle of the polygon in our list
					{
						TVec3d a = vert[ind[ i * 3 + 0 ]] - offset;
						TVec3d b = vert[ind[ i * 3 + 1 ]] - offset;
						TVec3d c = vert[ind[ i * 3 + 2 ]] - offset;

						Triangle* t = new Triangle(a,b,c);
						t->objectType = obj->getType();
						t->objectId = obj->getId();

						triangles.push_back(t);
					}
				}
		}
	}

	return new TriangleList(triangles);
}