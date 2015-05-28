#ifndef __VISIBILITE_HPP__
#define __VISIBILITE_HPP__

#include <string>
#include <unordered_map>

#include <qrgb.h>
#include <QColor>

#include "Triangle.hpp"
#include "Hit.hpp"

/**
*	Data that are for the whole scene (not specific to a pixel)
*/
struct GlobalData
{
	/**
	*	@brief Create a new Global
	*/
	GlobalData()
	{
		minDistance = FLT_MAX;
		maxDistance = FLT_MIN;
	}

	float minDistance;///< Minimum distance to a triangle in the scene
	float maxDistance;///< Maximum distance to a triangle in the scene
	std::map<std::string,QColor> objectToColor;///< Map a city object id to a color
};

/**
*	Result of a ray tracing algorithm
*/
struct RayTracingResult
{
	RayTracingResult(unsigned int width, unsigned int height)
	{
		this->width = width;
		this->height = height;
		hits = new Hit*[width];
		for(unsigned int i = 0; i < width; i++)
			hits[i] = new Hit[height];
	}

	~RayTracingResult()
	{
		for(unsigned int i = 0; i < width; i++)
			delete[] hits[i];
		delete[] hits;
	}

	Hit** hits;///< Hit of the rays
	unsigned int width;///< Width of hits
	unsigned int height;///< Height of hits
	TVec3d lightDir;///< Direction of the light in the scene
};

/**
*	@build Analyse a CityGML tile
*	@param path Path to the CityGML file on disk
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param useSkipMiscBuilding If not remarquable building must be skip during the building analysis
*/
void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam = nullptr, bool useSkipMiscBuilding = false);

/**
*	@build Perform a raytracing on a CityGML tile
*	@param triangles The list of triangle from the CityGML tile
*	@param globalData Where to write some data generate globaly
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param result The result
*/
void RayTracing(std::vector<Triangle*> triangles, GlobalData* globalData, TVec3d offset, osg::Camera* cam, RayTracingResult* result);

/**
*	@brief Build list of triangle from a CityGML building tile
*	@param tile CityGML tile from which we want the triangle list
*	@param offset offset Offset of the geometry in 3Duse
*	@param globalData Where to write some data generate from the triangle list
*	@param ignoreMiscBuilding If true this will ignore building that are not remarquable
*	@return The list of triangle from the CityGML tile
*/
std::vector<Triangle*> BuildBuildingTriangleList(vcity::Tile* tile, TVec3d offset, GlobalData* globalData, bool ignoreMiscBuilding);

/**
*	@brief Build list of triangle from a CityGML terrain tile
*	@param tile CityGML tile from which we want the triangle list
*	@param offset offset Offset of the geometry in 3Duse
*	@param globalData Where to write some data generate from the triangle list
*	@return The list of triangle from the CityGML tile
*/
std::vector<Triangle*> BuildTerrainTriangleList(vcity::Tile* tile, TVec3d offset, GlobalData* globalData);

#endif