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
	Hit** hits;///< Hit of the rays
	Hit** hitsRemarquableBuildingOnly;///< Hit of the rays but with only the remarquable building taken in account;
	unsigned int width;///< Width of hits
	unsigned int height;///< Height of hits
	TVec3d lightDir;///< Direction of the light in the scene
};

/**
*	@build Analyse a CityGML tile
*	@param path Path to the CityGML file on disk
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*/
void Analyse(std::string path, TVec3d offset,osg::Camera* cam = nullptr);

/**
*	@build Perform a raytracing on a CityGML tile
*	@param triangles The list of triangle from the CityGML tile
*	@param globalData Where to write some data generate globaly
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@return The result
*/
RayTracingResult RayTracing(std::vector<Triangle*> triangles, GlobalData* globalData, TVec3d offset, osg::Camera* cam = nullptr);

/**
*	@brief Export image from a ray tracing result
*	@param result Result from the ray tracing algorithm
*	@param globalData Where to write some data generate globaly
*/
void ExportImage(RayTracingResult result, GlobalData* globalData);

/**
*	@brief Process the results of a ray tracing algorithm
*	@param result Result from the ray tracing algorithm
*	@param globalData Where to write some data generate globaly
*/
void ProcessResult(RayTracingResult result, GlobalData* globalData);

/**
*	@brief Build list of triangle from a CityGML tile
*	@param tile CityGML tile from which we want the triangle list
*	@param offset offset Offset of the geometry in 3Duse
*	@param globalData Where to write some data generate from the triangle list
*	@return The list of triangle from the CityGML tile
*/
std::vector<Triangle*> BuildTriangleList(vcity::Tile* tile, TVec3d offset, GlobalData* globalData);

