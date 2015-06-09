#ifndef __VISIBILITE_HPP__
#define __VISIBILITE_HPP__

#include <string>
#include <unordered_map>

#include <qrgb.h>
#include <QColor>

#include "Triangle.hpp"
#include "Hit.hpp"

struct ViewPoint
{
	ViewPoint(unsigned int width, unsigned int height)
	{
		this->width = width;
		this->height = height;
		hits = new Hit*[width];
		for(unsigned int i = 0; i < width; i++)
			hits[i] = new Hit[height];

		minDistance = FLT_MAX;
		maxDistance = FLT_MIN;
	}

	~ViewPoint()
	{
		for(unsigned int i = 0; i < width; i++)
			delete[] hits[i];
		delete[] hits;
	}

	/**
	*	@brief Compute the skyline of the viewpoint
	*/
	void ComputeSkyline();

	/**
	*	@brief Reset the viewpoint hits array
	*/
	void Reset();

	

	Hit** hits;///< Hit of the rays
	unsigned int width;///< Width of hits
	unsigned int height;///< Height of hits
	TVec3d lightDir;///< Direction of the light in the scene
	float minDistance;///< Minimum distance to a triangle in the scene
	float maxDistance;///< Maximum distance to a triangle in the scene
	std::map<std::string,QColor> objectToColor;///< Map a city object id to a color
	std::vector<std::pair<unsigned int,unsigned int>> skyline;///< Skyline of the viewpoints
	RayCollection* rays;///< Ray used for rendering this viewpoint

private:
	inline unsigned int Clamp(unsigned int x,unsigned int a,unsigned int b);

	/**
	*	Enum used when build skyline, position relative to a pixel
	*/
	enum Position
	{
		W = 0,//West
		NW = 1,//North West
		N = 2,//North
		NE = 3,//North East
		E = 4,//East
		SE = 5,//South East
		S = 6,//South
		SW = 7//South West
	};

	/**
	*	@brief Get the shift need to know the coordinate in the position p
	*	@param p Position
	*	@return Shift need in <x,y>
	*/
	std::pair<int, int> GetCoord(Position p);
};


/**
*	@build Analyse a CityGML tile
*	@param path Path to the CityGML file on disk
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param useSkipMiscBuilding If not remarquable building must be skip during the building analysis
*	@param exportFilePrefix	If we must put something before the file name when exporting
*	@return A skyline of the analysis
*/
std::vector<TVec3d> Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam = nullptr, bool useSkipMiscBuilding = false, std::string exportFilePrefix = "");

/**
*	@build Analyse a CityGML tile
*	@param path Path to the CityGML file on disk
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param count How many time we rune the analysis
*	@param zIncrement How much we increase the z coordinate of the cam each analysis
*/
void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement);

/**
*	@build Analyse a CityGML tile
*	@param path Path to the CityGML file on disk
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*	@param viewpoints List of viewpoint where to analyse, <position,target>
*/
void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);

/**
*	@build Perform a raytracing on a CityGML tile
*	@param triangles The list of triangle from the CityGML tile
*	@param viewpoint Data about the viewpoint we are rendering
*	@param offset Offset of the geometry in 3Duse
*	@param cam A camera that can be used for the ray tracing
*/
void RayTracing(TriangleList* triangles, ViewPoint* viewpoint, TVec3d offset, osg::Camera* cam);

/**
*	@brief Build list of triangle from a CityGML building tile
*	@param tile CityGML tile from which we want the triangle list
*	@param offset offset Offset of the geometry in 3Duse
*	@param viewpoint Data about the viewpoint we are rendering
*	@param ignoreMiscBuilding If true this will ignore building that are not remarquable
*	@return The list of triangle from the CityGML tile
*/
TriangleList BuildBuildingTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint, bool ignoreMiscBuilding);

/**
*	@brief Build list of triangle from a CityGML terrain tile
*	@param tile CityGML tile from which we want the triangle list
*	@param offset offset Offset of the geometry in 3Duse
*	@param viewpoint Data about the viewpoint we are rendering
*	@return The list of triangle from the CityGML tile
*/
TriangleList BuildTerrainTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint);

/**
*	@brief Build the osg node to display a skyline in 3DUse
*	@param skyline Skyline
*	@return The osg node
*/
osg::ref_ptr<osg::Geode> BuildSkylineOSGNode(std::vector<TVec3d> skyline);

#endif