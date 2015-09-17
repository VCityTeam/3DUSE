#ifndef __AABB_HPP__
#define __AABB_HPP__

#include <string>


#include "gui/osg/osgScene.hpp"

/**
*	@brief An axis aligned bounding box
*/
struct AABB
{
	TVec3d min;///< Min point of the box
	TVec3d max;///< Max point of the box
	std::string name;///< Name of the box

	bool operator==(AABB const& other)
	{
		return name == other.name;
	}
};

/**
*	@brief A hit between a ray and a bounding box
*/
struct RayBoxHit
{
	AABB box;///< Box hitted
	float minDistance;///< Distance between the ray origin and the bounding box

	bool operator<(RayBoxHit const& b)
	{
		return this->minDistance < b.minDistance;
	}
};

/**
*	Used when ordering a collection of bounding box
*/
struct BoxOrder
{
	std::string box;///< Name of the bounding box
	unsigned int order;///< Order of the box in the collection

	bool operator<(BoxOrder const& b)
	{
		return this->order < b.order;
	}
};

/**
*	Used to store bounding box collection of different layer
*/
struct AABBCollection
{
	std::vector<AABB> building;///< Bounding box of the building layer
	std::vector<AABB> terrain;///< Bounding box of the terrain layer
	std::vector<AABB> water;///< Bounding box of the water layer
	std::vector<AABB> veget;///< Bounding box of the veget layer
	// #AABBNewDataSet
	// Uncomment next comment to add a data set (and replace myData by your data)
	// std::vector<AABB> myData;
};

/**
*	@brief Build bounding box for a set of tile, bounding box are saved on disk, not returned
*	@param dir Directory where citygml are located
*/
void BuildAABB(std::string dir);

/**
*	@brief Load an aabb set previously build
*	@param dir Directory where citygml are located, must contain two directory for terrain and building
*	@param name Name of the set of tile to load
*	@return List of AABB for the set of tile, <BuildingAABB,TerrainAABB>
*/
AABBCollection LoadAABB(std::string dir);

#endif