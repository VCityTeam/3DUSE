#ifndef __AABB_HPP__
#define __AABB_HPP__

#include <string>


#include "gui/osg/osgScene.hpp"

/**
*	@brief An axis aligned bounding box
*/
struct AABB
{
	osg::Vec3d min;///< Min point of the box
	osg::Vec3d max;///< Max point of the box
	std::string name;///< Name of the box
};



/**
*	@brief Build bounding box for a set of tile, bounding box are saved on disk, not returned
*	@param dir Directory where citygml are located, must contain two directory for terrain and building
*	@param name Name of the tiles to build
*	@param offset Offset of the geometry in 3Duse
*/
void BuildAABB(std::string dir, std::string name, TVec3d offset);

/**
*	@brief Load an aabb set previously build
*	@param dir Directory where citygml are located, must contain two directory for terrain and building
*	@param name Name of the set of tile to load
*	@return List of AABB for the set of tile, <BuildingAABB,TerrainAABB>
*/
std::pair<std::vector<AABB>,std::vector<AABB>> LoadAABB(std::string dir, std::string name);

#endif