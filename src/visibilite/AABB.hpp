#ifndef __AABB_HPP__
#define __AABB_HPP__

#include <string>


#include "gui/osg/osgScene.hpp"

struct AABB
{
	osg::Vec3d min;
	osg::Vec3d max;
	std::string name;
};



/**
*	@brief Build bounding box for a set of tile
*	@param dir Directory where citygml are located, must contain two directory for terrain and building
*	@param name Name of the tiles to build
*	@param offset Offset of the geometry in 3Duse
*/
void BuildAABB(std::string dir, std::string name, TVec3d offset);

/**
*	@brief Load an aabb set previously build
*	@param dir Directory where citygml are located, must contain two directory for terrain and building
*	@param name Name of the set of tile to load
*	@return List of AABB for the set of tile
*/
std::pair<std::vector<AABB>,std::vector<AABB>> LoadAABB(std::string dir, std::string name);

#endif