#ifndef __BUILDOSGNODE_HPP__
#define __BUILDOSGNODE_HPP__

#include "gui/osg/osgScene.hpp"

#include "src/visibilite/data/ViewPoint.h"

/**
*	@brief Build the osg node to display a skyline in 3DUse
*	@param skyline Skyline
*	@return The osg node
*/
osg::ref_ptr<osg::Node> BuildSkylineOSGNode(std::vector<std::pair<TVec2d,TVec3d>> skyline, std::string prefix = "");

/**
*	@Brief Build the viewshed
*	@param result Result of the analysis
*/
osg::ref_ptr<osg::Node> BuildViewshedOSGNode(ViewPoint* result, std::string prefix = "");

/**
*	@Brief Build an ogs node from a set of points
*	@param points The set of points
*/
osg::ref_ptr<osg::Node> BuildViewshedOSGNode(std::vector<TVec3d> points, std::string prefix = "");

#endif