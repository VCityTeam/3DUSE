#ifndef __VISIBILITE_HPP__
#define __VISIBILITE_HPP__

#include <string>
#include <unordered_map>

#include <qrgb.h>
#include <QColor>

#include "data/Triangle.hpp"
#include "data/Hit.hpp"

/**
*	@brief Viewpoint analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param prefix File prefix when exporting
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTileBasicAnalyse(std::string dirTile, TVec3d offset, osg::Camera* cam, std::string prefix = "");
/**
*	@brief Vertical cascade viewpoints analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param count How many viewpoint there is in the cascade
*	@param zIncrement Delta height between viewpoint of the cascade
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTileCascadeAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement);
/**
*	@brief Multiple viewpoints analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param viewpoints Set of viewpoints to analyse
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTileMultiViewpointAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);
/**
*	@brief Viewpoint panorama analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param prefix File prefix when exporting
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTilePanoramaAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, std::string prefix = "");
/**
*	@brief Vertical cascade panorama viewpoints analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param count How many viewpoint there is in the cascade
*	@param zIncrement Delta height between viewpoint of the cascade
*	@return The analysis results
*/
std::vector<std::vector<ViewPoint*>> MultiTileCascadePanoramaAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement);

/**
*	@brief Perform a viewpoint analysis on different viewpoint
*	@param cams List of camera corresponding to viewpoints
*	@param buildingPath Path to the citygml file of the buildings
*	@param terrainPath Path to the citygml file of the terrains
*	@param offset Offset of the geometry in 3Duse
*	@return The analysis results
*/
std::vector<ViewPoint*> DoMonoTileAnalysis(std::vector<osg::ref_ptr<osg::Camera>> cams,std::vector<std::string> paths, TVec3d offset);

/**
*	@brief Viewpoint analysis
*	@param paths List of tiles to use for the analysis
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@return The analysis results
*/
std::vector<ViewPoint*> BasisAnalyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam);

/**
*	@brief Vertical cascade viewpoints analysis
*	@param paths List of tiles to use for the analysis
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param count How many viewpoint there is in the cascade
*	@param zIncrement Delta height between viewpoint of the cascade
*	@return The analysis results
*/
std::vector<ViewPoint*> CascadeAnalyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement);

/**
*	@brief Multiple viewpoints analysis
*	@param paths List of tiles to use for the analysis
*	@param offset 3D offset used by 3D-Use
*	@param cam Camera used to capture the viewpoint	
*	@param viewpoints Set of viewpoints to analyse
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiViewpointAnalyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);

#endif