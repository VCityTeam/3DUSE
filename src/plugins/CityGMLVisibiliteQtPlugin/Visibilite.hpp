#ifndef __VISIBILITE_HPP__
#define __VISIBILITE_HPP__

#include <string>
#include <unordered_map>

#include <qrgb.h>
#include <QColor>

#include "Triangle.hpp"
#include "filters/raytracing/Hit.hpp"

/**
*	@brief Viewpoint analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param cam Camera used to capture the viewpoint	
*	@param prefix File prefix when exporting
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTileBasicAnalyse(std::string dirTile, osg::Camera* cam, std::string prefix = "", double DistLod1 = 0);
/**
*	@brief Vertical cascade viewpoints analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param cam Camera used to capture the viewpoint	
*	@param count How many viewpoint there is in the cascade
*	@param zIncrement Delta height between viewpoint of the cascade
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTileCascadeAnalyse(std::string dirTile,osg::Camera* cam, unsigned int count, float zIncrement);
/**
*	@brief Multiple viewpoints analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param cam Camera used to capture the viewpoint	
*	@param viewpoints Set of viewpoints to analyse
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTileMultiViewpointAnalyse(std::string dirTile,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);
/**
*	@brief Viewpoint panorama analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param cam Camera used to capture the viewpoint	
*	@param prefix File prefix when exporting
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiTilePanoramaAnalyse(std::string dirTile,osg::Camera* cam, std::string prefix = "");
/**
*	@brief Vertical cascade panorama viewpoints analysis using the multitile algorithm
*	@param dirTile Directory where tiles are located
*	@param cam Camera used to capture the viewpoint	
*	@param count How many viewpoint there is in the cascade
*	@param zIncrement Delta height between viewpoint of the cascade
*	@return The analysis results
*/
std::vector<std::vector<ViewPoint*>> MultiTileCascadePanoramaAnalyse(std::string dirTile,osg::Camera* cam, unsigned int count, float zIncrement);

/**
*	@brief Perform a viewpoint analysis on different viewpoint
*	@param cams List of camera corresponding to viewpoints
*	@param buildingPath Path to the citygml file of the buildings
*	@param terrainPath Path to the citygml file of the terrains
*	@return The analysis results
*/
std::vector<ViewPoint*> DoMonoTileAnalysis(std::string dirTile, std::vector<osg::ref_ptr<osg::Camera>> cams,std::vector<std::string> paths);

/**
*	@brief Viewpoint analysis
*	@param paths List of tiles to use for the analysis
*	@param cam Camera used to capture the viewpoint	
*	@return The analysis results
*/
std::vector<ViewPoint*> BasisAnalyse(std::string dirTile, std::vector<std::string> paths,osg::Camera* cam);

/**
*	@brief Vertical cascade viewpoints analysis
*	@param paths List of tiles to use for the analysis
*	@param cam Camera used to capture the viewpoint	
*	@param count How many viewpoint there is in the cascade
*	@param zIncrement Delta height between viewpoint of the cascade
*	@return The analysis results
*/
std::vector<ViewPoint*> CascadeAnalyse(std::string dirTile, std::vector<std::string> paths,osg::Camera* cam, unsigned int count, float zIncrement);

/**
*	@brief Multiple viewpoints analysis
*	@param paths List of tiles to use for the analysis
*	@param cam Camera used to capture the viewpoint	
*	@param viewpoints Set of viewpoints to analyse
*	@return The analysis results
*/
std::vector<ViewPoint*> MultiViewpointAnalyse(std::string dirTile, std::vector<std::string> paths,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints);

#endif
