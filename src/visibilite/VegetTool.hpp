#ifndef _VEGETTOOL_

#define _VEGETTOOL_

#include "src/core/application.hpp"

/**
*	@brief Cut a Shp file using a bounding box
*	@param min Bottom left point of the bounding box
*	@param max Top right point of the bounding box
*	@param outputFile Name of the resulting shp
*/
void CutShape(TVec2d min, TVec2d max, std::string outputFile);

/**
*	@brief Compute vegatation using a shp and las file
*	@return outputFile Name of the resulting file (without ext)
*/
std::string ProcessLasShpVeget();

/**
*	@brief Convert a cloud point to a vegetation city object
*	@param veg The cloud point
*	@param cpt Int param caracterising the point cloud
*/
citygml::CityObject* VegToCityObject(std::vector<TVec3d> veg, unsigned int cpt);

/**
*	@brief Process a point cloud from a file name
*/
void ProcessCL(std::string filename);

/**
*	@brief Process a point cloud from a set of points
*	@param vegets A set of point clouds
*/
void ProcessCL(std::vector<std::pair<std::string,std::vector<TVec3d>>> vegets, std::string output);

#endif