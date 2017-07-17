// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef _VEGETTOOL_

#define _VEGETTOOL_

#include "src/core/application.hpp"

/**
*	@brief Compute vegatation using a shp and las file
*	@return outputFile Name of the resulting file (without ext)
*/
std::string ProcessLasShpVeget(std::string dirTile);

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
