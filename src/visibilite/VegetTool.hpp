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
*	@param outputFile Name of the resulting file (without ext)
*/
void ProcessLasShpVeget();

void ProcessCL(std::map<OGRPolygon*,std::vector<OGRPoint>> vegets, std::string outputFile);

#endif