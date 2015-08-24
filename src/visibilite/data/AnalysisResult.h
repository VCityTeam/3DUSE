#ifndef __ANALYSISRESULT_HPP__
#define __ANALYSISRESULT_HPP__

#include "ViewPoint.h"

/**
*	@brief the result of a viewpoint analysis
*/
struct AnalysisResult
{
	std::vector<std::pair<TVec2d,TVec3d>> skyline;///< Points composing the skyline of the viewpoint, <Fragment Coordinates, 3D Coordinates>
	TVec3d viewpointPosition;///< 3D position of the viewpoint
	ViewPoint* viewpoint;///< Reference to the viewpoint data
};


#endif