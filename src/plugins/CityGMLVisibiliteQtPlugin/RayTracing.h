#ifndef __RAYTRACING_HPP__
#define __RAYTRACING_HPP__

#include "data/Ray.hpp"

/**
*	@build Perform a raytracing on a set of triangles
*	@param triangles The list of triangle from the CityGML tile
*	@param rays List of rays
*/
void RayTracing(TriangleList* triangles, std::vector<Ray*> rays);

#endif