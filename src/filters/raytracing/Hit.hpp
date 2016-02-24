#ifndef __HIT_HPP__
#define __HIT_HPP__

#include "vecs.hpp"
#include "Triangle.hpp"
#include "Ray.hpp"

//struct Triangle;
//struct Ray;

/**
*	@brief An impact from a ray on a triangle
*/
struct Hit
{
	/**
	*	@brief Create a new Hit
	*/
    Hit();

	Ray ray;///< The ray that hit
	bool intersect;///< If yes or no the ray hit the triangle
	float distance;///< Distance from the origin of the ray to the hit position
	float parameter;///< T0
	float triangleBary[3];///< ?
	TVec3d point;///< Hit position
	Triangle triangle;///< Triangle that was hit
};

#endif
