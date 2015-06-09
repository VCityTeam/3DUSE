#ifndef __RAY_HPP__
#define __RAY_HPP__

#include "Triangle.hpp"

struct Hit;



/**
*	@brief A simple ray
*/
struct Ray
{
	/**
	*	@brief Build a new ray
	*	@param ori Origin of the ray
	*	@param dir Direction of the ray
	*/
	Ray(TVec3d ori = TVec3d(0.0,0.0,0.0),TVec3d dir = TVec3d(0.0,0.0,0.0))
	{
		this->ori = ori;
		this->dir = dir;
		inv_dir = TVec3d(1/dir.x, 1/dir.y, 1/dir.z);
		sign[0] = (inv_dir.x < 0);
		sign[1] = (inv_dir.y < 0);
		sign[2] = (inv_dir.z < 0);
	}

	/**
	*	@brief To know if the ray instersect a given triangle
	*	@param triangle The triangle to intersect
	*	@param hit Information about to hit of the ray on the triangle
	*	@return True if this ray intersect with triangle
	*/
	bool Intersect(Triangle* triangle, Hit* hit = nullptr);
	/**
	*	@brief Build a ray 
	*	@param fragCoord Coordinate on screen of the ray
	*	@param cam Camera used to build the ray
	*	@return A newly created ray coresponding to the given coord on the cam screen
	*/
	static Ray* BuildRd(TVec2d fragCoord,osg::Camera* cam);

	static float DotCross(TVec3d v0, TVec3d v1,
		TVec3d v2);
	static TVec3d Normalized(TVec3d vec);

	TVec2d fragCoord;///< Fragment coordinate of the ray
	TVec3d ori;///< Origin of the ray
	TVec3d dir;///< Direction of the ray
	TVec3d inv_dir;///< inv Direction of the ray
	int sign[3];
};

struct RayCollection
{
	RayCollection(std::vector<Ray*> rays = std::vector<Ray*>())
	{
		this->rays = rays;
	}

	~RayCollection()
	{
		for(unsigned int i = 0; i < rays.size(); i++)
		{
			delete rays[i];
		}
	}

	static RayCollection BuildCollection(osg::Camera* cam);

	std::vector<Ray*> rays;
};

#endif