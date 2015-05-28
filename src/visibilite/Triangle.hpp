#ifndef __TRIANGLE_HPP__
#define __TRIANGLE_HPP__

#include "gui/osg/osgScene.hpp"

/**
*	@brief A triangle create from a citygml polygon
*/
struct Triangle
{
	/**
	*	@brief Build a new triangle
	*	@param a First point of the triangle
	*	@param b Second point of the triangle
	*	@param c Third point of the triangle
	*/
	Triangle(TVec3d a = TVec3d(0.0,0.0,0.0),TVec3d b = TVec3d(0.0,0.0,0.0),TVec3d c = TVec3d(0.0,0.0,0.0))
	{
		this->a = a;
		this->b = b;
		this->c = c;
		polygon = nullptr;
		geometry = nullptr;
		subObject = nullptr;
		object = nullptr;
	}

	TVec3d GetNormal()
	{
		TVec3d normal = (b - a).cross(c - a);
		return normal/normal.length();
	}

	TVec3d a; ///< First point of the triangle
	TVec3d b; ///< Second point of the triangle
	TVec3d c; ///< Third point of the triangle

	citygml::Polygon* polygon; ///< CityGML polygon in which the triangle belong
	citygml::Geometry* geometry; ///< CityGML geometry in which the polygon belong
	citygml::CityObject* subObject; ///< Roof or wall in which the geometry belong
	citygml::CityObject* object; ///< City Object in which the object belong
};

#endif