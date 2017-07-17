// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef __TRIANGLE_HPP__
#define __TRIANGLE_HPP__

#include "libcitygml/citygml.hpp"
#include "libcitygml/utils/tile.hpp"
#include "citygmlutils_export.h"
#ifdef _MSC_VER
#pragma warning(disable: 4251) // VC++ DLL jejune complains on STL _Id member
#endif

/**
*	@brief A triangle created from a citygml polygon
*/
struct CITYGMLUTILS_EXPORT Triangle
{
    /**
    *	@brief Build a new triangle
    *	@param a First point of the triangle
    *	@param b Second point of the triangle
    *	@param c Third point of the triangle
    */
    Triangle(TVec3d a = TVec3d(0.0, 0.0, 0.0), TVec3d b = TVec3d(0.0, 0.0, 0.0), TVec3d c = TVec3d(0.0, 0.0, 0.0));

    TVec3d GetNormal();

    TVec3d a; ///< First point of the triangle
    TVec3d b; ///< Second point of the triangle
    TVec3d c; ///< Third point of the triangle

    citygml::CityObjectsType objectType;
    citygml::CityObjectsType subObjectType;
    std::string objectId;
    std::string polygonId;
    std::string tileFile;
};

/**
*	A list of triangle
*
*/
struct CITYGMLUTILS_EXPORT TriangleList
{
    /**
    *	@brief Build a new collection of triangle
    */
    TriangleList(std::vector<Triangle*> triangles = std::vector<Triangle*>());
    ~TriangleList();

    std::vector<Triangle*> triangles;///< Triangles of the list
};

/**
*	@brief Build list of triangle from a CityGML building tile
*	@param tile CityGML tile from which we want the triangle list
*	@param viewpoint Data about the viewpoint we are rendering
*	@param objectType The type of cityobject to load
*   @param cityObjId The id of city object to load.
*          Default = "" (i.e. all cityobjects of the tile are loaded)
*   @param zMin A minimum z value of triangles to load
*          (if all vertices of the triangle are below this zMin value, triangle is not loaded)
*          Default = -10000.0
*	@return The list of triangle from the CityGML tile
*/
CITYGMLUTILS_EXPORT TriangleList* BuildTriangleList(
  const std::string& tilefilename,
  const citygml::CityObjectsType& objectType,
  const std::string& cityObjId = "",
  const double& zMin = -10000.0
);

#endif
