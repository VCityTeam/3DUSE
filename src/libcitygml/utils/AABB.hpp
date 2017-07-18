// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef __AABB_HPP__
#define __AABB_HPP__

#include <QString>

#include <string>
#include <vector>

#include "vecs.hpp"
#include "citygmlutils_export.h"
#include <float.h>
#ifdef _MSC_VER
#pragma warning(disable: 4251) // VC++ DLL jejune complains on STL _Id member
#endif

// FIXME: convert this file to plain ascii (remove accented letters)

/**
*	@brief An axis aligned bounding box
*/
struct CITYGMLUTILS_EXPORT AABB
{
    TVec3d min;///< Min point of the box
    TVec3d max;///< Max point of the box
    std::string name;///< Name of the box

    bool operator==(AABB const& other);
};

/**
*	Used when ordering a collection of bounding boxes
*/
struct CITYGMLUTILS_EXPORT BoxOrder
{
    std::string box;///< Name of the bounding box
    unsigned int order;///< Order of the box in the collection
};

CITYGMLUTILS_EXPORT bool operator<(const BoxOrder& a, const BoxOrder& b);

/**
*	@brief Pour une box, contient un certain nombre d'informations lies aux rayons que l'on aura tente d'intersecter avec celle ci.
*/
struct CITYGMLUTILS_EXPORT BoxwithRays
{
    AABB box; //  Box concernee
    std::vector<int> IndicesRays; //Contient les indices des rayons qui ont intersecte cette box
    float minDistance; //Distance minimale entre la box et la camera
};

CITYGMLUTILS_EXPORT bool operator<(const BoxwithRays& a, const BoxwithRays& b);

/**
*	Used to store a bounding box collection for different layers
*/
struct CITYGMLUTILS_EXPORT AABBCollection
{
    std::vector<AABB> building;///< Bounding box of the building layer
    std::vector<AABB> terrain;///< Bounding box of the terrain layer
    std::vector<AABB> water;///< Bounding box of the water layer
    std::vector<AABB> veget;///< Bounding box of the veget layer
    // #AABBNewDataSet
    // Uncomment next comment to add a data set (and replace myData by your data)
    // std::vector<AABB> myData;
};

/**
*	@brief Build bounding box from a set of tile, bounding boxes are saved on disk in a text file, not returned
*	@param dir Directory where the citygml files are located
*/
CITYGMLUTILS_EXPORT void BuildLayersAABBs(std::string dir);

/**
*	@brief Load a collection of box from a file
*	@param path Path to the file
*	@return A collection of box
*/
CITYGMLUTILS_EXPORT std::vector<AABB> LoadAABBFile(std::string path);

/**
*	@brief Load an aabb set previously build
*	@param dir Directory where citygml files are located, must contain several subdirectories, one for each data layer (terrain, building, water, ..)
*	@return AABB Collection of the set of tiles
*/
CITYGMLUTILS_EXPORT AABBCollection LoadLayersAABBs(std::string dir);

///
/// \brief BuildBuildingAABBs Loops recursively through folder containing gml files representing buildings, compute and save AABBs for CityGML Building and Building Parts notions.
///                           One file will contain Building AABBs (tilename_Building_AABB.dat) and the other one Building Parts AABBs (tilename_BuildingParts_AABB.dat).
///
/// \param buildingFilesFolder Path to folder holding Building gml files
///
CITYGMLUTILS_EXPORT void BuildBuildingAABBs (QString buildingFilesFolder);

#endif
