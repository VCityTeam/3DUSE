#ifndef __AABB_HPP__
#define __AABB_HPP__

#include <string>
#include <vector>

#include "vecs.hpp"
#include "citygmlutils_export.h"
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

bool operator<(const BoxOrder& a, const BoxOrder& b);

/**
*	@brief Pour une box, contient un certain nombre d'informations liés aux rayons que l'on aura tenté d'intersecter avec celle ci.
*/
struct CITYGMLUTILS_EXPORT BoxwithRays
{
    AABB box; //  Box concernée
    std::vector<int> IndicesRays; //Contient les indices des rayons qui ont intersecté cette box
    float minDistance; //Distance minimale entre la box et la caméra
};

bool operator<(const BoxwithRays& a, const BoxwithRays& b);

/**
*	Used to store a bounding box collection of different layer
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
CITYGMLUTILS_EXPORT void BuildAABB(std::string dir);

/**
*	@brief Load an aabb set previously build
*	@param dir Directory where citygml files are located, must contain several subdirectories, one for each data layer (terrain, building, water, ..)
*	@return AABB Collection of the set of tiles
*/
CITYGMLUTILS_EXPORT AABBCollection LoadAABB(std::string dir);

#endif
