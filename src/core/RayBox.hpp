#ifndef RAYBOX_HPP
#define RAYBOX_HPP

#include <vector>

#include "raytracing/RayTracing.hpp"
#include "AABB.hpp"
#include "osg/Camera"
#include "vecs.hpp"

/**
*	@brief A hit between a ray and a bounding box
*/
struct RayBoxHit
{
    AABB box;///< Box hitted
    float minDistance;///< Distance between the ray origin and the bounding box
};

bool operator<(const RayBoxHit& a, const RayBoxHit& b);


/**
 * @brief Ray - Bounding Boxes related methods and members
 */
class RayBox : public Ray
{

public:

    /**
    *	@brief Build a new rayBox
    *	@param ori Origin of the rayBox
    *	@param dir Direction of the rayBox
    *   @param id oh the rayBox
    */
    RayBox(TVec3d ori = TVec3d(0.0,0.0,0.0),TVec3d dir = TVec3d(1.0,1.0,1.0),int id = -1 );

    /**
    *	@brief To know if the ray instersects a given box
    *	@param box Box to intersect
    *	@param hitt0 t0 at when ray first hits the box
    *	@param hitt1 t1 at when ray first leaves the box
    *	@return True if this ray intersect with the box
    */
    bool Intersect(AABB box, float *hitt0, float *hitt1);

    std::vector<RayBoxHit> boxes;///< List of boxes that this ray go through

};



/**
*	@brief A collection of RayBox
*/
struct RayBoxCollection
{
    /**
    *	@brief Build a new collection
    */
    RayBoxCollection(std::vector<RayBox*> raysBoxes = std::vector<RayBox*>());

    /**
    *	@brief Build a collection of rays from a camera
    *	@param cam The camera we want to used to build the collection
    */
    static RayBoxCollection* BuildCollection(osg::Camera* cam);

    std::vector<RayBox*> raysBB;///< The rays of the collection
};


#endif // RAYBOX_HPP
