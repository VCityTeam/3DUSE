#include "ogrsf_frmts.h"
#include "src/utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"

#include "RayBox.hpp"

//RayBoxHit

bool operator<(const RayBoxHit& a, const RayBoxHit& b)
{
    return a.minDistance < b.minDistance;
}

//RayBoxCollection

RayBoxCollection::RayBoxCollection(std::vector<RayBox*> raysBoxes)
{
    this->raysBB = raysBoxes;
}

//RayBox

RayBox::RayBox(TVec3d ori, TVec3d dir, std::string id)
{
    this->id = id;
    this->ori = ori;
    this->dir = dir;
    inv_dir = TVec3d(1 / dir.x, 1 / dir.y, 1 / dir.z);
    sign[0] = (inv_dir.x < 0);
    sign[1] = (inv_dir.y < 0);
    sign[2] = (inv_dir.z < 0);
    fragCoord.x = -1;
    fragCoord.y = -1;
    boxes = std::vector<RayBoxHit>();
}


//Ray aabb intersection, from pbrt-v2
//License : http://www.pbrt.org/LICENSE.txt
bool RayBox::Intersect(AABB box, float *hitt0, float *hitt1)
{
    float t0 = 0, t1 = FLT_MAX;
    for (int i = 0; i < 3; ++i) {
        // Update interval for _i_th bounding box slab
        float invRayDir = 1.f / dir[i];
        float tNear = (box.min[i] - ori[i]) * invRayDir;
        float tFar = (box.max[i] - ori[i]) * invRayDir;

        // Update parametric interval from slab intersection $t$s
        if (tNear > tFar) std::swap(tNear, tFar);
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;
        if (t0 > t1) return false;
    }
    if (hitt0) *hitt0 = t0;
    if (hitt1) *hitt1 = t1;
    return true;
}

osg::Vec3 Rotation(osg::Vec3 d, osg::Vec3 u, double angle) //Rotation d'angle de V autour de u
{
    osg::Vec3 Res;
    u.normalize();
    double C = cos(osg::DegreesToRadians(angle));
    double S = sin(osg::DegreesToRadians(angle));

    double x = (u.x()*u.x()*(1 - C) + C)*d.x() + (u.x()*u.y()*(1 - C) - u.z()*S)*d.y() + (u.x()*u.z()*(1 - C) + u.y()*S)*d.z();
    double y = (u.x()*u.y()*(1 - C) + u.z()*S)*d.x() + (u.y()*u.y()*(1 - C) + C)*d.y() + (u.y()*u.z()*(1 - C) - u.x()*S)*d.z();
    double z = (u.x()*u.z()*(1 - C) - u.y()*S)*d.x() + (u.y()*u.z()*(1 - C) + u.x()*S)*d.y() + (u.z()*u.z()*(1 - C) + C)*d.z();

    Res = osg::Vec3(x, y, z);

    Res.normalize();

    return Res;
}

RayBoxCollection* RayBoxCollection::BuildCollection(osg::Camera* cam)
{
    //OGRMultiLineString* MLS = new OGRMultiLineString;
    RayBoxCollection* rays = new RayBoxCollection();

    double fovx;
    double aspect;
    double znear;
    double zfar;
    cam->getProjectionMatrixAsPerspective(fovx, aspect, znear, zfar);

    if (fovx < 0)
        fovx += 360;

    osg::Viewport* viewport = cam->getViewport();

    float width = viewport->width();
    float height = viewport->height();

    std::cout << width << " " << height << std::endl;

    double fovy = (height / width)*fovx;

    osg::Vec3d pos;
    osg::Vec3d target;
    osg::Vec3d up;
    cam->getViewMatrixAsLookAt(pos, target, up);

    target -= pos;

    osg::Vec3d right = target ^ osg::Vec3d(0.0, 0.0, 1.0);
    up = right ^ target;

    TVec3d rayori(pos.x(), pos.y(), pos.z());

    for (int i = -width / 2; i < width / 2; ++i)
    {
        osg::Vec3d RayX = Rotation(target, up, -i * fovx / width);
        right = RayX ^ up;

        /////
        TVec3d raydir(RayX.x(), RayX.y(), RayX.z());

        RayBox* ray = new RayBox(rayori, raydir);
        TVec2d fragCoord = TVec2d(i + width / 2, height / 2);
        /*if(fragCoord.x >= width || fragCoord.y >= height)
        {
            std:: cout << fragCoord.x << " " << width << " || " << fragCoord.y << " " << height << std::endl;
            int a;
            std::cin >> a;
        }*/
        ray->fragCoord = fragCoord;
        //ray->collection = rays;
        rays->raysBB.push_back(ray);
        /////

        //OGRLineString* Line = new OGRLineString;
        //Line->addPoint(rayori.x, rayori.y, rayori.z);
        //Line->addPoint(rayori.x + 1000 * raydir.x, rayori.y + 1000 * raydir.y, rayori.z + 1000 * raydir.z);
        //MLS->addGeometry(Line);

        for (int j = -height / 2; j < height / 2; ++j)
        {
            osg::Vec3d NewRay = Rotation(RayX, right, j * fovy / height);

            TVec3d raydir(NewRay.x(), NewRay.y(), NewRay.z());

            RayBox* ray = new RayBox(rayori, raydir);
            TVec2d fragCoord = TVec2d(i + width / 2, j + height / 2);
            ray->fragCoord = fragCoord;
            /*if(fragCoord.x >= width || fragCoord.y >= height)
            {
                std:: cout << fragCoord.x << " " << width << " || " << fragCoord.y << " " << height << std::endl;
                int a;
                std::cin >> a;
            }*/
            //ray->collection = rays;
            rays->raysBB.push_back(ray);

            //OGRLineString* Line = new OGRLineString;
            //Line->addPoint(rayori.x, rayori.y, rayori.z);
            //Line->addPoint(rayori.x + 1000 * raydir.x, rayori.y + 1000 * raydir.y, rayori.z + 1000 * raydir.z);
            //MLS->addGeometry(Line);
        }
    }

    //SaveGeometrytoShape("Rayons.shp", MLS);

    //delete MLS;

    return rays;
}



/*RayBoxCollection* RayBoxCollection::BuildCollection(osg::Camera* cam)
{
RayBoxCollection* raysBoxes = new RayBoxCollection();
for(unsigned int i = 0; i < cam->getViewport()->width(); i++)
{
for(unsigned int j = 0; j < cam->getViewport()->height(); j++)
{
RayBox* raybox = new RayBox();
raybox->BuildRd(TVec2d(i,j),cam);
raysBoxes->raysBB.push_back(raybox);
}
}

return raysBoxes;
}*/
