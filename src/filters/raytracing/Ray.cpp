#include "Ray.hpp"
#include "Hit.hpp"
#include "Triangle.hpp"

//Ray

Ray::Ray(TVec3d ori, TVec3d dir, std::string id)
{
    this->id = id;
    this->ori = ori;
    this->dir = dir;
    inv_dir = TVec3d(1/dir.x, 1/dir.y, 1/dir.z);
    sign[0] = (inv_dir.x < 0);
    sign[1] = (inv_dir.y < 0);
    sign[2] = (inv_dir.z < 0);
    fragCoord.x = -1;
    fragCoord.y = -1;
}

//Ray triangle intersection, from geometric tools engine
//License : http://www.boost.org/LICENSE_1_0.txt
bool Ray::Intersect(Triangle* triangle, Hit* hit)
{
    bool result;
    Hit tempHit;

    // Compute the offset origin, edges, and normal.
    TVec3d diff = this->ori - triangle->a;
    TVec3d edge1 = triangle->b - triangle->a;
    TVec3d edge2 = triangle->c - triangle->a;
    TVec3d normal = edge1.cross(edge2);

    // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
    // E1 = edge1, E2 = edge2, N = Cross(E1,E2)) by
    //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
    //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
    //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
    float DdN = this->dir.dot(normal);
    float sign;
    if (DdN > (float)0)
    {
        sign = (float)1;
    }
    else if (DdN < (float)0)
    {
        sign = (float)-1;
        DdN = -DdN;
    }
    else
    {
        // Ray and triangle are parallel, call it a "no intersection"
        // even if the ray does intersect.
        result = false;
        return result;
    }

    float DdQxE2 = sign*DotCross(this->dir, diff, edge2);
    if (DdQxE2 >= (float)0)
    {
        float DdE1xQ = sign*DotCross(this->dir, edge1, diff);
        if (DdE1xQ >= (float)0)
        {
            if (DdQxE2 + DdE1xQ <= DdN)
            {
                // Line intersects triangle, check whether ray does.
                float QdN = -sign*diff.dot(normal);
                if (QdN >= (float)0)
                {
                    // Ray intersects triangle.
                    result = true;

                    tempHit.intersect = true;
                    float inv = ((float)1) / DdN;
                    tempHit.parameter = QdN*inv;
                    tempHit.triangleBary[1] = DdQxE2*inv;
                    tempHit.triangleBary[2] = DdE1xQ*inv;
                    tempHit.triangleBary[0] = (float)1 - tempHit.triangleBary[1]
                    - tempHit.triangleBary[2];
                    tempHit.point = this->ori +
                        this->dir * tempHit.parameter;

                    tempHit.distance = (this->dir * tempHit.parameter).length();
                    tempHit.triangle = *triangle;
                    tempHit.ray = (*this);

                    if(hit != nullptr)
                    {
                        *hit = tempHit;
                    }
                    return result;
                }
                // else: t < 0, no intersection
            }
            // else: b1+b2 > 1, no intersection
        }
        // else: b2 < 0, no intersection
    }
    // else: b1 < 0, no intersection

    result = false;
    return result;
}

void Ray::BuildRd(TVec2d fragCoord,osg::Camera* cam)
{
    double fov;
    double aspect;
    double znear;
    double zfar;
    cam->getProjectionMatrixAsPerspective(fov,aspect,znear,zfar);

    osg::Viewport* viewport = cam->getViewport();

    float width = viewport->width();
    float height = viewport->height();

    fov = osg::DegreesToRadians(fov);
    float fovy = (height/width)*fov;

	
    //See : http://www.unknownroad.com/rtfm/graphics/rt_eyerays.html for the code to build a ray

    float x = ((2*fragCoord.x - width)/width) * tan(fov);
    float y = ((2*fragCoord.y - height)/height) * tan(fovy);

    osg::Vec3f ori(0.0,0.0,0.0);
    osg::Vec3f dir(x,y,-znear/2);

    osg::Quat rot = osg::Matrixd::inverse(cam->getViewMatrix()).getRotate();

    ori = ori * osg::Matrixd::inverse(cam->getViewMatrix());
    dir = rot *  dir ;

    dir.normalize();

    TVec3d rayori(ori.x(),ori.y(),ori.z());
    TVec3d raydir(dir.x(),dir.y(),dir.z());

    //Ray* ray = new Ray(rayori,raydir);

    this->ori = rayori;
    this->dir = raydir;
    inv_dir = TVec3d(1/this->dir.x, 1/this->dir.y, 1/this->dir.z);
    sign[0] = (inv_dir.x < 0);
    sign[1] = (inv_dir.y < 0);
    sign[2] = (inv_dir.z < 0);

    this->fragCoord = fragCoord;
}

float Ray::DotCross(TVec3d v0, TVec3d v1,
               TVec3d v2)
{
    return v0.dot( v1.cross(v2));
}

TVec3d Ray::Normalized(TVec3d vec)
{
    return vec/vec.length();
}

//RayCollection

RayCollection::RayCollection(std::vector<Ray*> rays)
{
    this->rays = rays;
}

RayCollection* RayCollection::BuildCollection(osg::Camera* cam)
{
    RayCollection* rays = new RayCollection();
    for(unsigned int i = 0; i < cam->getViewport()->width(); i++)
    {
        for(unsigned int j = 0; j < cam->getViewport()->height(); j++)
        {
            Ray* ray = new Ray();
            ray->BuildRd(TVec2d(i,j),cam);
            //ray->collection = rays;
            rays->rays.push_back(ray);
        }
    }

    return rays;
}
