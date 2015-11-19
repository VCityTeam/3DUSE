#include "Ray.hpp"
#include "Hit.hpp"
#include "src/processes/ExportToShape.hpp"

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

//Ray aabb intersection, from pbrt-v2
//License : http://www.pbrt.org/LICENSE.txt
bool Ray::Intersect(AABB box, float *hitt0, float *hitt1)
{
	float t0 = 0, t1 = FLT_MAX;
	for (int i = 0; i < 3; ++i) {
		// Update interval for _i_th bounding box slab
		float invRayDir = 1.f / dir[i];
		float tNear = (box.min[i] - ori[i]) * invRayDir;
		float tFar  = (box.max[i] - ori[i]) * invRayDir;

		// Update parametric interval from slab intersection $t$s
		if (tNear > tFar) std::swap(tNear, tFar);
		t0 = tNear > t0 ? tNear : t0;
		t1 = tFar  < t1 ? tFar  : t1;
		if (t0 > t1) return false;
	}
	if (hitt0) *hitt0 = t0;
	if (hitt1) *hitt1 = t1;
	return true;
}


Ray* Ray::BuildRd(TVec2d fragCoord,osg::Camera* cam, OGRLineString* Line)
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
	dir = rot * dir ;

	dir.normalize();

	TVec3d rayori(ori.x(), ori.y(), ori.z());
	TVec3d raydir(dir.x(), dir.y(), dir.z());

	Line->addPoint(rayori.x, rayori.y, rayori.z);
	Line->addPoint(rayori.x + 1000 * raydir.x, rayori.y + 1000 * raydir.y, rayori.z + 1000 * raydir.z);

	Ray* ray = new Ray(rayori,raydir);
	ray->fragCoord = fragCoord;

	return ray;
}

float Ray::DotCross(TVec3d v0, TVec3d v1, TVec3d v2)
{
	return v0.dot( v1.cross(v2));
}

TVec3d Ray::Normalized(TVec3d vec)
{
	return vec/vec.length();
}

osg::Vec3 Rotation(osg::Vec3 d, osg::Vec3 u, double angle) //Rotation d'angle de V autour de u
{
	osg::Vec3 Res;
	u.normalize();
	double C = cos(osg::DegreesToRadians(angle));
	double S = sin(osg::DegreesToRadians(angle));

	double x = (u.x()*u.x()*(1-C) + C)*d.x() + (u.x()*u.y()*(1-C) - u.z()*S)*d.y() + (u.x()*u.z()*(1-C) + u.y()*S)*d.z();
	double y = (u.x()*u.y()*(1-C) + u.z()*S)*d.x() + (u.y()*u.y()*(1-C) + C)*d.y() + (u.y()*u.z()*(1-C) - u.x()*S)*d.z();
	double z = (u.x()*u.z()*(1-C) - u.y()*S)*d.x() + (u.y()*u.z()*(1-C) + u.x()*S)*d.y() + (u.z()*u.z()*(1-C) + C)*d.z();

	Res = osg::Vec3(x, y, z);

	Res.normalize();

	return Res;
}

RayCollection* BuildCollectionOld(osg::Camera* cam)
{
	OGRMultiLineString* MLS = new OGRMultiLineString;
	RayCollection* rays = new RayCollection();
	for(unsigned int i = 0; i < cam->getViewport()->width(); i++)
	{
		for(unsigned int j = 0; j < cam->getViewport()->height(); j++)
		{
			OGRLineString* Line = new OGRLineString;
			Ray* ray = Ray::BuildRd(TVec2d(i,j), cam, Line);
			ray->collection = rays;
			rays->rays.push_back(ray);
			MLS->addGeometry(Line);
		}
	}

	SaveGeometrytoShape("RayonsOld.shp", MLS);

	delete MLS;

	return rays;
}
RayCollection* RayCollection::BuildCollection(osg::Camera* cam)
{
	//BuildCollectionOld(cam);

	OGRMultiLineString* MLS = new OGRMultiLineString;
	RayCollection* rays = new RayCollection();

	double fovx;
	double aspect;
	double znear;
	double zfar;
	cam->getProjectionMatrixAsPerspective(fovx,aspect,znear,zfar);

	osg::Viewport* viewport = cam->getViewport();

	float width = viewport->width();
	float height = viewport->height();

	double fovy = (height/width)*fovx;

	std::cout << fovx << " " << fovy << std::endl;

	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);
	osg::Vec3d right = target ^ up;

	TVec3d rayori(pos.x(), pos.y(), pos.z());

	for(int i = - width/2; i <= width/2; ++i)
	{
		osg::Vec3d RayX = Rotation(target, up, - i * fovx/width);
		right = RayX ^ up;
		for(int j = - height/2; j <= height/2; ++j)
		{
			osg::Vec3d NewRay = Rotation(RayX, right, j * fovy/height);

			TVec3d raydir(NewRay.x(), NewRay.y(), NewRay.z());

			Ray* ray = new Ray(rayori, raydir);
			TVec2d fragCoord = TVec2d(i + width/2, j + height/2);
			ray->fragCoord = fragCoord;
			ray->collection = rays;
			rays->rays.push_back(ray);

			OGRLineString* Line = new OGRLineString;
			Line->addPoint(rayori.x, rayori.y, rayori.z);
			Line->addPoint(rayori.x + 1000 * raydir.x, rayori.y + 1000 * raydir.y, rayori.z + 1000 * raydir.z);
			MLS->addGeometry(Line);
		}
	}

	SaveGeometrytoShape("Rayons.shp", MLS);

	int a;
	std::cin >> a;

	return rays;
}