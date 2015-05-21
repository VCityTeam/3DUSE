#include "visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"
#include "src/processes/ExportToShape.hpp"

#include <QDir>
#include <QImage>

float DotCross(TVec3d v0, TVec3d v1,
			   TVec3d v2)
{
	return v0.dot( v1.cross(v2));
}

TVec3d Normalized(TVec3d vec)
{
	return vec/vec.length();
}
/*
TVec3d BuildRd(TVec2d fragCoord,float width, float height, TVec3d camPos, TVec3d camTarget, TVec3d camUp)
{
TVec3d ro = camPos;
TVec3d ta = camTarget;

// Calculate orthonormal camera reference system
TVec3d camDir   = Normalized(ta-ro); // direction for center ray
TVec3d camRight = Normalized(camDir.cross(camUp));

TVec2d coord = TVec2d((fragCoord[0]/width*2)-1.0,(fragCoord[1]/height*2)-1.0);
coord[0] *= width/height;

// Get direction for this pixel
TVec3d rd = Normalized(camDir + (camRight*coord[0] + camUp*coord[1])) ;

return rd;
}*/

Ray BuildRd(TVec2d fragCoord,osg::Camera* cam)
{
	osg::Vec3f temppos;
	osg::Vec3f temptarget;
	osg::Vec3f tempup;

	cam->getViewMatrixAsLookAt(temppos,temptarget,tempup);

	TVec3d camPos(temppos.x(),temppos.y(),temppos.z());
	TVec3d camTarget(temptarget.x(),temptarget.y(),temptarget.z());
	TVec3d camUp(tempup.x(),tempup.y(),tempup.z());

	TVec3d ro = camPos;
	TVec3d ta = camTarget;

	// Calculate orthonormal camera reference system
	TVec3d camDir   = Normalized(ta-ro); // direction for center ray
	TVec3d camRight = Normalized(camDir.cross(camUp));

	osg::Viewport* viewport = cam->getViewport();
	int width = viewport->width();
	int height = viewport->height();

	TVec2d coord = TVec2d((fragCoord[0]/width*2)-1.0,(fragCoord[1]/height*2)-1.0);
	coord[0] *= width/height;

	// Get direction for this pixel
	TVec3d rd = Normalized(camDir + (camRight*coord[0] + camUp*coord[1]));

	return Ray(camPos,rd);
}

Ray BuildRdOldNot(TVec2d fragCoord,osg::Camera* cam)
{
	osg::Vec3f origine(fragCoord.x,fragCoord.y,0.0);
	osg::Vec3f extremite(origine.x(), origine.y(), 1.f);

	osg::Viewport* viewport = cam->getViewport();

	osg::Matrixd viewportMatrix(
		viewport->width(), 0,   0,   viewport->width(),
		0, viewport->height(),   0,   viewport->height(),
		0, 0, .5f, .5f,
		0, 0,   0,   1
		);

	/*osg::Matrixd viewportMatrix(
	viewport->width(), 0,   0,   0,
	0, viewport->height(),   0,   0,
	0, 0, .5f, 0,
	viewport->width(), viewport->height(),   .5f,   1
	);*/

	osg::Matrixd mvpi= cam->getViewMatrix()*cam->getProjectionMatrix()*viewportMatrix;

	osg::Vec3d o = origine * mvpi.inverse(mvpi);
	osg::Vec3d e = extremite * mvpi.inverse(mvpi);

	return Ray(TVec3d(o.x(),o.y(),o.z()), Normalized(TVec3d(e.x(),e.y(),e.z())));
}


Ray BuildRdOld(TVec2d fragCoord,osg::Camera* cam)
{
	double fov;
	double aspect;
	double znear;
	double zfar;
	cam->getProjectionMatrixAsPerspective(fov,aspect,znear,zfar);

	osg::Vec3f temppos;
	osg::Vec3f temptarget;
	osg::Vec3f tempup;

	cam->getViewMatrixAsLookAt(temppos,temptarget,tempup);

	osg::Viewport* viewport = cam->getViewport();

	fov = osg::DegreesToRadians(fov);

	float dx;
	float dy;
	dx = tanf(fov*0.5f)*(fragCoord.x/(viewport->width()/2)-1.0f)/aspect;
	dy = tanf(fov*0.5f)*(1.0f-fragCoord.y/(viewport->height()/2));

	osg::Vec3f p1(dx*znear,dy*znear,znear);
	osg::Vec3f p2(dx*zfar,dy*zfar,zfar);

	p1 = p1 * cam->getInverseViewMatrix();
	p2 = p2 * cam->getInverseViewMatrix();

	osg::Vec3f dir = p2-p1;

	TVec3d finalDir(dir.x(),dir.y(),dir.z());
	TVec3d pos(temppos.x(),temppos.y(),temppos.z());
	return Ray(pos, Normalized(finalDir));
}
/*
#define WWIDTH 500
#define WHEIGHT 500*/

void RayTracing(std::string path, TVec3d offset,osg::Camera* cam)
{
	int WWIDTH = cam->getViewport()->width();
	int WHEIGHT = cam->getViewport()->height();

	TVec3d camPos;
	TVec3d camDir;
	TVec3d camUp;

	if(cam == nullptr)
	{
		camPos = TVec3d(4964.19,6961.07,243.893);
		camDir = TVec3d(4965.14,6960.9,243.619);
		camUp = TVec3d(0.272763,-0.0326287,-0.961528);
	}
	else
	{
		osg::Vec3d pos;
		osg::Vec3d target;
		osg::Vec3d up;
		cam->getViewMatrixAsLookAt(pos,target,up);

		camPos = TVec3d(pos.x(),pos.y(),pos.z());
		camDir = TVec3d(target.x(),target.y(),target.z());
		camUp = TVec3d(up.x(),up.y(),-up.z());
	}

	std::vector<Triangle*> triangles = BuildTriangleList(path,offset);

	Hit** result = new Hit*[WWIDTH];
	GlobalData globalData;
	for(unsigned int i = 0; i < WWIDTH; i++)
	{
		std::cout << i << std::endl;
		result[i] = new Hit[WHEIGHT];
		for(unsigned int j = 0; j < WHEIGHT; j++)
		{
			Ray ray = BuildRd(TVec2d(i,j),cam);

					bool intersect = false;

			for(Triangle* tri : triangles)
			{
				Hit hit;
				if(ray.Intersect(tri,&hit))
				{
					if(!result[i][j].intersect || result[i][j].distance > hit.distance)
					{
						result[i][j] = hit;
						globalData.maxDistance = std::max(hit.distance,globalData.maxDistance);
						globalData.minDistance = std::min(hit.distance,globalData.minDistance);
					}
				}
			}

		}
	}

	QImage imageMurToit(WWIDTH,WHEIGHT,QImage::Format::Format_ARGB32);
	QImage imageZBuffer(WWIDTH,WHEIGHT,QImage::Format::Format_ARGB32);
	TVec3d light1 = Normalized(  camPos - camDir);
	for(unsigned int i = 0; i < WWIDTH; i++)
	{
		for(unsigned int j = 0; j < WHEIGHT; j++)
		{
			if(!result[i][j].intersect)
			{
				imageMurToit.setPixel(i,j,qRgba(0,0,0,0));
				imageZBuffer.setPixel(i,j,qRgba(255,255,255,0));
			}
			else 
			{
				TVec3d v1 = result[i][j].triangle->a;
				TVec3d v2 = result[i][j].triangle->b;
				TVec3d v3 = result[i][j].triangle->c;
				//Get its normal
				TVec3d normal = Normalized((v2 - v1).cross(v3 - v1));

				float sundot = (normal.dot(light1) + 1.0)/2.0;

				if (result[i][j].triangle->object->getType() == citygml::COT_RoofSurface)
				{
					imageMurToit.setPixel(i,j,qRgba(255*sundot,0,0,255));
				}
				else
				{
					imageMurToit.setPixel(i,j,qRgba(205*sundot,183*sundot,158*sundot,255));
				}

				float factor = (globalData.maxDistance - result[i][j].distance)/(globalData.maxDistance - globalData.minDistance);
				factor *= factor;
				imageZBuffer.setPixel(i,j,qRgba(217 * factor, 1 * factor, 21 * factor, 255));
			}
		}
	}
	imageMurToit = imageMurToit.mirrored(false,true);
	imageZBuffer = imageZBuffer.mirrored(false,true);
	imageMurToit.save("./SkylineOutput/raytraceMurToit.png");
	imageZBuffer.save("./SkylineOutput/raytraceZBuffer.png");

	delete[] result;
	std::cout << "Image saved !" << std::endl;

	for(unsigned int i = 0; i != triangles.size(); i++)
		delete triangles[i];
}

std::vector<Triangle*> BuildTriangleList(std::string path, TVec3d offset)
{
	std::vector<Triangle*> triangles;

	vcity::Tile* tile = new vcity::Tile(path);
	citygml::CityModel * model = tile->getCityModel();
	unsigned int i = 0;
	for(citygml::CityObject* obj : model->getCityObjectsRoots())
	{
		if(obj->getType() == citygml::COT_Building)
		{
			OGRMultiPolygon* Building = new OGRMultiPolygon;//Version OGR du bâtiment qui va être remplie

			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
			{
				for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
				{
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
					{
						const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
						const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

						for(unsigned int i = 0 ; i < ind.size() / 3; i++)
						{
							TVec3d a = vert[ind[ i * 3 + 0 ]] - offset;
							TVec3d b = vert[ind[ i * 3 + 1 ]] - offset;
							TVec3d c = vert[ind[ i * 3 + 2 ]] - offset;

							Triangle* t = new Triangle(a,b,c);
							t->polygon = PolygonCityGML;
							t->geometry = Geometry;
							t->object = object;
							t->parent = obj;

							triangles.push_back(t);
						}
					}
				}
			}
		}
	}	

	delete tile;
	return triangles;
}

void DetectionToitsPlats(std::string path, float minArea, float slopeFactor)
{
	vcity::Tile* tile = new vcity::Tile(path);
	citygml::CityModel * model = tile->getCityModel();

	OGRMultiPolygon* toitsPlats = new OGRMultiPolygon;//Carte contenant uniquement les toits plats
	OGRMultiPolygon* toits = new OGRMultiPolygon;//Carte de tous les toits.

	for(citygml::CityObject* obj : model->getCityObjectsRoots())
	{
		if(obj->getType() == citygml::COT_Building)
		{
			OGRMultiPolygon* Building = new OGRMultiPolygon;//Version OGR du bâtiment qui va être remplie

			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
			{
				if(object->getType() == citygml::COT_RoofSurface)
				{
					for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
						{
							const std::vector<TVec3f>& norms = PolygonCityGML->getNormals();

							OGRPolygon * OgrPoly = new OGRPolygon;
							OGRLinearRing * OgrRing = new OGRLinearRing;

							for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
							{
								OgrRing->addPoint(Point.x, Point.y, Point.z);
							}

							OgrRing->closeRings();

							if(OgrRing->getNumPoints() > 3)//Vérification qu'on a bien un triangle
							{
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid())//Vérification que notre triangle n'est pas dégénéré
								{
									toits->addGeometryDirectly(OgrPoly);
									TVec3f norm = norms.at(0);
									if(norm.z >= slopeFactor)//On test la normal à notre triangle
									{
										toitsPlats->addGeometry(OgrPoly);
									}
								}
							}
						}
					}
				}
			}
		}
	}
	OGRGeometry* toitsGobalUnion = toits->UnionCascaded();
	OGRGeometry* Union = toitsPlats->UnionCascaded();
	OGRGeometryCollection* UnionMP = (OGRGeometryCollection*)Union;
	OGRMultiPolygon* ToitsRes = new OGRMultiPolygon;
	for(int i = 0; i < UnionMP->getNumGeometries(); ++i)
	{
		OGRPolygon* poly = dynamic_cast<OGRPolygon*>(UnionMP->getGeometryRef(i));
		if(poly == nullptr)
			continue;
		if(poly->get_Area() > minArea)
			ToitsRes->addGeometry(poly);
	}

	QDir outputDir("./SkylineOutput/");
	if(!outputDir.exists("./SkylineOutput/"))
	{
		outputDir.mkpath(outputDir.absolutePath());
	}

	SaveGeometrytoShape("SkylineOutput/Toits.shp", toits);
	SaveGeometrytoShape("SkylineOutput/ToitsUnion.shp", toitsGobalUnion);
	SaveGeometrytoShape("SkylineOutput/ToitsPlats.shp", toitsPlats);
	SaveGeometrytoShape("SkylineOutput/ToitsPlatsUnion.shp", Union);
	SaveGeometrytoShape("SkylineOutput/ToitsPlatsUnionFiltre.shp", ToitsRes);

	delete UnionMP;
	delete ToitsRes;
	delete toits;
	delete toitsPlats;
	delete tile;
}

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
					tempHit.triangle = triangle;

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