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

bool RayTriangleIntersect(Ray ray, Triangle triangle)
{
    bool result;

    // Compute the offset origin, edges, and normal.
    TVec3d diff = ray.ori - triangle.a;
    TVec3d edge1 = triangle.b - triangle.a;
    TVec3d edge2 = triangle.c - triangle.a;
    TVec3d normal = edge1.cross(edge2);

    // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
    // E1 = edge1, E2 = edge2, N = Cross(E1,E2)) by
    //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
    //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
    //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
    float DdN = ray.dir.dot(normal);
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

    float DdQxE2 = sign*DotCross(ray.dir, diff, edge2);
    if (DdQxE2 >= (float)0)
    {
        float DdE1xQ = sign*DotCross(ray.dir, edge1, diff);
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

TVec3d Normalized(TVec3d vec)
{
	return vec/vec.length();
}

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
}

#define WWIDTH 500
#define WHEIGHT 500

void RayTracing(std::string path, TVec3d offset,osg::Camera* cam)
{
	TVec3d camPos;
	TVec3d camDir;
	TVec3d camUp;

	if(cam == nullptr)
	{
		camPos = TVec3d(4964.19+offset.x,6961.07+offset.y,243.893+offset.z);
		camDir = TVec3d(4965.14+offset.x,6960.9+offset.y,243.619+offset.z);
		camUp = TVec3d(0.272763,-0.0326287,-0.961528);
	}
	else
	{
		osg::Vec3d pos;
		osg::Vec3d target;
		osg::Vec3d up;
		cam->getViewMatrixAsLookAt(pos,target,up);

		camPos = TVec3d(pos.x()+offset.x,pos.y()+offset.y,pos.z()+offset.z);
		camDir = TVec3d(target.x()+offset.x,target.y()+offset.y,target.z()+offset.z);
		camUp = TVec3d(up.x(),up.y(),-up.z());
	}

	std::vector<Triangle> triangles;
	

	float maxz = 0.0;
	float miny = FLT_MAX;
	float minx = FLT_MAX;
	float maxx = -FLT_MAX;

	unsigned int polyCPt = 0;

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
						if(PolygonCityGML->getExteriorRing()->getVertices().size() != 3)
						{
							polyCPt++;
							continue;
						}

						std::vector<TVec3d> list = PolygonCityGML->getExteriorRing()->getVertices();

						for(TVec3d v : list)
						{
							maxz = std::max(maxz,float(v.z));
							miny = std::min(miny,float(v.y));
							minx = std::min(minx,float(v.x));
							maxx = std::max(maxx,float(v.x));
						}

						Triangle t(list[0],list[1],list[2]);
						t.polygon = PolygonCityGML;
						t.geometry = Geometry;
						t.object = object;
						t.parent = obj;

						triangles.push_back(t);
					}
				}
			}
		}
}	

	delete tile;

	std::cout << "Poly cpt : " << polyCPt << std::endl;
	
	Triangle*** result = new Triangle**[WWIDTH];
	for(unsigned int i = 0; i < WWIDTH; i++)
	{
		std::cout << i << std::endl;
		result[i] = new Triangle*[WHEIGHT];
		for(unsigned int j = 0; j < WHEIGHT; j++)
		{
			TVec3d dir = BuildRd(TVec2d(i,j),WWIDTH,WHEIGHT,camPos,camDir,camUp);

			Ray ray(camPos,dir);

			bool intersect = false;

			result[i][j] = nullptr;

			for(Triangle& tri : triangles)
			{
				if(RayTriangleIntersect(ray,tri))
				{
					result[i][j] = &tri;
					intersect = true;
					break;
				}
			}

		}
	}
	
	QImage image(WWIDTH,WHEIGHT,QImage::Format::Format_RGB888);
	for(unsigned int i = 0; i < WWIDTH; i++)
	{
		for(unsigned int j = 0; j < WHEIGHT; j++)
		{
			if(result[i][j] == nullptr)
			{
				image.setPixel(i,j,qRgb(0,0,0));
			}
			else if (result[i][j]->object->getType() == citygml::COT_RoofSurface)
			{
				image.setPixel(i,j,qRgb(255,0,0));
			}
			else
			{
				image.setPixel(i,j,qRgb(255,255,255));
			}
		}
	}
	image.save("./SkylineOutput/raytrace.png");

	delete[] result;
	std::cout << "Image saved !" << std::endl;
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