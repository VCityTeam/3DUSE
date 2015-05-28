#include "Visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"

#include <QImage>
#include <QColor>

#include <thread>

#include "Hit.hpp"
#include "Export.hpp"

typedef std::pair<Ray,std::pair<unsigned int,unsigned int>> RayFragCoord;

inline RayFragCoord GetRayFragCoord(Ray ray,unsigned int i, unsigned int j)
{
	return std::make_pair(ray, std::make_pair(i,j));
}

int RandInt(int low, int high)
{
	// Random number between low and high
	return qrand() % ((high + 1) - low) + low;
}

void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam, bool useSkipMiscBuilding)
{
	RayTracingResult result(cam->getViewport()->width(),cam->getViewport()->height());

	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
	TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

	result.lightDir = Ray::Normalized(camPos - camDir);

	GlobalData globalData;

	std::cout << "Loading Building Scene." << std::endl;
	vcity::Tile* tile = new vcity::Tile(buildingPath);

	//Get the triangle list
	std::vector<Triangle*> triangles = BuildBuildingTriangleList(tile,offset,&globalData,useSkipMiscBuilding);

	RayTracing(triangles,&globalData,offset,cam,&result);

	vcity::Tile* tileT;
	std::vector<Triangle*> trianglesT;

	if(terrainPath != "")
	{
		std::cout << "Loading Terrain Scene." << std::endl;
		tileT = new vcity::Tile(terrainPath);

		//Get the triangle list
		trianglesT = BuildTerrainTriangleList(tileT,offset,&globalData);

		RayTracing(trianglesT,&globalData,offset,cam,&result);
	}

	ExportImages(&globalData,&result,useSkipMiscBuilding ? "onlyremarquable_" : "");
	ExportData(&globalData,&result,useSkipMiscBuilding ? "onlyremarquable_" : "");

	for(unsigned int i = 0; i != triangles.size(); i++)
		delete triangles[i];

	delete tile;

	if(terrainPath != "")
	{
		for(unsigned int i = 0; i != trianglesT.size(); i++)
			delete trianglesT[i];

		delete tileT;
	}
}

struct RayTracingData
{
	RayTracingResult* result;
	float* minDistance; 
	float* maxDistance; 
	std::vector<Triangle*>* triangles; 
	osg::Camera* cam;
	std::vector<unsigned int>* rowToDo;
};

//Loop through all triangles and check if any rays intersect with triangles
void RayLoop(RayTracingData data)
{
	for(unsigned int k = 0; k < data.rowToDo->size(); k++)
	{
		unsigned int i = data.rowToDo->at(k);

		for(unsigned int j = 0; j < data.cam->getViewport()->height(); j++)
		{
			Ray ray = Ray::BuildRd(TVec2d(i,j),data.cam);
			for(unsigned int l = 0; l < data.triangles->size(); l++)
			{
				Triangle* tri = data.triangles->at(l);

				Hit hit;
				if(ray.Intersect(tri,&hit))//Check if the ray hit the triangle and
				{
					if(!data.result->hits[i][j].intersect || data.result->hits[i][j].distance > hit.distance)//Check if it is closer than the previous one
					{
						data.result->hits[i][j] = hit;
						*data.maxDistance = std::max(hit.distance,*data.maxDistance);
						*data.minDistance = std::min(hit.distance,*data.minDistance);
					}
				}
			}
		}
	}
}

void RayTracing(std::vector<Triangle*> triangles, GlobalData* globalData, TVec3d offset, osg::Camera* cam, RayTracingResult* result)
{
	QTime time;
	time.start();

	unsigned int tCount = std::thread::hardware_concurrency();//Get how many thread we have

	//List of rays and their frag coord
	std::vector<unsigned int>* toDo = new std::vector<unsigned int>[tCount];//List of rays for each threads

	unsigned int current = 0;//At which thread we are going to add a ray

	for(unsigned int i = 0; i < result->width; i++)
	{
		toDo[current].push_back(i);//Add a ray to a thread list
		current++;
		current = current % tCount;
	}	

	std::cout << "Thread : " << tCount << std::endl;

	float* minDistance = new float[tCount];//Where thread are going to write their minimum recorded distance for intersection
	float* maxDistance = new float[tCount];//Where thread are going to write their maximum recorded distance for intersection

	std::vector<std::thread*> threads;//Our thread list

	for(unsigned int i = 0; i < tCount; i++)
	{
		minDistance[i] = globalData->minDistance;
		maxDistance[i] = globalData->maxDistance;
		RayTracingData data;
		data.result = result;
		data.minDistance = &minDistance[i];
		data.maxDistance = &maxDistance[i];
		data.triangles = &triangles;
		data.rowToDo = &toDo[i];
		data.cam = cam;

		std::thread* t = new std::thread(RayLoop,data);
		threads.push_back(t);
	}

	std::cout << "Thread Launched " << std::endl;

	for(unsigned int i = 0; i < tCount; i++)//Join all our thread and update global data min and max distance
	{
		(*threads[i]).join();
		globalData->minDistance = std::min(globalData->minDistance,minDistance[i]);
		globalData->maxDistance = std::max(globalData->maxDistance,maxDistance[i]);
		delete threads[i];
	}

	std::cout << "The joining is completed" << std::endl;

	delete[] minDistance;
	delete[] maxDistance;
	delete[] toDo;

	std::cout << "Time : " << time.elapsed()/1000 << " sec" << std::endl;
}


std::vector<Triangle*> BuildBuildingTriangleList(vcity::Tile* tile, TVec3d offset, GlobalData* globalData, bool ignoreMiscBuilding)
{
	std::vector<Triangle*> triangles;

	citygml::CityModel * model = tile->getCityModel();
	unsigned int i = 0;
	for(citygml::CityObject* obj : model->getCityObjectsRoots()) //For each city object
	{
		if(obj->getType() == citygml::COT_Building ) //We only take building or terrain
		{
			QString tempStr(obj->getId().c_str());
			if(tempStr.startsWith("LYON") && ignoreMiscBuilding)
				continue;

			QColor objectColor = qRgba(RandInt(0,255),RandInt(0,255),RandInt(0,255),255);
			std::pair<std::string,QColor> pair(obj->getId(),objectColor);
			globalData->objectToColor.insert(pair);

			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
			{
				for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
				{
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
					{
						//Get triangle list
						const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
						const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

						for(unsigned int i = 0 ; i < ind.size() / 3; i++)//Push all triangle of the polygon in our list
						{
							TVec3d a = vert[ind[ i * 3 + 0 ]] - offset;
							TVec3d b = vert[ind[ i * 3 + 1 ]] - offset;
							TVec3d c = vert[ind[ i * 3 + 2 ]] - offset;

							Triangle* t = new Triangle(a,b,c);
							t->polygon = PolygonCityGML;
							t->geometry = Geometry;
							t->subObject = object;
							t->object = obj;

							triangles.push_back(t);
						}
					}
				}
			}
		}
	}	

	return triangles;
}

std::vector<Triangle*> BuildTerrainTriangleList(vcity::Tile* tile, TVec3d offset, GlobalData* globalData)
{
	std::vector<Triangle*> triangles;

	citygml::CityModel * model = tile->getCityModel();
	unsigned int i = 0;
	for(citygml::CityObject* obj : model->getCityObjectsRoots()) //For each city object
	{
		if(obj->getType() == citygml::COT_TINRelief ) //We only take building or terrain
		{
			QColor objectColor = qRgba(RandInt(0,255),RandInt(0,255),RandInt(0,255),255);
			std::pair<std::string,QColor> pair(obj->getId(),objectColor);
			globalData->objectToColor.insert(pair);
			for(citygml::Geometry* Geometry : obj->getGeometries()) //pour chaque géométrie
			{
				for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
				{
					//Get triangle list
					const std::vector<TVec3d>& vert = PolygonCityGML->getVertices();
					const std::vector<unsigned int>& ind = PolygonCityGML->getIndices();

					for(unsigned int i = 0 ; i < ind.size() / 3; i++)//Push all triangle of the polygon in our list
					{
						TVec3d a = vert[ind[ i * 3 + 0 ]] - offset;
						TVec3d b = vert[ind[ i * 3 + 1 ]] - offset;
						TVec3d c = vert[ind[ i * 3 + 2 ]] - offset;

						Triangle* t = new Triangle(a,b,c);
						t->polygon = PolygonCityGML;
						t->geometry = Geometry;
						t->object = obj;

						triangles.push_back(t);
					}
				}

			}
		}
	}

	return triangles;
}