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

void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam)
{
	RayTracingResult result;

	GlobalData globalData;

	if(buildingPath != "")
	{
		vcity::Tile* tile = new vcity::Tile(buildingPath);

		//Get the triangle list
		std::vector<Triangle*> triangles = BuildBuildingTriangleList(tile,offset,&globalData,false);

		RayTracingResult result = RayTracing(triangles,&globalData,offset,cam);

		ExportImages(&globalData,result);
		ExportData(&globalData,result);

		for(unsigned int i = 0; i < result.width; i++)
		{
			delete[] result.hits[i];
		}
		delete[] result.hits;

		for(unsigned int i = 0; i != triangles.size(); i++)
			delete triangles[i];

		delete tile;
	}
}

struct RayTracingData
{
	Hit** result;
	float* minDistance; 
	float* maxDistance; 
	std::vector<Triangle*>* triangles; 
	std::vector<RayFragCoord>* rowToDo;
};

//Loop through all triangles and check if any rays intersect with triangles
void RayLoop(RayTracingData data)
{
	for(unsigned int k = 0; k < data.rowToDo->size(); k++)
	{
		Ray ray = data.rowToDo->at(k).first;
		unsigned int i = data.rowToDo->at(k).second.first;
		unsigned int j = data.rowToDo->at(k).second.second;
		for(unsigned int l = 0; l < data.triangles->size(); l++)
		{
			Triangle* tri = data.triangles->at(l);

			Hit hit;
			if(ray.Intersect(tri,&hit))//Check if the ray hit the triangle and
			{
				if(!data.result[i][j].intersect || data.result[i][j].distance > hit.distance)//Check if it is closer than the previous one
				{
					data.result[i][j] = hit;
					*data.maxDistance = std::max(hit.distance,*data.maxDistance);
					*data.minDistance = std::min(hit.distance,*data.minDistance);
				}
			}
		}
	}
}

RayTracingResult RayTracing(std::vector<Triangle*> triangles, GlobalData* globalData, TVec3d offset, osg::Camera* cam)
{
	//=========<Camera Initialization>=========
	unsigned int WWIDTH;
	unsigned int WHEIGHT;

	TVec3d camPos;
	TVec3d camDir;
	TVec3d camUp;

	if(cam == nullptr)
	{
		camPos = TVec3d(4964.19,6961.07,243.893);
		camDir = TVec3d(4965.14,6960.9,243.619);
		camUp = TVec3d(0.272763,-0.0326287,-0.961528);

		WWIDTH = 500;
		WHEIGHT = 500;
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

		WWIDTH = cam->getViewport()->width();
		WHEIGHT = cam->getViewport()->height();
	}
	//=========</Camera Initialization>=========


	Hit** result = new Hit*[WWIDTH];

	QTime time;
	time.start();

	unsigned int tCount = std::thread::hardware_concurrency();//Get how many thread we have

	//List of rays and their frag coord
	std::vector<RayFragCoord>* toDo = new std::vector<RayFragCoord>[tCount];//List of rays for each threads

	unsigned int current = 0;//At which thread we are going to add a ray

	for(unsigned int i = 0; i < WWIDTH; i++)
	{
		result[i] = new Hit[WHEIGHT];
		for(unsigned int j = 0; j < WHEIGHT; j++)//Foreach pixel on our image
		{
			toDo[current].push_back(GetRayFragCoord(Ray::BuildRd(TVec2d(i,j),cam),i,j));//Add a ray to a thread list
			current++;
			current = current % tCount;
		}
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

	RayTracingResult rtResult;
	rtResult.width = WWIDTH;
	rtResult.height = WHEIGHT;
	rtResult.hits = result;
	rtResult.lightDir = Ray::Normalized(  camPos - camDir);

	return rtResult;
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