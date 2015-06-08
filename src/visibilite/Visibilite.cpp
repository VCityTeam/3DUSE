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

std::vector<TVec3d> Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam, bool useSkipMiscBuilding, std::string exportFilePrefix)
{
	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,256);

	ViewPoint result(cam->getViewport()->width(),cam->getViewport()->height());

	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
	TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

	result.lightDir = Ray::Normalized(camPos - camDir);

	std::cout << "Loading Building Scene." << std::endl;
	vcity::Tile* tile = new vcity::Tile(buildingPath);

	//Get the triangle list
	std::vector<Triangle*> triangles = BuildBuildingTriangleList(tile,offset,&result,useSkipMiscBuilding);

	RayTracing(triangles,&result,offset,cam);

	vcity::Tile* tileT;
	std::vector<Triangle*> trianglesT;

	if(terrainPath != "")
	{
		std::cout << "Loading Terrain Scene." << std::endl;
		tileT = new vcity::Tile(terrainPath);

		//Get the triangle list
		trianglesT = BuildTerrainTriangleList(tileT,offset,&result);

		RayTracing(trianglesT,&result,offset,cam);
	}

	result.ComputeSkyline();
	ExportData(&result,exportFilePrefix);
	ExportImages(&result,exportFilePrefix);

	std::vector<TVec3d> skylineReturn;

	for(unsigned int i = 0; i < result.skyline.size(); i++)
	{
		Hit h = result.hits[result.skyline[i].first][result.skyline[i].second];
		skylineReturn.push_back(h.point);
	}


	for(unsigned int i = 0; i != triangles.size(); i++)
		delete triangles[i];

	delete tile;

	if(terrainPath != "")
	{
		for(unsigned int i = 0; i != trianglesT.size(); i++)
			delete trianglesT[i];

		delete tileT;
	}

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return skylineReturn;
}

void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement)
{
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	osg::Vec3d dir = target - pos;
	dir.z() = 0;
	dir.normalize();
	target = pos + dir;
	up = osg::Vec3d(0,0,1);
	cam->setViewMatrixAsLookAt(pos,target,up);

	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,256);

	ViewPoint result(cam->getViewport()->width(),cam->getViewport()->height());

	TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
	TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

	result.lightDir = Ray::Normalized(camPos - camDir);

	std::cout << "Loading Building Scene." << std::endl;
	vcity::Tile* tile = new vcity::Tile(buildingPath);

	//Get the triangle list
	std::vector<Triangle*> triangles = BuildBuildingTriangleList(tile,offset,&result,false);

	vcity::Tile* tileT;
	std::vector<Triangle*> trianglesT;

	if(terrainPath != "")
	{
		std::cout << "Loading Terrain Scene." << std::endl;
		tileT = new vcity::Tile(terrainPath);

		//Get the triangle list
		trianglesT = BuildTerrainTriangleList(tileT,offset,&result);
	}

	for(unsigned int i = 0; i < count; i++)
	{
		RayTracing(triangles,&result,offset,cam);
		if(terrainPath != "")
			RayTracing(trianglesT,&result,offset,cam);

		result.ComputeSkyline();
		ExportData(&result,std::to_string(i)+"_");
		ExportImages(&result,std::to_string(i)+"_");

		result.Reset();

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}

	for(unsigned int i = 0; i != triangles.size(); i++)
		delete triangles[i];

	delete tile;

	if(terrainPath != "")
	{
		for(unsigned int i = 0; i != trianglesT.size(); i++)
			delete trianglesT[i];

		delete tileT;
	}

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);
}

void Analyse(std::string buildingPath, std::string terrainPath, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
{
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	osg::Vec3d dir = target - pos;
	dir.z() = 0;
	dir.normalize();
	target = pos + dir;
	up = osg::Vec3d(0,0,1);
	cam->setViewMatrixAsLookAt(pos,target,up);

	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,256);

	ViewPoint result(cam->getViewport()->width(),cam->getViewport()->height());

	TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
	TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

	result.lightDir = Ray::Normalized(camPos - camDir);

	std::cout << "Loading Building Scene." << std::endl;
	vcity::Tile* tile = new vcity::Tile(buildingPath);

	//Get the triangle list
	std::vector<Triangle*> triangles = BuildBuildingTriangleList(tile,offset,&result,false);

	vcity::Tile* tileT;
	std::vector<Triangle*> trianglesT;

	if(terrainPath != "")
	{
		std::cout << "Loading Terrain Scene." << std::endl;
		tileT = new vcity::Tile(terrainPath);

		//Get the triangle list
		trianglesT = BuildTerrainTriangleList(tileT,offset,&result);
	}

	for(unsigned int i = 0; i < viewpoints.size(); i++)
	{
		pos = osg::Vec3d(viewpoints[i].first.x,viewpoints[i].first.y,viewpoints[i].first.z);
		target = osg::Vec3d(viewpoints[i].second.x,viewpoints[i].second.y,viewpoints[i].second.z);
		cam->setViewMatrixAsLookAt(pos,target,up);

		RayTracing(triangles,&result,offset,cam);
		if(terrainPath != "")
			RayTracing(trianglesT,&result,offset,cam);

		result.ComputeSkyline();
		ExportData(&result,std::to_string(i)+"_");
		ExportImages(&result,std::to_string(i)+"_");

		result.Reset();
	}

	for(unsigned int i = 0; i != triangles.size(); i++)
		delete triangles[i];

	delete tile;

	if(terrainPath != "")
	{
		for(unsigned int i = 0; i != trianglesT.size(); i++)
			delete trianglesT[i];

		delete tileT;
	}

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);
}

struct RayTracingData
{
	ViewPoint* result;
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

void RayTracing(std::vector<Triangle*> triangles, ViewPoint* viewpoint, TVec3d offset, osg::Camera* cam)
{
	QTime time;
	time.start();

	unsigned int tCount = std::thread::hardware_concurrency();//Get how many thread we have

	//List of rays and their frag coord
	std::vector<unsigned int>* toDo = new std::vector<unsigned int>[tCount];//List of rays for each threads

	unsigned int current = 0;//At which thread we are going to add a ray

	for(unsigned int i = 0; i < viewpoint->width; i++)
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
		minDistance[i] = viewpoint->minDistance;
		maxDistance[i] = viewpoint->maxDistance;
		RayTracingData data;
		data.result = viewpoint;
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
		viewpoint->minDistance = std::min(viewpoint->minDistance,minDistance[i]);
		viewpoint->maxDistance = std::max(viewpoint->maxDistance,maxDistance[i]);
		delete threads[i];
	}

	std::cout << "The joining is completed" << std::endl;

	delete[] minDistance;
	delete[] maxDistance;
	delete[] toDo;

	std::cout << "Time : " << time.elapsed()/1000 << " sec" << std::endl;
}


std::vector<Triangle*> BuildBuildingTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint, bool ignoreMiscBuilding)
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
			viewpoint->objectToColor.insert(pair);

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
							t->subObjectType = object->getType();
							t->objectType = obj->getType();
							t->objectId = obj->getId();



							triangles.push_back(t);
						}
					}
				}
			}
		}
	}	

	return triangles;
}

std::vector<Triangle*> BuildTerrainTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint)
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
			viewpoint->objectToColor.insert(pair);
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
						t->objectType = obj->getType();
						t->objectId = obj->getId();

						triangles.push_back(t);
					}
				}

			}
		}
	}

	return triangles;
}

osg::ref_ptr<osg::Geode> BuildSkylineOSGNode(std::vector<TVec3d> skyline)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

	for(unsigned int i = 0; i < skyline.size(); i++)
	{
		vertices->push_back(osg::Vec3(skyline[i].x,skyline[i].y,skyline[i].z));
	}

	for(unsigned int i = 0; i < skyline.size()-1; i++)
	{
		indices->push_back(i); indices->push_back(i+1);
	}

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geode->addDrawable(geom);

    return geode;
}

std::pair<int, int> ViewPoint::GetCoord(Position p)
{
	switch (p)
	{
	case W:
		return std::make_pair(-1,0);
		break;
	case NW:
		return std::make_pair(-1,1);
		break;
	case N:
		return std::make_pair(0,1);
		break;
	case NE:
		return std::make_pair(1,1);
		break;
	case E:
		return std::make_pair(1,0);
		break;
	case SE:
		return std::make_pair(1,-1);
		break;
	case S:
		return std::make_pair(0,-1);
		break;
	case SW:
		return std::make_pair(-1,-1);
		break;
	default:
		throw std::exception("Mega Error, pos not in enum range !");
		break;
	}
}

inline unsigned int ViewPoint::Clamp(unsigned int x,unsigned int a,unsigned int b)
{
	return x < a ? a : (x > b ? b : x);
}

void ViewPoint::ComputeSkyline()
{
	bool found = false;

	unsigned int x = 0;
	unsigned int y = 0;

	for(unsigned int i = 0; i < width; i++)
	{

		for(int j = height - 1; j >= 0; j--)
		{
			if(hits[i][j].intersect)
			{
				found = true;
				x = i;
				y = j;
				skyline.push_back(std::make_pair(i,j));
				break;
			}
		}
		if(found)
			break;
	}

	Position pos = N;
	std::pair<int, int> c = GetCoord(pos);

	unsigned int realX = Clamp(x+c.first,0,width);
	unsigned int realY = Clamp(y+c.second,0,height);

	while(x < width - 1)
	{
		while(realY >= height || (realY < height && !hits[realX][realY].intersect))
		{
			int posInt = int(pos);
			posInt++;
			posInt = posInt % 8;
			pos = Position(posInt);
			c = GetCoord(pos);
			realX = x+c.first,0,width;
			realY = y+c.second,0,height;
		}

		x = realX;
		y = realY;

		if(x == skyline.front().first && y == skyline.front().second)
			break;

		skyline.push_back(std::make_pair(x,y));

		int posInt = int(pos);
		posInt += 5;
		posInt = posInt % 8;
		pos = Position(posInt);

		c = GetCoord(pos);

		realX = Clamp(x+c.first,0,width);
		realY = Clamp(y+c.second,0,height);
	}
}

void ViewPoint::Reset()
{
	minDistance = FLT_MAX;
	maxDistance = FLT_MIN;

	skyline.clear();

	for(unsigned int i = 0; i < width; i++)
	{
		for(unsigned int j = 0; j < height; j++)
		{
			hits[i][j] = Hit();
		}
	}
}

