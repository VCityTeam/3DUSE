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

std::vector<AnalysisResult> DoAnalyse(std::vector<osg::ref_ptr<osg::Camera>> cams,std::vector<std::string> buildingPath, std::vector<std::string> terrainPath, TVec3d offset)
{
	ViewPoint** result = new ViewPoint*[cams.size()];
	RayCollection** rays = new RayCollection*[cams.size()];

	std::vector<Ray*> allRays;

	for(unsigned int i = 0; i < cams.size(); i++)
	{
		osg::Camera* cam = cams[i];
		osg::Vec3d pos;
		osg::Vec3d target;
		osg::Vec3d up;
		cam->getViewMatrixAsLookAt(pos,target,up);

		TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
		TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

		result[i] = new ViewPoint(cam->getViewport()->width(),cam->getViewport()->height());
		result[i]->lightDir = Ray::Normalized(camPos - camDir);
		rays[i] = RayCollection::BuildCollection(cam);
		result[i]->rays = rays[i];

		rays[i]->viewpoint = result[i];

		allRays.insert(allRays.end(),rays[i]->rays.begin(),rays[i]->rays.end());
	}

	std::vector<TriangleList*> triangles;
	std::vector<TriangleList*> trianglesTerrain;

	std::cout << "Loading Building Scene." << std::endl;

	for(unsigned int i = 0; i < buildingPath.size(); i++)
	{
		vcity::Tile* tile = new vcity::Tile(buildingPath[i]);

		//Get the triangle list
		TriangleList* trianglesTemp = BuildBuildingTriangleList(tile,offset,result[0],false);
		triangles.push_back(trianglesTemp);
		delete tile;
	}

	
	std::cout << "Loading Terrain Scene." << std::endl;

	for(unsigned int i = 0; i < terrainPath.size(); i++)
	{
		vcity::Tile* tileT = new vcity::Tile(terrainPath[i]);
		TriangleList* trianglesT = BuildTerrainTriangleList(tileT,offset,result[0]);

		trianglesTerrain.push_back(trianglesT);

		delete tileT;
	}

	for(unsigned int i = 1; i < cams.size(); i++)
		result[i]->objectToColor = result[0]->objectToColor;

	
	for(unsigned int i = 0; i < triangles.size(); i++)
		RayTracing(triangles[i],allRays);
	for(unsigned int i = 0; i < trianglesTerrain.size(); i++)
		RayTracing(trianglesTerrain[i],allRays);

	std::vector<AnalysisResult> resReturn;

	for(unsigned int i = 0; i < cams.size(); i++)
	{
		result[i]->ComputeSkyline();
		result[i]->ComputeMinMaxDistance();
		ExportData(result[i],std::to_string(i)+"_");
		ExportImages(result[i],std::to_string(i)+"_");
		AnalysisResult temp;
		temp.viewpointPosition = rays[i]->rays.front()->ori;
		
		for(unsigned int j = 0; j < result[i]->skyline.size(); j++)
		{
			Hit h = result[i]->hits[result[i]->skyline[j].first][result[i]->skyline[j].second];
			temp.skyline.push_back(h.point);
		}

		resReturn.push_back(temp);
	}

	for(unsigned int i = 0; i < cams.size(); i++)
	{
		delete result[i];
		delete rays[i];
	}

	delete[] result;
	delete[] rays;

	for(unsigned int i = 0; i <  triangles.size();i++)
		delete triangles[i];

	for(unsigned int i = 0; i <  trianglesTerrain.size();i++)
		delete trianglesTerrain[i];

	return resReturn;
}

std::vector<AnalysisResult> Analyse(std::vector<std::string> buildingPath, std::vector<std::string> terrainPath, TVec3d offset,osg::Camera* cam)
{
	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,256);

	std::vector<osg::ref_ptr<osg::Camera>> temp;

	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	temp.push_back(mycam);

	std::vector<AnalysisResult> result = DoAnalyse(temp,buildingPath,terrainPath,offset);

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}




std::vector<AnalysisResult> Analyse(std::vector<std::string> buildingPath, std::vector<std::string> terrainPath, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement)
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

	std::vector<osg::ref_ptr<osg::Camera>> temp;

	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
		temp.push_back(mycam);

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}


	std::vector<AnalysisResult> result = DoAnalyse(temp,buildingPath,terrainPath,offset);


	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}

std::vector<AnalysisResult> Analyse(std::vector<std::string> buildingPath, std::vector<std::string> terrainPath, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
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
	std::vector<osg::ref_ptr<osg::Camera>> temp;

	for(unsigned int i = 0; i < viewpoints.size(); i++)
	{
		pos = osg::Vec3d(viewpoints[i].first.x,viewpoints[i].first.y,viewpoints[i].first.z);
		target = osg::Vec3d(viewpoints[i].second.x,viewpoints[i].second.y,viewpoints[i].second.z);
		cam->setViewMatrixAsLookAt(pos,target,up);


		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
		temp.push_back(mycam);
	}

	std::vector<AnalysisResult> result = DoAnalyse(temp,buildingPath,terrainPath,offset);

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}

struct RayTracingData
{
	TriangleList* triangles; 
	std::vector<Ray*>* rowToDo;
};

//Loop through all triangles and check if any rays intersect with triangles
void RayLoop(RayTracingData data)
{
	for(unsigned int k = 0; k < data.rowToDo->size(); k++)
	{
		Ray* ray = data.rowToDo->at(k);
		for(unsigned int l = 0; l < data.triangles->triangles.size(); l++)
		{
			Triangle* tri = data.triangles->triangles.at(l);

			Hit hit;
			if(ray->Intersect(tri,&hit))//Check if the ray hit the triangle and
			{
				if(!ray->collection->viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)].intersect || ray->collection->viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)].distance > hit.distance)//Check if it is closer than the previous one
				{
					ray->collection->viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)] = hit;
				}
			}
		}

	}
}

void RayTracing(TriangleList* triangles, std::vector<Ray*> rays)
{
	QTime time;
	time.start();

	unsigned int tCount = std::thread::hardware_concurrency();//Get how many thread we have
	unsigned int rayPerThread = rays.size() / tCount;

	//List of rays and their frag coord
	std::vector<Ray*>* toDo = new std::vector<Ray*>[tCount];//List of rays for each threads

	for(unsigned int i = 0; i < tCount; i++)
	{
		toDo[i].insert(toDo[i].begin(),rays.begin()+i*rayPerThread,rays.begin()+(i+1)*rayPerThread);
	}

	std::cout << "Thread : " << tCount << std::endl;

	std::vector<std::thread*> threads;//Our thread list

	for(unsigned int i = 0; i < tCount; i++)
	{
		RayTracingData data;
		data.triangles = triangles;
		data.rowToDo = &toDo[i];

		std::thread* t = new std::thread(RayLoop,data);
		threads.push_back(t);
	}

	std::cout << "Thread Launched " << std::endl;

	for(unsigned int i = 0; i < tCount; i++)//Join all our thread and update global data min and max distance
	{
		(*threads[i]).join();
		delete threads[i];
	}

	std::cout << "The joining is completed" << std::endl;

	delete[] toDo;

	std::cout << "Time : " << time.elapsed()/1000 << " sec" << std::endl;
}


TriangleList* BuildBuildingTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint, bool ignoreMiscBuilding)
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

	return new TriangleList(triangles);
}

TriangleList* BuildTerrainTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint)
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

	return new TriangleList(triangles);
}

osg::ref_ptr<osg::Geode> BuildSkylineOSGNode(std::vector<TVec3d> skyline)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::Geometry* geom = new osg::Geometry;
	osg::Vec3Array* vertices = new osg::Vec3Array;
	osg::Vec3Array* colors = new osg::Vec3Array;
	osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

	for(unsigned int i = 0; i < skyline.size(); i++)
	{
		vertices->push_back(osg::Vec3(skyline[i].x,skyline[i].y,skyline[i].z));
		colors->push_back(osg::Vec3(0.0,0.0,0.0));
	}

	for(unsigned int i = 0; i < skyline.size()-1; i++)
	{
		indices->push_back(i); indices->push_back(i+1);
	}

	
	geom->setVertexArray(vertices);
	geom->setColorArray(colors);
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

	if(!found)
		return;

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
			realX = Clamp(x+c.first,0,width);
			realY = Clamp(y+c.second,0,height);
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

void ViewPoint::ComputeMinMaxDistance()
{
	for(unsigned int i = 0; i < width; i++)
	{
		for(unsigned int j = 0; j < height; j++)
		{
			if(hits[i][j].intersect)
			{
				minDistance = std::min(minDistance,hits[i][j].distance);
				maxDistance = std::max(maxDistance,hits[i][j].distance);
			}
		}
	}
}

RayCollection* RayCollection::BuildCollection(osg::Camera* cam)
{
	RayCollection* rays = new RayCollection();
	for(unsigned int i = 0; i < cam->getViewport()->width(); i++)
	{
		for(unsigned int j = 0; j < cam->getViewport()->height(); j++)
		{
			Ray* ray = Ray::BuildRd(TVec2d(i,j),cam);
			ray->collection = rays;
			rays->rays.push_back(ray);
		}
	}

	return rays;
}