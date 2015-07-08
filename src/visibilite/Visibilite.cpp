#include "Visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"
#include "osg/LineWidth"

#include <thread>
#include <queue>

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

	unsigned int tCount = std::thread::hardware_concurrency()-1;//Get how many thread we have
	unsigned int rayPerThread = rays.size() / tCount;

	//List of rays and their frag coord
	std::vector<Ray*>* toDo = new std::vector<Ray*>[tCount];//List of rays for each threads

	for(unsigned int i = 0; i < tCount; i++)
	{
		toDo[i].insert(toDo[i].begin(),rays.begin()+i*rayPerThread,rays.begin()+(i+1)*rayPerThread);
	}

	std::cout << "Thread : " << tCount << std::endl;
	std::cout << "Ray count : " << rays.size() << std::endl;

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

TriangleList* BuildTriangleList(vcity::Tile* tile, TVec3d offset, ViewPoint* viewpoint, citygml::CityObjectsType objectType)
{
	std::vector<Triangle*> triangles;

	citygml::CityModel * model = tile->getCityModel();

	for(citygml::CityObject* obj : model->getCityObjectsRoots()) //For each city object
	{
		if(obj->getType() == citygml::COT_Building && objectType == citygml::COT_Building) //We only take building or terrain
		{
			QColor objectColor = qRgba(RandInt(0,255),RandInt(0,255),RandInt(0,255),255);
			std::pair<std::string,QColor> pair(obj->getId(),objectColor);
			if(viewpoint != nullptr)
			viewpoint->objectToColor.insert(pair);

			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
				for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
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
		else if((obj->getType() == citygml::COT_TINRelief  && objectType == citygml::COT_TINRelief) || (obj->getType() == citygml::COT_WaterBody  && objectType == citygml::COT_WaterBody)) //We only take building or terrain
		{
			QColor objectColor = qRgba(RandInt(0,255),RandInt(0,255),RandInt(0,255),255);
			std::pair<std::string,QColor> pair(obj->getId(),objectColor);
			if(viewpoint != nullptr)
			viewpoint->objectToColor.insert(pair);

			for(citygml::Geometry* Geometry : obj->getGeometries()) //pour chaque géométrie
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
		colors->push_back(osg::Vec3(1.0,0.0,1.0));
	}

	for(unsigned int i = 0; i < skyline.size()-1; i++)
	{
		indices->push_back(i); indices->push_back(i+1);
	}


	geom->setVertexArray(vertices);
	geom->setColorArray(colors);
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(indices);
	geode->addDrawable(geom);

	osg::LineWidth* linewidth = new osg::LineWidth(); 
	linewidth->setWidth(3.0f); 
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth); 

	return geode;
}

osg::ref_ptr<osg::Geode> BuildViewshedOSGNode(AnalysisResult result)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::Geometry* geom = new osg::Geometry;
	osg::Vec3Array* vertices = new osg::Vec3Array;
	osg::Vec3Array* colors = new osg::Vec3Array;
	osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

	ViewPoint* viewpoint = result.viewpoint;

	unsigned int cpt = 0;

	std::cout << "Beginning" << std::endl;

	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			Hit hit = viewpoint->hits[i][j];

			if(hit.intersect)
			{
				TVec3d point = hit.point;
				vertices->push_back(osg::Vec3(point.x,point.y,point.z));
				vertices->push_back(osg::Vec3(point.x,point.y,point.z+1.0));
				colors->push_back(osg::Vec3(0.0,1.0,0.0));
				colors->push_back(osg::Vec3(0.0,1.0,0.0));
				indices->push_back(cpt++);
				indices->push_back(cpt++);
			}
		}
	}
	std::cout << "Ending" << std::endl;

	geom->setVertexArray(vertices);
	geom->setColorArray(colors);
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(indices);
	geode->addDrawable(geom);

	osg::LineWidth* linewidth = new osg::LineWidth(); 
	linewidth->setWidth(6.0f); 
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth); 

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

	skyline.clear();

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

	unsigned int realX = Clamp(x+c.first,0,width-1);
	unsigned int realY = Clamp(y+c.second,0,height-1);
	
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
	minDistance = FLT_MAX;
	maxDistance = -FLT_MAX;
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