#include "Visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"

#include <thread>
#include <queue>

#include "Hit.hpp"
#include "Export.hpp"


std::queue<std::string> Setup(std::vector<AABB> boxes,RayCollection* rays)
{
	std::map<std::string,int> boxToMaxOrder;

	for(unsigned int i = 0; i < boxes.size(); i++)
	{
		boxToMaxOrder[boxes[i].name] = -1;
	}

	for(unsigned int i = 0; i < rays->rays.size(); i++)
	{
		Ray* ray = rays->rays[i];
		ray->boxes.clear();
		

		for(unsigned int j = 0; j < boxes.size(); j++)
		{
			float hit0,hit1;
			if(ray->Intersect(boxes[j],&hit0,&hit1))
			{
				RayBoxHit hTemp;
				hTemp.box = boxes[j];
				hTemp.minDistance = hit0;
				ray->boxes.push_back(hTemp);
			}
		}

		std::sort(ray->boxes.begin(),ray->boxes.end());

		for(int j = 0; j < ray->boxes.size(); j++)
		{
			int current = boxToMaxOrder[ray->boxes[j].box.name];
			boxToMaxOrder[ray->boxes[j].box.name] = std::max(j,current);
		}
	}

	std::vector<BoxOrder> boxesOrder;

	for(auto it = boxToMaxOrder.begin();it != boxToMaxOrder.end(); it++)
	{
		if(it->second >= 0)
		{
			BoxOrder boTemp;
			boTemp.box = it->first;
			boTemp.order = it->second;
			boxesOrder.push_back(boTemp);
		}
	}

	std::sort(boxesOrder.begin(),boxesOrder.end());

	std::queue<std::string> tileOrder;

	for(BoxOrder& bo : boxesOrder)
		tileOrder.push(bo.box);

	return tileOrder;
}

ViewPoint* DoMultiTileAnalysis(std::string dirTile, std::vector<AABB> boxes, TVec3d offset, osg::Camera* cam, citygml::CityObjectsType objectType)
{
	ViewPoint* viewpoint = new ViewPoint(cam->getViewport()->width(),cam->getViewport()->height());

	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
	TVec3d camDir = TVec3d(target.x(),target.y(),target.z());
	viewpoint->lightDir = Ray::Normalized(camPos - camDir);

	RayCollection* rays = RayCollection::BuildCollection(cam);
	viewpoint->rays = rays;
	rays->viewpoint = viewpoint;

	std::cout << "Viewpoint and collection created" << std::endl;

	std::queue<std::string> tileOrder = Setup(boxes,rays);

	std::cout << "Setup Completed" << " " << tileOrder.size() << std::endl;

	while(tileOrder.size() != 0)
	{
		std::string tileName = tileOrder.front();
		tileOrder.pop();

		RayCollection raysTemp;
		raysTemp.viewpoint = viewpoint;

		for(unsigned int i = 0; i < rays->rays.size(); i++)
		{
			Ray* ray = rays->rays[i];
			bool inter = viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)].intersect;
			bool found = false;

			for(RayBoxHit& rbh : ray->boxes)
				found = found || rbh.box.name == tileName;

			if(found && !inter)
			{
				raysTemp.rays.push_back(ray);
			}
		}

		std::cout << "Temp collection completed" << std::endl;

		std::string path = dirTile + tileName;

		vcity::Tile* tile = new vcity::Tile(path);
		//Get the triangle list
		TriangleList* trianglesTemp;
		trianglesTemp = BuildTriangleList(tile,offset,viewpoint,objectType);
		delete tile;

		RayTracing(trianglesTemp,raysTemp.rays);

		raysTemp.rays.clear();
	}

	viewpoint->ComputeSkyline();
	viewpoint->ComputeMinMaxDistance();

	delete rays;

	return viewpoint;
}

void MergeViewpoint(ViewPoint* a, ViewPoint* b)
{
	if(a->width != b->width || a->height != b->height)
		return;

	for(unsigned int i = 0; i < a->width; i++)
	{
		for(unsigned int j = 0; j < a->height; j++)
		{
			Hit hita = a->hits[i][j];
			Hit hitb = b->hits[i][j];

			if(!hita.intersect && hitb.intersect || (hita.intersect && hitb.intersect && hitb.distance < hita.distance))
			{
				a->hits[i][j] = hitb;
			}
		}
	}
}

std::vector<AnalysisResult> Analyse(std::string dirTile, TVec3d offset, osg::Camera* cam, std::string prefix)
{
	QTime time;
	time.start();

	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,256);

	std::pair<std::vector<AABB>,std::vector<AABB>> boxes = LoadAABB(dirTile);

	ViewPoint* viewpoint = DoMultiTileAnalysis(dirTile,boxes.first,offset,cam,citygml::CityObjectsType::COT_Building);
	ViewPoint* viewpointBis = DoMultiTileAnalysis(dirTile,boxes.second,offset,cam,citygml::CityObjectsType::COT_TINRelief);

	MergeViewpoint(viewpoint,viewpointBis);
	
	viewpoint->ComputeSkyline();
	viewpoint->ComputeMinMaxDistance();

	ExportData(viewpoint, prefix);
	ExportImages(viewpoint, prefix);

	
	delete viewpoint;
	delete viewpointBis;

	
	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	std::cout << "Total Time : " << time.elapsed()/1000 << " sec" << std::endl;

	return std::vector<AnalysisResult>();
}

std::vector<AnalysisResult> Analyse(std::string dirTile, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement)
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

	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));

		Analyse(dirTile,offset,mycam, std::to_string(i)+"_");

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}


	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return std::vector<AnalysisResult>();
}

std::vector<AnalysisResult> Analyse(std::string dirTile, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
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


	for(unsigned int i = 0; i < viewpoints.size(); i++)
	{
		pos = osg::Vec3d(viewpoints[i].first.x,viewpoints[i].first.y,viewpoints[i].first.z);
		target = osg::Vec3d(viewpoints[i].second.x,viewpoints[i].second.y,viewpoints[i].second.z);
		cam->setViewMatrixAsLookAt(pos,target,up);


		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
		Analyse(dirTile,offset,mycam, std::to_string(i)+"_");
	}


	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return std::vector<AnalysisResult>();
}