#include "Visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"

#include <thread>
#include <queue>

#include "data/Hit.hpp"
#include "Export.hpp"
#include "RayTracing.h"

/**
*	@brief Setup a set of boxes in the right order depending on a set of rays
*/
std::queue<std::string> Setup(std::vector<AABB> boxes,RayCollection* rays)
{
	std::map<std::string,int> boxToMaxOrder;//We keep record the maximum order the box has be traverse

	for(unsigned int i = 0; i < boxes.size(); i++)
	{
		boxToMaxOrder[boxes[i].name] = -1;
	}
	//For each rays
	for(unsigned int i = 0; i < rays->rays.size(); i++)
	{
		Ray* ray = rays->rays[i];
		ray->boxes.clear();

		//For each boxes we check if the ray intersect the box and store it in the box list of the array
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

		std::sort(ray->boxes.begin(),ray->boxes.end());//Sort the boxes depending on the intersection distance

		for(int j = 0; j < ray->boxes.size(); j++)//We update the order of each boxes
		{
			int current = boxToMaxOrder[ray->boxes[j].box.name];
			boxToMaxOrder[ray->boxes[j].box.name] = std::max(j,current);
		}
	}

	//Sort the boxis depending on their max order
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

	//Setup the queue and return it
	std::queue<std::string> tileOrder;

	for(BoxOrder& bo : boxesOrder)
		tileOrder.push(bo.box);

	return tileOrder;
}

/**
*	@brief Perform the multitile analysis
*/
AnalysisResult DoMultiTileAnalysis(std::string dirTile, std::vector<AABB> boxes, TVec3d offset, osg::Camera* cam, citygml::CityObjectsType objectType)
{
	//Create the viewpoint
	ViewPoint* viewpoint = new ViewPoint(cam->getViewport()->width(),cam->getViewport()->height());

	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	//Get the camera position and direction to setup the light
	TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
	TVec3d camDir = TVec3d(target.x(),target.y(),target.z());
	viewpoint->lightDir = Ray::Normalized(camPos - camDir);

	//Build the ray collection
	RayCollection* rays = RayCollection::BuildCollection(cam);
	rays->viewpoint = viewpoint;

	std::cout << "Viewpoint and collection created" << std::endl;

	//Setup and get the tile's boxes in the right intersection order
	std::queue<std::string> tileOrder = Setup(boxes,rays);

	std::cout << "Setup Completed" << " " << tileOrder.size() << std::endl;

	//While we have boxes, tiles
	while(tileOrder.size() != 0)
	{
		std::string tileName = tileOrder.front();
		tileOrder.pop();

		RayCollection raysTemp;//Not all rays intersect the box
		raysTemp.viewpoint = viewpoint;

		//We get only the rays that intersect the box
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

		//Load triangles and perform analysis
		std::string path = dirTile + tileName;

		vcity::Tile* tile = new vcity::Tile(path);
		//Get the triangle list
		TriangleList* trianglesTemp;
		trianglesTemp = BuildTriangleList(tile,offset,viewpoint,objectType);
		delete tile;

		RayTracing(trianglesTemp,raysTemp.rays);

		raysTemp.rays.clear();
	}

	//Return the result
	AnalysisResult result;
	result.viewpointPosition = rays->rays.front()->ori;
	result.viewpoint = viewpoint;

	delete rays;

	return result;
}

//Merge two viewpoint, b into a
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

//Compare two float with a bigger epsilon than usual
bool AreSame(float a, float b)
{
	return fabs(a - b) < 2*std::numeric_limits<float>::epsilon();
}

//Merge two viewpoint, other into terrain, the comparaison function is not the base one
void MergeViewpointTerrainOther(ViewPoint* terrain, ViewPoint* other)
{
	if(terrain->width != other->width || terrain->height != other->height)
		return;

	for(unsigned int i = 0; i < terrain->width; i++)
	{
		for(unsigned int j = 0; j < terrain->height; j++)
		{
			Hit hita = terrain->hits[i][j];
			Hit hitb = other->hits[i][j];

			if(!hita.intersect && hitb.intersect || (hita.intersect && hitb.intersect && hitb.distance < hita.distance)|| (hita.intersect && hitb.intersect && AreSame(hitb.distance,hita.distance)))
			{
				terrain->hits[i][j] = hitb;
			}
		}
	}
}

std::vector<AnalysisResult> MultiTileBasicAnalyse(std::string dirTile, TVec3d offset, osg::Camera* cam, std::string prefix)
{
	QTime time;
	time.start();

	AABBCollection boxes = LoadAABB(dirTile);

	//Do the analysis for each layers, building, terrain, water
	AnalysisResult result = DoMultiTileAnalysis(dirTile,boxes.building,offset,cam,citygml::CityObjectsType::COT_Building);
	std::cout << "===================================================" << std::endl;
	std::cout << "Building Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	AnalysisResult resultBis = DoMultiTileAnalysis(dirTile,boxes.terrain,offset,cam,citygml::CityObjectsType::COT_TINRelief);
	std::cout << "===================================================" << std::endl;
	std::cout << "Terrain Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	AnalysisResult resultTer = DoMultiTileAnalysis(dirTile,boxes.water,offset,cam,citygml::CityObjectsType::COT_WaterBody);
	std::cout << "===================================================" << std::endl;
	std::cout << "Water Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	AnalysisResult resultQuad = DoMultiTileAnalysis(dirTile,boxes.veget,offset,cam,citygml::CityObjectsType::COT_SolitaryVegetationObject);
	std::cout << "===================================================" << std::endl;
	std::cout << "Veget Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	std::cout << "Got boxes : " << boxes.veget.size() << std::endl;
	ExportImages(resultQuad.viewpoint, "CheckCheck");

	//Merge all viewpoint into one
	MergeViewpointTerrainOther(resultBis.viewpoint,resultTer.viewpoint);
	MergeViewpointTerrainOther(resultBis.viewpoint,resultQuad.viewpoint);
	MergeViewpoint(result.viewpoint,resultBis.viewpoint);

	//Get and export the results
	result.viewpoint->ComputeSkyline();
	result.viewpoint->ComputeMinMaxDistance();

	for(unsigned int j = 0; j < result.viewpoint->skyline.size(); j++)
	{
		Hit h = result.viewpoint->hits[result.viewpoint->skyline[j].first][result.viewpoint->skyline[j].second];
		result.skyline.push_back(std::make_pair(TVec2d(result.viewpoint->skyline[j].first,result.viewpoint->skyline[j].second),h.point));
	}

	ExportData(result.viewpoint, prefix);
	ExportImages(result.viewpoint, prefix);

	std::cout << "Total Time : " << time.elapsed()/1000 << " sec" << std::endl;

	std::vector<AnalysisResult> returns;
	returns.push_back(result);

	return returns;
}

std::vector<AnalysisResult> MultiTileCascadeAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement)
{
	//See monotile cascade analyse
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

	std::vector<AnalysisResult> results;

	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));

		results.push_back(MultiTileBasicAnalyse(dirTile,offset,mycam, std::to_string(i)+"_").front());

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}

	return results;
}

std::vector<AnalysisResult> MultiTileMultiViewpointAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
{
	//See monotile multiviewpoint analyse
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

	std::vector<AnalysisResult> results;

	for(unsigned int i = 0; i < viewpoints.size(); i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
		results.push_back(MultiTileBasicAnalyse(dirTile,offset,mycam, std::to_string(i)+"_").front());

		pos = osg::Vec3d(viewpoints[i].first.x,viewpoints[i].first.y,viewpoints[i].first.z);
		target = osg::Vec3d(viewpoints[i].second.x,viewpoints[i].second.y,viewpoints[i].second.z);
		cam->setViewMatrixAsLookAt(pos,target,up);
	}

	return results;
}

std::vector<AnalysisResult> MultiTilePanoramaAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, std::string prefix)
{
	//Panorama viewpoint, basicaly we do the front view, right view, back and left view. We just turn the camera betweeen each analysis
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

	double fov;
	double ratio;
	double znear;
	double zfar;
	cam->getProjectionMatrixAsPerspective(fov,ratio,znear,zfar);
	cam->setProjectionMatrixAsPerspective(50,ratio,znear,zfar);

	std::vector<AnalysisResult> results;

	cam->setViewMatrixAsLookAt(pos,target,up);
	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,offset,mycam, prefix+"Front_").front());

	osg::Vec3d targetRight = pos + (dir^up);
	cam->setViewMatrixAsLookAt(pos,targetRight,up);
	osg::ref_ptr<osg::Camera> mycamT(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,offset,mycamT, prefix+"Right_").front());

	osg::Vec3d targetBack = pos + ((dir^up)^up);
	cam->setViewMatrixAsLookAt(pos,targetBack,up);
	osg::ref_ptr<osg::Camera> mycamB(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,offset,mycamB, prefix+"Back_").front());

	osg::Vec3d targetLeft = pos + (((dir^up)^up)^up);
	cam->setViewMatrixAsLookAt(pos,targetLeft,up);
	osg::ref_ptr<osg::Camera> mycamQ(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,offset,mycamQ, prefix+"Left_").front());

	ExportPanoramaSkyline(results[0],results[1],results[2],results[3],prefix);

	return results;
}

std::vector<std::vector<AnalysisResult>> MultiTileCascadePanoramaAnalyse(std::string dirTile, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement)
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

	std::vector<std::vector<AnalysisResult>> results;

	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));

		results.push_back(MultiTilePanoramaAnalyse(dirTile,offset,mycam, std::to_string(i)+"_"));

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}

	return results;
}