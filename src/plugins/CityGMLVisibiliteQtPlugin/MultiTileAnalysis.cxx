#include "Visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"

#include <thread>
#include <queue>
#include <qfileinfo.h>

#include "data/Hit.hpp"
#include "Export.hpp"
#include "RayTracing.h"
#include "data/BelvedereDB.h"


/**
*	@brief Setup a set of boxes in the right order depending on a set of rays
*/
std::queue<RayBoxHit> Setup(std::vector<AABB> boxes,RayCollection* rays)
{
	std::map<std::string,int> boxToMaxOrder;//We keep record the maximum order the box has be traverse
	std::map<std::string,RayBoxHit> boxToRayBoxHit;//We keep record the smallest rayboxhit of a box /// MultiResolution

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
				ray->boxes.push_back(hTemp); //On remplit la liste des boîtes intersectées par ce rayon
			}
		}

		std::sort(ray->boxes.begin(),ray->boxes.end());//Sort the boxes depending on the intersection distance

		for(int j = 0; j < ray->boxes.size(); j++)//We update the order of each boxes
		{
			int current = boxToMaxOrder[ray->boxes[j].box.name];
			boxToMaxOrder[ray->boxes[j].box.name] = std::max(j,current);

			if(boxToRayBoxHit.find(ray->boxes[j].box.name) != boxToRayBoxHit.end())//= Si ray->boxes[j].box.name existe déjà dans boxToRayBoxHit/// MultiResolution 
			{
				boxToRayBoxHit[ray->boxes[j].box.name] = std::min(ray->boxes[j],boxToRayBoxHit[ray->boxes[j].box.name]);
			}	
			else
			{
				boxToRayBoxHit.insert(std::make_pair(ray->boxes[j].box.name,ray->boxes[j]));
			}
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
	std::queue<RayBoxHit> tileOrder;

	for(BoxOrder& bo : boxesOrder)
		tileOrder.push(boxToRayBoxHit[bo.box]);

	return tileOrder;
}

std::string GetTilePrefixFromDistance(float distance)
{
	std::string result = "";

	if(distance > 5000)
		result = "LOD0_";
	else if(distance > 3000)
		result = "LOD1_";

	return result;
}

/**
*	@brief Perform the multitile analysis
*/
ViewPoint* DoMultiTileAnalysis(std::string dirTile, std::vector<AABB> boxes, osg::Camera* cam, citygml::CityObjectsType objectType)
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
	std::queue<RayBoxHit> tileOrder = Setup(boxes,rays);

	std::cout << "Setup Completed" << " " << tileOrder.size() << std::endl;

	//While we have boxes, tiles
	while(tileOrder.size() != 0)
	{
		RayBoxHit myRayBoxHit = tileOrder.front();
		std::string tileName = myRayBoxHit.box.name;
		tileOrder.pop();

		RayCollection raysTemp;//Not all rays intersect the box
		raysTemp.viewpoint = viewpoint;

		//We get only the rays that intersect the box
		for(unsigned int i = 0; i < rays->rays.size(); i++)
		{
			Ray* ray = rays->rays[i];//Get the ray
			bool inter = viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)].intersect;//Check the viewpoint to see if at the ray coordinates we have intersect something in a previous iteration
			bool found = false;

			for(RayBoxHit& rbh : ray->boxes)//Go through all the box that the ray intersect to see if the current box is one of them
				found = found || rbh.box.name == tileName;

			if(found && !inter)
			{
				raysTemp.rays.push_back(ray);
			}
		}

		std::cout << "Temp collection completed" << std::endl;

		if(raysTemp.rays.size() == 0)
		{
			std::cout << "Skipping." << std::endl;
			raysTemp.rays.clear();
			continue;
		}

		//Load triangles and perform analysis
		std::string path = dirTile + tileName;
		std::string pathWithPrefix = dirTile + GetTilePrefixFromDistance(myRayBoxHit.minDistance) + tileName; /// MultiResolution

		//Get the triangle list
		TriangleList* trianglesTemp;
		if(QFileInfo(pathWithPrefix.c_str()).exists()) /// MultiResolution
			trianglesTemp = BuildTriangleList(pathWithPrefix,objectType);
		else
			trianglesTemp = BuildTriangleList(path,objectType);

		RayTracing(trianglesTemp,raysTemp.rays);

		raysTemp.rays.clear();
	}

	//Return the result
	viewpoint->position = rays->rays.front()->ori;

	delete rays;

	return viewpoint;
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

			if((!hita.intersect && hitb.intersect) || (hita.intersect && hitb.intersect && hitb.distance < hita.distance))
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

			if((!hita.intersect && hitb.intersect) || (hita.intersect && hitb.intersect && hitb.distance < hita.distance)|| (hita.intersect && hitb.intersect && AreSame(hitb.distance,hita.distance)))
			{
				terrain->hits[i][j] = hitb;
			}
		}
	}
}

std::vector<ViewPoint*> MultiTileBasicAnalyse(std::string dirTile, osg::Camera* cam, std::string prefix)
{
	// In order to add a new data set, uncomment exemple and replace fillers <..> by your data
	QTime time;
	time.start();

	AABBCollection boxes = LoadAABB(dirTile);

	//Do the analysis for each layers, building, terrain, water
	ViewPoint* result = DoMultiTileAnalysis(dirTile,boxes.building,cam,citygml::CityObjectsType::COT_Building);
	std::cout << "===================================================" << std::endl;
	std::cout << "Building Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	ViewPoint* resultBis = DoMultiTileAnalysis(dirTile,boxes.terrain,cam,citygml::CityObjectsType::COT_TINRelief);
	std::cout << "===================================================" << std::endl;
	std::cout << "Terrain Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	ViewPoint* resultTer = DoMultiTileAnalysis(dirTile,boxes.water,cam,citygml::CityObjectsType::COT_WaterBody);
	std::cout << "===================================================" << std::endl;
	std::cout << "Water Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	ViewPoint* resultQuad = DoMultiTileAnalysis(dirTile,boxes.veget,cam,citygml::CityObjectsType::COT_SolitaryVegetationObject);
	std::cout << "===================================================" << std::endl;
	std::cout << "Veget Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	// ViewPoint* result<MyData> = DoMultiTileAnalysis(dirTile,boxes.<myData>,cam,citygml::CityObjectsType::COT_<MyDataType>);
	// std::cout << "===================================================" << std::endl;
	// std::cout << "<MyData> Done ! " << std::endl;
	// std::cout << "===================================================" << std::endl;

	//Merge all viewpoint into one
	MergeViewpointTerrainOther(resultBis,resultTer); //Au cas où il y a de l'eau collé à du terrain, on les merge en donnant la priorité à l'eau.
	MergeViewpointTerrainOther(resultBis,resultQuad);
	// MergeViewpoint(result,result<MyData>);
	MergeViewpoint(result,resultBis);

	//Get and export the results
	result->ComputeSkyline();
	result->ComputeMinMaxDistance();

	ExportData(result, prefix);
	ExportImages(result, prefix);
	BelvedereDB::Get().ExportViewpointData(result);

	std::cout << "Total Time : " << time.elapsed()/1000 << " sec" << std::endl;

	std::vector<ViewPoint*> returns;
	returns.push_back(result);

	return returns;
}

std::vector<ViewPoint*> MultiTileCascadeAnalyse(std::string dirTile,osg::Camera* cam, unsigned int count, float zIncrement)
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

	std::vector<ViewPoint*> results;

	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));

		results.push_back(MultiTileBasicAnalyse(dirTile,mycam, std::to_string(i)+"_").front());

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}

	return results;
}

std::vector<ViewPoint*> MultiTileMultiViewpointAnalyse(std::string dirTile,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
{
	//See monotile multiviewpoint analyse
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	up = osg::Vec3d(0,0,1);

	std::vector<ViewPoint*> results;

	for(unsigned int i = 0; i < viewpoints.size(); i++)
	{
		pos = osg::Vec3d(viewpoints[i].first.x,viewpoints[i].first.y,viewpoints[i].first.z);
		target = osg::Vec3d(viewpoints[i].second.x,viewpoints[i].second.y,viewpoints[i].second.z);
		cam->setViewMatrixAsLookAt(pos,target,up);

		results.push_back(MultiTileBasicAnalyse(dirTile,cam, std::to_string(i)+"_").front());
	}

	return results;
}

std::vector<ViewPoint*> MultiTilePanoramaAnalyse(std::string dirTile,osg::Camera* cam, std::string prefix)
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

	std::vector<ViewPoint*> results;

	cam->setViewMatrixAsLookAt(pos,target,up);
	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,mycam, prefix+"Front_").front());

	osg::Vec3d targetRight = pos + (dir^up);
	cam->setViewMatrixAsLookAt(pos,targetRight,up);
	osg::ref_ptr<osg::Camera> mycamT(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,mycamT, prefix+"Right_").front());

	osg::Vec3d targetBack = pos + ((dir^up)^up);
	cam->setViewMatrixAsLookAt(pos,targetBack,up);
	osg::ref_ptr<osg::Camera> mycamB(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,mycamB, prefix+"Back_").front());

	osg::Vec3d targetLeft = pos + (((dir^up)^up)^up);
	cam->setViewMatrixAsLookAt(pos,targetLeft,up);
	osg::ref_ptr<osg::Camera> mycamQ(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	results.push_back(MultiTileBasicAnalyse(dirTile,mycamQ, prefix+"Left_").front());

	ExportPanoramaSkyline(results[0],results[1],results[2],results[3],prefix);

	return results;
}

std::vector<std::vector<ViewPoint*>> MultiTileCascadePanoramaAnalyse(std::string dirTile,osg::Camera* cam, unsigned int count, float zIncrement)
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

	std::vector<std::vector<ViewPoint*>> results;

	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));

		results.push_back(MultiTilePanoramaAnalyse(dirTile,mycam, std::to_string(i)+"_"));

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}

	return results;
}
