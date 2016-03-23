#include "Visibilite.hpp"

#include "citygml.hpp"

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "src/gui/osg/osgGDAL.hpp"

#include <thread>
#include <queue>

#include "raytracing/Hit.hpp"
#include "Export.hpp"
#include "raytracing/RayTracing.hpp"


/**
*	@brief Perform the analysis of a set of viewpoint with a set of tile
*	@param cams List of camera, one for each viewpoint
*	@param paths Paths to all tiles used in the analysis
*	@return The analysis results
*/
std::vector<ViewPoint*> DoMonoTileAnalysis(std::string dirTile, std::vector<osg::ref_ptr<osg::Camera>> cams,std::vector<std::string> paths)
{
	ViewPoint** result = new ViewPoint*[cams.size()];
	RayCollection** rays = new RayCollection*[cams.size()];

    std::vector<Ray*> allRays;//All rays of all viewpoints are merged in one vector to launch the algo only once

	//For each camera, create its ray collection and viewpoint
	for(unsigned int i = 0; i < cams.size(); i++)
	{
		//Get the position and direction of the camera
		osg::Camera* cam = cams[i];
		std::cout << cam->getViewport() << std::endl;
		osg::Vec3d pos;
		osg::Vec3d target;
		osg::Vec3d up;
		cam->getViewMatrixAsLookAt(pos,target,up);

		TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
		TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

		//Create the viewpoint and ray collection
        std::string viewpointId = std::to_string(i);
        result[i] = new ViewPoint(cam->getViewport()->width(),cam->getViewport()->height(),viewpointId);
		result[i]->lightDir = Ray::Normalized(camPos - camDir);
		rays[i] = RayCollection::BuildCollection(cam);

        //rays[i]->viewpoint = result[i];
        for(Ray* r : rays[i]->rays)
        {
            r->id = viewpointId;
        }

		allRays.insert(allRays.end(),rays[i]->rays.begin(),rays[i]->rays.end());
	}

	//List of list of triangle for tiles
	std::vector<TriangleList*> triangles;

	std::cout << "Loading Scene." << std::endl;

	//Load each tiles
	for(unsigned int i = 0; i < paths.size(); i++)
	{
		//Get the triangle list
		TriangleList* trianglesTemp = BuildTriangleList(paths[i],citygml::CityObjectsType::COT_Building);
		triangles.push_back(trianglesTemp);
	}

	//For each tiles perform raytracing on it
	for(unsigned int i = 0; i < triangles.size(); i++)
    {
        std::vector<Hit*>* tmpHits = RayTracing(triangles[i],allRays);

        //Change viewpoint hits depending on distance of new hits
        for(Hit* h : *tmpHits)
        {
            //Get id of the viewpoint corresponding to the current hit
            std::string vpId = h->ray.id;

            //Get corresponding viewpoint
            ViewPoint* viewpoint = result[std::stoi(vpId)]; // The id of the viewpoint is actually its location in result

            //if there is not already a hit for this fragCoord or if the distance of the current hit is smaller than the existing one
            if(!viewpoint->hits[int(h->ray.fragCoord.x)][int(h->ray.fragCoord.y)].intersect
                    || viewpoint->hits[int(h->ray.fragCoord.x)][int(h->ray.fragCoord.y)].distance > h->distance)
            {
                viewpoint->hits[int(h->ray.fragCoord.x)][int(h->ray.fragCoord.y)] = *h;
            }
        }
    }

	std::vector<ViewPoint*> resReturn;//When results are stored

	for(unsigned int i = 0; i < cams.size(); i++)
	{
		result[i]->ComputeSkyline();//Compute the skyline of the viewpoint
		result[i]->ComputeMinMaxDistance();//Compute the min and max distance of the viewpoint
		//Export data
		ExportData(dirTile, result[i],std::to_string(i)+"_");
		ExportImages(dirTile, result[i],std::to_string(i)+"_");
		result[i]->position = rays[i]->rays.front()->ori;

		resReturn.push_back(result[i]);
	}

	//Delete everything not useful
	for(unsigned int i = 0; i < cams.size(); i++)
	{
		delete rays[i];
	}

	delete[] result;
	delete[] rays;

	for(unsigned int i = 0; i <  triangles.size();i++)
		delete triangles[i];

	return resReturn;
}

std::vector<ViewPoint*> BasisAnalyse(std::string dirTile, std::vector<std::string> paths,osg::Camera* cam)
{
	QTime time;
	time.start();

	//Copy cam and perform analysis
	std::vector<osg::ref_ptr<osg::Camera>> temp;

	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	temp.push_back(mycam);

	std::vector<ViewPoint*> result = DoMonoTileAnalysis(dirTile, temp,paths);
	
	std::cout << "Total Time : " << time.elapsed()/1000 << " sec" << std::endl;

	return result;

}

std::vector<ViewPoint*> CascadeAnalyse(std::string dirTile, std::vector<std::string> paths,osg::Camera* cam, unsigned int count, float zIncrement)
{
	//Get the info about the camera and update it
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

	std::vector<osg::ref_ptr<osg::Camera>> temp;

	//Perform the cascade analysis
	for(unsigned int i = 0; i < count; i++)
	{
		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
		temp.push_back(mycam);

		pos.z()+=zIncrement;
		target.z()+=zIncrement;
		cam->setViewMatrixAsLookAt(pos,target,up);
	}


	std::vector<ViewPoint*> result = DoMonoTileAnalysis(dirTile, temp,paths);

	return result;
}

std::vector<ViewPoint*> MultiViewpointAnalyse(std::string dirTile, std::vector<std::string> paths,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
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

	std::vector<osg::ref_ptr<osg::Camera>> temp;

	for(unsigned int i = 0; i < viewpoints.size(); i++)
	{
		pos = osg::Vec3d(viewpoints[i].first.x,viewpoints[i].first.y,viewpoints[i].first.z);
		target = osg::Vec3d(viewpoints[i].second.x,viewpoints[i].second.y,viewpoints[i].second.z);
		cam->setViewMatrixAsLookAt(pos,target,up);


		osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
		temp.push_back(mycam);
	}

	std::vector<ViewPoint*> result = DoMonoTileAnalysis(dirTile, temp,paths);

	return result;
}
