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


std::vector<AnalysisResult> DoMonoTileAnalysis(std::vector<osg::ref_ptr<osg::Camera>> cams,std::vector<std::string> paths, TVec3d offset)
{
	ViewPoint** result = new ViewPoint*[cams.size()];
	RayCollection** rays = new RayCollection*[cams.size()];

	std::vector<Ray*> allRays;

	for(unsigned int i = 0; i < cams.size(); i++)
	{
		osg::Camera* cam = cams[i];
		std::cout << cam->getViewport() << std::endl;
		osg::Vec3d pos;
		osg::Vec3d target;
		osg::Vec3d up;
		cam->getViewMatrixAsLookAt(pos,target,up);

		TVec3d camPos = TVec3d(pos.x(),pos.y(),pos.z());
		TVec3d camDir = TVec3d(target.x(),target.y(),target.z());

		result[i] = new ViewPoint(cam->getViewport()->width(),cam->getViewport()->height());
		result[i]->lightDir = Ray::Normalized(camPos - camDir);
		rays[i] = RayCollection::BuildCollection(cam);

		rays[i]->viewpoint = result[i];

		allRays.insert(allRays.end(),rays[i]->rays.begin(),rays[i]->rays.end());
	}

	std::vector<TriangleList*> triangles;

	std::cout << "Loading Scene." << std::endl;

	for(unsigned int i = 0; i < paths.size(); i++)
	{
		vcity::Tile* tile = new vcity::Tile(paths[i]);

		//Get the triangle list
		TriangleList* trianglesTemp = BuildTriangleList(tile,offset,result[0],citygml::CityObjectsType::COT_Building);
		triangles.push_back(trianglesTemp);
		delete tile;
	}

	for(unsigned int i = 1; i < cams.size(); i++)
		result[i]->objectToColor = result[0]->objectToColor;


	for(unsigned int i = 0; i < triangles.size(); i++)
		RayTracing(triangles[i],allRays);

	std::vector<AnalysisResult> resReturn;

	for(unsigned int i = 0; i < cams.size(); i++)
	{
		result[i]->ComputeSkyline();
		result[i]->ComputeMinMaxDistance();
		ExportData(result[i],std::to_string(i)+"_");
		ExportImages(result[i],std::to_string(i)+"_");
		AnalysisResult temp;
		temp.viewpointPosition = rays[i]->rays.front()->ori;
		temp.viewpoint = result[i];

		for(unsigned int j = 0; j < result[i]->skyline.size(); j++)
		{
			Hit h = result[i]->hits[result[i]->skyline[j].first][result[i]->skyline[j].second];
			temp.skyline.push_back(h.point);
		}

		resReturn.push_back(temp);
	}

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

std::vector<AnalysisResult> Analyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam)
{
	QTime time;
	time.start();

	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,256);



	std::vector<osg::ref_ptr<osg::Camera>> temp;

	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	temp.push_back(mycam);

	std::vector<AnalysisResult> result = DoMonoTileAnalysis(temp,paths,offset);

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);
	
	std::cout << "Total Time : " << time.elapsed()/1000 << " sec" << std::endl;

	return result;

}

std::vector<AnalysisResult> Analyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam, unsigned int count, float zIncrement)
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


	std::vector<AnalysisResult> result = DoMonoTileAnalysis(temp,paths,offset);


	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}

std::vector<AnalysisResult> Analyse(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam, std::vector<std::pair<TVec3d,TVec3d>> viewpoints)
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

	std::vector<AnalysisResult> result = DoMonoTileAnalysis(temp,paths,offset);

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}

std::vector<AnalysisResult> AnalyseTestCam(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam)
{
	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,128);

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

	osg::ref_ptr<osg::Viewport> vp(new osg::Viewport(0,0,256,128));
	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycam->setViewport(vp);
	temp.push_back(mycam);

	osg::ref_ptr<osg::Viewport> vpB(new osg::Viewport(0,0,128,256));
	osg::ref_ptr<osg::Camera> mycamB(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamB->setViewport(vpB);
	temp.push_back(mycamB);

	osg::ref_ptr<osg::Viewport> vpT(new osg::Viewport(0,0,256,256*1.3));
	osg::ref_ptr<osg::Camera> mycamT(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamT->setViewport(vpT);
	temp.push_back(mycamT);

	up = osg::Vec3d(0,-1,0);
	cam->setViewMatrixAsLookAt(pos,target,up);
	osg::ref_ptr<osg::Viewport> vpQ(new osg::Viewport(0,0,256,128));
	osg::ref_ptr<osg::Camera> mycamQ(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamQ->setViewport(vpQ);
	temp.push_back(mycamQ);

	std::vector<AnalysisResult> result = DoMonoTileAnalysis(temp,paths,offset);

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}

std::vector<AnalysisResult> AnalyseTestViewport(std::vector<std::string> paths, TVec3d offset,osg::Camera* cam)
{
	unsigned int oldWidth = cam->getViewport()->width();
	unsigned int oldHeight = cam->getViewport()->height();

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),256,128);

	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);


	std::vector<osg::ref_ptr<osg::Camera>> temp;

	osg::ref_ptr<osg::Viewport> vp(new osg::Viewport(0,0,128,128));
	osg::ref_ptr<osg::Camera> mycam(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycam->setViewport(vp);
	temp.push_back(mycam);

	osg::ref_ptr<osg::Viewport> vpB(new osg::Viewport(0,0,256,256));
	osg::ref_ptr<osg::Camera> mycamB(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamB->setViewport(vpB);
	temp.push_back(mycamB);

	osg::ref_ptr<osg::Viewport> vpT(new osg::Viewport(0,0,512,512));
	osg::ref_ptr<osg::Camera> mycamT(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamT->setViewport(vpT);
	temp.push_back(mycamT);

	osg::ref_ptr<osg::Viewport> vpTQ(new osg::Viewport(0,0,1024,1024));
	osg::ref_ptr<osg::Camera> mycamTQ(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamTQ->setViewport(vpTQ);
	temp.push_back(mycamTQ);

	osg::ref_ptr<osg::Viewport> vpTQQ(new osg::Viewport(0,0,2048,2048));
	osg::ref_ptr<osg::Camera> mycamTQQ(new osg::Camera(*cam,osg::CopyOp::DEEP_COPY_ALL));
	mycamTQQ->setViewport(vpTQQ);
	temp.push_back(mycamTQQ);

	std::vector<AnalysisResult> result = DoMonoTileAnalysis(temp,paths,offset);

	cam->setViewport(cam->getViewport()->x(),cam->getViewport()->y(),oldWidth,oldHeight);

	return result;
}
