#include <thread>
#include <queue>
#ifdef _MSC_VER                 
  // Inhibit dll-interface warnings concerning gdal-1.11.4 internals
  // (cpl_string.h) when including ogrsf_frmts.h and/or gdal_priv.h
  // on VC++
  # pragma warning(disable:4251) 
#endif
#include <ogrsf_frmts.h>
#include <gdal_priv.h>    // Gdal
#include <cpl_conv.h>     // Gdal for CPLMalloc()
#include <qfileinfo.h>

#include "Visibilite.hpp"
#include "Export.hpp"
#include "data/BelvedereDB.h"
#include "AABB.hpp"
#include "core/RayBox.hpp"

/**
*	@brief Calcule la distance 2D min entre une box et un point (la camera)
*/
float CalculDistMin(TVec3d camPos, AABB box)
{
  //If the camera lies within the box
	if(camPos.x >= box.min.x && camPos.x <= box.max.x && camPos.y >= box.min.y && camPos.y <= box.max.y) 
		return 0;

	TVec2d BoxP1 = TVec2d(box.min.x, box.min.y);
	TVec2d BoxP2 = TVec2d(box.min.x, box.max.y);
	TVec2d BoxP3 = TVec2d(box.max.x, box.max.y);
	TVec2d BoxP4 = TVec2d(box.max.x, box.min.y);

	OGRPolygon* Poly = new OGRPolygon;
	OGRLinearRing* Ring = new OGRLinearRing;
	Ring->addPoint(BoxP1.x, BoxP1.y);
	Ring->addPoint(BoxP2.x, BoxP2.y);
	Ring->addPoint(BoxP3.x, BoxP3.y);
	Ring->addPoint(BoxP4.x, BoxP4.y);
	Ring->addPoint(BoxP1.x, BoxP1.y);
	Poly->addRingDirectly(Ring);

	OGRPoint* Point = new OGRPoint(camPos.x, camPos.y);

	double DistMin = Point->Distance(Poly);

	delete Point;
	delete Poly;

	return DistMin;
}

/**
*	@brief Setup a set of boxes in the right order depending on a set of rays
*/
std::vector<BoxwithRays> Setup(std::vector<AABB> boxes, RayBoxCollection* rays, TVec3d camPos)
{
	std::vector<BoxwithRays> OrderedListBox;

	for(AABB box: boxes)
	{
		BoxwithRays BoxInfo;
		BoxInfo.box = box;
		BoxInfo.minDistance = CalculDistMin(camPos, box);

		for(unsigned int i = 0; i < rays->raysBB.size(); i++)
		{
			RayBox* rayBox = rays->raysBB[i];
			rayBox->boxes.clear();

			float hit0, hit1;
			if(rayBox->Intersect(box, &hit0, &hit1))
			{
				RayBoxHit hTemp;
				hTemp.box = box;
				hTemp.minDistance = hit0;
        // On remplit la liste des boites intersectees par ce rayon
				rayBox->boxes.push_back(hTemp);
        // On met l'indice de ce rayon dans la boite car il l'intersecte
				BoxInfo.IndicesRays.push_back(i);
			}
		}

		OrderedListBox.push_back(BoxInfo);
	}

  // On trie les boites suivant leur minDistance pour avoir les plus
  // proches de la camera en premier
	std::sort(OrderedListBox.begin(), OrderedListBox.end());

	return OrderedListBox;
}

std::string GetTilePrefixFromDistance(float distance, double DistLod1)
{
	std::string result = "";

	return result;
}

/**
*	@brief Perform the multitile analysis
*/
ViewPoint* DoMultiTileAnalysis(std::string dirTile, std::vector<AABB> boxes, osg::Camera* cam, citygml::CityObjectsType objectType, double DistLod1)
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
	RayBoxCollection* raysBoxes  = RayBoxCollection::BuildCollection(cam);
	//rays->viewpoint = viewpoint;

	std::cout << "Viewpoint and collection created" << std::endl;
	 
	//Setup and get the tile's boxes in the right intersection order
	std::vector<BoxwithRays> tileOrder = Setup(boxes, raysBoxes, camPos);

	std::cout << "Setup Completed" << " " << tileOrder.size() << " tiles" << std::endl;

	for(BoxwithRays currBox : tileOrder)
	{
		std::string tileName = currBox.box.name;

    // Parmi les rayons qui intersectent la box, il ne faut conserver
    // que ceux qui n'ont pas deja eu une intersection avec une boite plus proche
		RayCollection raysTemp;

		// We get only the rays that intersect the box
		for(unsigned int i : currBox.IndicesRays)
		{
      // Get the ray
			RayBox* raybox = raysBoxes->raysBB[i]; 
			bool inter = viewpoint->hits[int(raybox->fragCoord.x)][int(raybox->fragCoord.y)].intersect;//Check the viewpoint to see if at the ray coordinates we have intersect something in a previous iteration
			if(!inter)
			{
				raysTemp.rays.push_back(raybox);
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
		std::string pathWithPrefix = dirTile + GetTilePrefixFromDistance(currBox.minDistance, DistLod1) + tileName; /// MultiResolution

		//Get the triangle list
		TriangleList* trianglesTemp;
		if(QFileInfo(pathWithPrefix.c_str()).exists()) /// MultiResolution
			trianglesTemp = BuildTriangleList(pathWithPrefix, objectType);
		else
			trianglesTemp = BuildTriangleList(path, objectType);

		//Perform raytracing and get vector of hits
		std::vector<Hit*>* tmpHits = RayTracing(trianglesTemp, raysTemp.rays);

		//std::cout << "Test1" << std::endl;

		//std::cout << tmpHits->size() << std::endl;
		//std::cout << trianglesTemp->triangles.size() << std::endl;

		//std::cout << "Test2" << std::endl;

		//Change viewpoint hits depending on distance of new hits
        for(Hit* h : *tmpHits)
        {
			//std::cout << "Test4" << std::endl;
            //if there is not already a hit for this fragCoord or if the distance of the current hit is smaller than the existing one
            if(!viewpoint->hits[int(h->ray.fragCoord.x)][int(h->ray.fragCoord.y)].intersect
                    || viewpoint->hits[int(h->ray.fragCoord.x)][int(h->ray.fragCoord.y)].distance > h->distance)
            {
				//std::cout << h->ray.fragCoord.x << " " << h->ray.fragCoord.y << " - " << viewpoint->width << " " << viewpoint->height << std::endl;
                viewpoint->hits[int(h->ray.fragCoord.x)][int(h->ray.fragCoord.y)] = *h;
            }
			//std::cout << "Test6" << std::endl;
        }

		//std::cout << "Test3" << std::endl;

		delete trianglesTemp;

		raysTemp.rays.clear();
		delete tmpHits;
	}

	//Return the result
	viewpoint->position = raysBoxes->raysBB.front()->ori;

	delete raysBoxes;

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

std::vector<ViewPoint*> MultiTileBasicAnalyse(std::string dirTile, osg::Camera* cam, std::string prefix, double DistLod1)
{
	// In order to add a new data set, uncomment exemple and replace fillers <..> by your data
	QTime time;
	time.start();

    AABBCollection boxes = LoadLayersAABBs(dirTile);

	//Do the analysis for each layers, building, terrain, water
	ViewPoint* result = DoMultiTileAnalysis(dirTile,boxes.building,cam,citygml::CityObjectsType::COT_Building, DistLod1);
	std::cout << "===================================================" << std::endl;
	std::cout << "Building Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	ViewPoint* resultBis = DoMultiTileAnalysis(dirTile,boxes.terrain,cam,citygml::CityObjectsType::COT_TINRelief, DistLod1);
	std::cout << "===================================================" << std::endl;
	std::cout << "Terrain Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	ViewPoint* resultTer = DoMultiTileAnalysis(dirTile,boxes.water,cam,citygml::CityObjectsType::COT_WaterBody, DistLod1);
	std::cout << "===================================================" << std::endl;
	std::cout << "Water Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;
	ViewPoint* resultQuad = DoMultiTileAnalysis(dirTile,boxes.veget,cam,citygml::CityObjectsType::COT_SolitaryVegetationObject, DistLod1);
	std::cout << "===================================================" << std::endl;
	std::cout << "Veget Done ! " << std::endl;
	std::cout << "===================================================" << std::endl;

	//Merge all viewpoint into one
  // Au cas ou il y a de l'eau collee a du terrain, on les merge en donnant
  // la priorite a l'eau.
	MergeViewpointTerrainOther(resultBis,resultTer);
	MergeViewpointTerrainOther(resultBis,resultQuad);
	MergeViewpoint(result,resultBis);

	//Get and export the results
	result->ComputeSkyline();
	result->ComputeMinMaxDistance();

	ExportData(dirTile, result, prefix);
	ExportImages(dirTile, result, prefix);

	BelvedereDB::Get().ExportViewpointData(result);

	std::cout << "Total Time : " << time.elapsed()/1000 << " sec" << std::endl;

	std::vector<ViewPoint*> returns;
	returns.push_back(result);

	delete result;
	delete resultBis;
	delete resultTer;
	delete resultQuad;

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
	cam->setProjectionMatrixAsPerspective(90,ratio,znear,zfar);
	//cam->setProjectionMatrixAsPerspective(50,ratio,znear,zfar);

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

	ExportPanoramaSkyline(dirTile, results[0],results[1],results[2],results[3],prefix);

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
