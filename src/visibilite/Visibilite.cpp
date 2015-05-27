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

int RandInt(int low, int high)
{
	// Random number between low and high
	return qrand() % ((high + 1) - low) + low;
}

void Analyse(std::string path, TVec3d offset,osg::Camera* cam)
{

	vcity::Tile* tile = new vcity::Tile(path);

	//Get the triangle list
	GlobalData globalData;
	std::vector<Triangle*> triangles = BuildTriangleList(tile,offset,&globalData);

	RayTracingResult result = RayTracing(triangles,&globalData,offset,cam);

	ExportImage(result,&globalData);
	ProcessResult(result,&globalData);

	for(unsigned int i = 0; i < result.width; i++)
	{
		delete[] result.hits[i];
	}
	delete[] result.hits;

	for(unsigned int i = 0; i != triangles.size(); i++)
		delete triangles[i];

	delete tile;
}

struct TempD
{
	Hit** result;
	Hit** resultRemarquable;
	float* minDistance; 
	float* maxDistance; 
	std::vector<Triangle*>* triangles; 
	std::vector<std::pair<Ray,std::pair<unsigned int,unsigned int>>>* rowToDo;
};

void RayLoop(TempD data)
{
	for(unsigned int l = 0; l < data.triangles->size(); l++)
	{
		Triangle* tri = data.triangles->at(l);

		for(unsigned int k = 0; k < data.rowToDo->size(); k++)
		{
			Ray ray = data.rowToDo->at(k).first;
			unsigned int i = data.rowToDo->at(k).second.first;
			unsigned int j = data.rowToDo->at(k).second.second;

			Hit hit;
			if(ray.Intersect(tri,&hit))//Check if the ray hit the triangle and
			{
				if(!data.result[i][j].intersect || data.result[i][j].distance > hit.distance)//Check if it is closer than the previous one
				{
					data.result[i][j] = hit;
					*data.maxDistance = std::max(hit.distance,*data.maxDistance);
					*data.minDistance = std::min(hit.distance,*data.minDistance);
				}

				QString tempStr(hit.triangle->parent->getId().c_str());
				if(!tempStr.startsWith("LYON") && ( !data.resultRemarquable[i][j].intersect || data.resultRemarquable[i][j].distance > hit.distance ) )//Check if it is closer than the previous one
				{
					data.resultRemarquable[i][j] = hit;
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
	Hit** resultRemarquable = new Hit*[WWIDTH];

	QTime time;
	time.start();

	unsigned int tCount = std::thread::hardware_concurrency();

	std::vector<std::pair<Ray,std::pair<unsigned int,unsigned int>>>* toDo = new std::vector<std::pair<Ray,std::pair<unsigned int,unsigned int>>>[tCount];
	unsigned int current = 0;


	for(unsigned int i = 0; i < WWIDTH; i++)
	{
		result[i] = new Hit[WHEIGHT];
		resultRemarquable[i] = new Hit[WHEIGHT];
		for(unsigned int j = 0; j < WHEIGHT; j++)//Foreach pixel on our image
		{
			toDo[current].push_back(std::make_pair(Ray::BuildRd(TVec2d(i,j),cam), std::make_pair(i,j)));
			current++;
			current = current % tCount;
		}
	}	

	std::cout << "Thread : " << tCount << std::endl;

	float* minDist = new float[tCount];
	float* maxDist = new float[tCount];

	std::vector<std::thread*> threads;

	for(unsigned int i = 0; i < tCount; i++)
	{
		minDist[i] = globalData->minDistance;
		maxDist[i] = globalData->maxDistance;
		TempD data;
		data.result = result;
		data.minDistance = &minDist[i];
		data.maxDistance = &maxDist[i];
		data.resultRemarquable = resultRemarquable;
		data.triangles = &triangles;
		data.rowToDo = &toDo[i];

		std::thread* t = new std::thread(RayLoop,data);
		threads.push_back(t);
	}

	std::cout << "Thread Launched " << std::endl;

	for(unsigned int i = 0; i < tCount; i++)
	{
		(*threads[i]).join();
		globalData->minDistance = std::min(globalData->minDistance,minDist[i]);
		globalData->maxDistance = std::max(globalData->maxDistance,maxDist[i]);
		delete threads[i];
	}

	std::cout << "The joining is completed" << std::endl;

	delete[] minDist;
	delete[] maxDist;
	delete[] toDo;

	std::cout << "Time : " << time.elapsed()/1000 << " sec" << std::endl;

	RayTracingResult rtResult;
	rtResult.width = WWIDTH;
	rtResult.height = WHEIGHT;
	rtResult.hits = result;
	rtResult.hitsRemarquableBuildingOnly = resultRemarquable;
	rtResult.lightDir = Ray::Normalized(  camPos - camDir);

	return rtResult;
}

void ProcessResult(RayTracingResult result, GlobalData* globalData)
{
	float cpt = 0.0f;
	float cptMiss = 0.0f;
	float cptHit = 0.0f;
	float cptRoof = 0.0f;
	float cptWall = 0.0f;
	float cptBuilding = 0.0f;
	float cptRemarquable = 0.0f;
	float cptOnlyRemarquable = 0.0f;

	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			cpt++;
			if(result.hits[i][j].intersect)
			{
				cptHit++;

				if(result.hits[i][j].triangle->parent->getType() == citygml::COT_Building)
				{
					if(result.hits[i][j].triangle->object->getType() == citygml::COT_RoofSurface)
						cptRoof++;
					else
						cptWall++;

					QString tempStr(result.hits[i][j].triangle->parent->getId().c_str());
					if(tempStr.startsWith("LYON"))//Check to see if this is an important building
						cptBuilding++;
					else
						cptRemarquable++;
				}
			}
			else
				cptMiss++;

			if(result.hitsRemarquableBuildingOnly[i][j].intersect)
				cptOnlyRemarquable++;
		}
	}

	std::ofstream ofs;
	ofs.open ("./SkylineOutput/result.csv", std::ofstream::out);

	ofs << "Param;Count;Global%;InHit%" << std::endl;
	ofs << "Pixel;"<<cpt << ";100%" << std::endl;
	ofs << "Hit;"<< cptHit << ";" << cptHit/cpt * 100.f << std::endl;
	ofs << "Miss;"<<cptMiss << ";" << cptMiss/cpt * 100.f << std::endl;
	ofs << "Roof;" << cptRoof << ";" << cptRoof/cpt * 100.f << ";" << cptRoof/cptHit * 100.f << std::endl;
	ofs << "Wall;" << cptWall << ";" << cptWall/cpt * 100.f << ";" << cptWall/cptHit * 100.f << std::endl;
	ofs << "Building Misc;" << cptBuilding << ";" << cptBuilding/cpt * 100.f << ";" << cptBuilding/cptHit * 100.f << std::endl;
	ofs << "Remarquable Building;" << cptRemarquable << ";" << cptRemarquable/cpt * 100.f << ";" << cptRemarquable/cptHit * 100.f << std::endl;
	ofs << "Remarquable Building Only;" << cptOnlyRemarquable << ";" << cptOnlyRemarquable/cpt * 100.f << std::endl;

	ofs.close();
}

void ExportImage(RayTracingResult result, GlobalData* globalData)
{
	//All image genereted
	QImage imageMurToit(result.width,result.height,QImage::Format::Format_ARGB32);//Wall/Roof Image
	QImage imageZBuffer(result.width,result.height,QImage::Format::Format_ARGB32);//Zbuffer image
	QImage imageObject(result.width,result.height,QImage::Format::Format_ARGB32);//Each city object got a color
	QImage imageBuilding(result.width,result.height,QImage::Format::Format_ARGB32);//Important build in color, the rest in white
	QImage imageRemarquableOnly(result.width,result.height,QImage::Format::Format_ARGB32);//Important build in color, the rest in white
	TVec3d light1 = result.lightDir;
	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			if(!result.hits[i][j].intersect)
			{
				imageMurToit.setPixel(i,j,qRgba(0,0,0,0));
				imageZBuffer.setPixel(i,j,qRgba(255,255,255,0));
				imageObject.setPixel(i,j,qRgba(0,0,0,0));
				imageBuilding.setPixel(i,j,qRgba(0,0,0,0));
			}
			else 
			{
				TVec3d v1 = result.hits[i][j].triangle->a;
				TVec3d v2 = result.hits[i][j].triangle->b;
				TVec3d v3 = result.hits[i][j].triangle->c;
				//Get its normal
				TVec3d normal = Ray::Normalized((v2 - v1).cross(v3 - v1));

				float sundot = (normal.dot(light1) + 1.0)/2.0;

				if(result.hits[i][j].triangle->parent->getType() == citygml::COT_TINRelief)
				{
					imageMurToit.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
					imageZBuffer.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
					imageObject.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
					imageBuilding.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
				}
				else
				{
					if (result.hits[i][j].triangle->object->getType() == citygml::COT_RoofSurface)//Check if roof or wall
					{
						imageMurToit.setPixel(i,j,qRgba(255*sundot,0,0,255));
					}
					else
					{
						imageMurToit.setPixel(i,j,qRgba(205*sundot,183*sundot,158*sundot,255));
					}

					QColor color = globalData->objectToColor[result.hits[i][j].triangle->parent->getId()];

					imageObject.setPixel(i,j,qRgba(color.red()*sundot,color.green()*sundot,color.blue()*sundot,255));//Set the color for the building

					float factor = (globalData->maxDistance - result.hits[i][j].distance)/(globalData->maxDistance - globalData->minDistance);
					factor *= factor;
					imageZBuffer.setPixel(i,j,qRgba(217 * factor, 1 * factor, 21 * factor, 255));//Set the color in the ZBuffer

					QString tempStr(result.hits[i][j].triangle->parent->getId().c_str());
					if(tempStr.startsWith("LYON"))//Check to see if this is an important building
					{
						imageBuilding.setPixel(i,j,qRgba(205*sundot,183*sundot,158*sundot,255));
					}
					else
					{
						imageBuilding.setPixel(i,j,qRgba(color.red()*sundot,color.green()*sundot,color.blue()*sundot,255));
					}
				}
			}

			if(!result.hitsRemarquableBuildingOnly[i][j].intersect)
			{
				imageRemarquableOnly.setPixel(i,j,qRgba(0,0,0,0));
			}
			else 
			{
				TVec3d v1 = result.hitsRemarquableBuildingOnly[i][j].triangle->a;
				TVec3d v2 = result.hitsRemarquableBuildingOnly[i][j].triangle->b;
				TVec3d v3 = result.hitsRemarquableBuildingOnly[i][j].triangle->c;
				//Get its normal
				TVec3d normal = Ray::Normalized((v2 - v1).cross(v3 - v1));

				float sundot = (normal.dot(light1) + 1.0)/2.0;
				QColor color = globalData->objectToColor[result.hitsRemarquableBuildingOnly[i][j].triangle->parent->getId()];
				imageRemarquableOnly.setPixel(i,j,qRgba(color.red()*sundot,color.green()*sundot,color.blue()*sundot,255));
			}
		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageMurToit = imageMurToit.mirrored(false,true);
	imageZBuffer = imageZBuffer.mirrored(false,true);
	imageObject = imageObject.mirrored(false,true);
	imageBuilding = imageBuilding.mirrored(false,true);
	imageRemarquableOnly = imageRemarquableOnly.mirrored(false,true);
	imageMurToit.save("./SkylineOutput/raytraceMurToit.png");
	imageZBuffer.save("./SkylineOutput/raytraceZBuffer.png");
	imageObject.save("./SkylineOutput/raytraceObject.png");
	imageBuilding.save("./SkylineOutput/raytraceBuilding.png");
	imageRemarquableOnly.save("./SkylineOutput/raytraceRemarquableOnly.png");


	std::cout << "Image saved !" << std::endl;
}

std::vector<Triangle*> BuildTriangleList(vcity::Tile* tile, TVec3d offset, GlobalData* globalData )
{
	std::vector<Triangle*> triangles;

	citygml::CityModel * model = tile->getCityModel();
	unsigned int i = 0;
	for(citygml::CityObject* obj : model->getCityObjectsRoots()) //For each city object
	{
		if(obj->getType() == citygml::COT_Building ) //We only take building or terrain
		{
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
							t->object = object;
							t->parent = obj;

							triangles.push_back(t);
						}
					}
				}
			}
		}

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
						t->parent = obj;

						triangles.push_back(t);
					}
				}

			}
		}
	}	

	return triangles;
}


