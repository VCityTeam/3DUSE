#include "Export.hpp"

#include <QImage>
#include <QColor>

#include "core/application.hpp"
#include "export/exportCityGML.hpp"

ExportParameter ExportParameter::globalParam;

std::map<citygml::CityObjectsType,std::string> BuildToStringMap()
{
	std::map<citygml::CityObjectsType,std::string> ToString;
		ToString.insert(std::make_pair(citygml::COT_GenericCityObject         , "GenericCityObject"));  
		ToString.insert(std::make_pair(citygml::COT_Building                  , "Building"));  
		ToString.insert(std::make_pair(citygml::COT_Room                      , "Room"));  
		ToString.insert(std::make_pair(citygml::COT_BuildingInstallation      , "BuildingInstallation"));  
		ToString.insert(std::make_pair(citygml::COT_BuildingFurniture         , "BuildingFurniture"));  
		ToString.insert(std::make_pair(citygml::COT_Door                      , "Door"));  
		ToString.insert(std::make_pair(citygml::COT_Window                    , "Window"));  
		ToString.insert(std::make_pair(citygml::COT_CityFurniture             , "CityFurniture"));  
		ToString.insert(std::make_pair(citygml::COT_Track                     , "Track"));  
		ToString.insert(std::make_pair(citygml::COT_Road                      , "Road"));  
		ToString.insert(std::make_pair(citygml::COT_Railway                   , "Railway"));  
		ToString.insert(std::make_pair(citygml::COT_Square                    , "Square"));  
		ToString.insert(std::make_pair(citygml::COT_PlantCover                , "PlantCover"));  
		ToString.insert(std::make_pair(citygml::COT_SolitaryVegetationObject  , "SolidaryVegetationObject"));  
		ToString.insert(std::make_pair(citygml::COT_WaterBody                 , "WaterBody"));  
		ToString.insert(std::make_pair(citygml::COT_TINRelief                 , "TINRelief"));  
		ToString.insert(std::make_pair(citygml::COT_LandUse                   , "LandUse"));  
		ToString.insert(std::make_pair(citygml::COT_Tunnel					, "Tunnel"));	
		ToString.insert(std::make_pair(citygml::COT_Bridge					, "Bridge"));	
		ToString.insert(std::make_pair(citygml::COT_BridgeConstructionElement	, "BridgeConstructionElement"));
		ToString.insert(std::make_pair(citygml::COT_BridgeInstallation		, "BridgeInstallation"));	
		ToString.insert(std::make_pair(citygml::COT_BridgePart                , "BridgePart"));  
		ToString.insert(std::make_pair(citygml::COT_BuildingPart       		, "BuildingPart"));
		ToString.insert(std::make_pair(citygml::COT_WallSurface               , "WallSurface"));  
		ToString.insert(std::make_pair(citygml::COT_RoofSurface               , "RoofSurface"));  
		ToString.insert(std::make_pair(citygml::COT_GroundSurface             , "GroundSurface"));  
		ToString.insert(std::make_pair(citygml::COT_ClosureSurface            , "ClosureSurface"));  
		ToString.insert(std::make_pair(citygml::COT_FloorSurface              , "FloorSurface"));  
		ToString.insert(std::make_pair(citygml::COT_InteriorWallSurface       , "InteriorWallSurface"));  
		ToString.insert(std::make_pair(citygml::COT_CeilingSurface        	, "CeilingSurface"));
		ToString.insert(std::make_pair(citygml::COT_All                       , "All"));

		return ToString;
}

std::string COTToString(citygml::CityObjectsType type)
{
	static std::map<citygml::CityObjectsType,std::string> ToString = BuildToStringMap();

	return ToString[type];
}

void ExportData(ViewPoint* viewpoint, std::string filePrefix)
{
	float cpt = 0.0f;
	float cptMiss = 0.0f;
	float cptHit = 0.0f;
	float cptRoof = 0.0f;
	float cptWall = 0.0f;
	float cptAllBuilding = 0.0f;
	float cptBuilding = 0.0f;
	float cptRemarquable = 0.0f;
	float cptTerrain = 0.0f;
	float cptWater = 0.0f;
	float cptVegetation = 0.0f;

	EmblematicView view = ExportParameter::GetGlobalParameter().emblematicView;

	std::map<std::string,std::pair<Triangle,unsigned int>> cptMap;

	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			cpt++;
			if(viewpoint->hits[i][j].intersect)
			{
				cptHit++;

				QString tempStr(viewpoint->hits[i][j].triangle.objectId.c_str());
					std::string temp = tempStr.toStdString();
				if(cptMap.find(temp)  == cptMap.end())
						cptMap.insert(std::make_pair(temp,std::make_pair(viewpoint->hits[i][j].triangle,1)));
					else
						cptMap[temp].second++;

				if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_Building)
				{
					QString tempStr(viewpoint->hits[i][j].triangle.objectId.c_str());
					std::string temp = tempStr.toStdString();
					
					cptAllBuilding++;

					if(viewpoint->hits[i][j].triangle.subObjectType == citygml::COT_RoofSurface)
						cptRoof++;
					else
						cptWall++;


					if(tempStr.startsWith("LYON"))//Check to see if this is an important building
						cptBuilding++;
					else
						cptRemarquable++;
				}
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_TINRelief)
				{
					cptTerrain++;
				}
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_WaterBody)
				{
					cptWater++;
				}
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_SolitaryVegetationObject)
				{
					cptVegetation++;
				}
			}
			else
				cptMiss++;
		}
	}

	std::ofstream ofs;
	ofs.open ("./SkylineOutput/"+filePrefix+"result.csv", std::ofstream::out);

	ofs << "Param;Count;Global%;InHit%;InBuilding%" << std::endl;
	ofs << "Pixel;"<<cpt << ";100%" << std::endl;
	ofs << "Hit;"<< cptHit << ";" << cptHit/cpt * 100.f << std::endl;
	ofs << "Miss;"<<cptMiss << ";" << cptMiss/cpt * 100.f << std::endl;
	ofs << "Terrain;"<<cptTerrain << ";" << cptTerrain/cpt * 100.f << ";" << cptTerrain/cptHit * 100.f << std::endl;
	ofs << "Vegetation;"<<cptVegetation << ";" << cptVegetation/cpt * 100.f << ";" << cptVegetation/cptHit * 100.f << std::endl;
	ofs << "Water;"<<cptWater << ";" << cptWater/cpt * 100.f << ";" << cptWater/cptHit * 100.f << std::endl;
	ofs << "All Building;"<<cptAllBuilding << ";" << cptAllBuilding/cpt * 100.f << ";" << cptAllBuilding/cptHit * 100.f << std::endl;
	ofs << "Roof;" << cptRoof << ";" << cptRoof/cpt * 100.f << ";" << cptRoof/cptHit * 100.f << std::endl;
	ofs << "Wall;" << cptWall << ";" << cptWall/cpt * 100.f << ";" << cptWall/cptHit * 100.f << std::endl;
	ofs << "Building Misc;" << cptBuilding << ";" << cptBuilding/cpt * 100.f << ";" << cptBuilding/cptHit * 100.f <<  ";" << cptBuilding / cptAllBuilding * 100.f<<  std::endl;
	ofs << "Remarquable Building;" << cptRemarquable << ";" << cptRemarquable/cpt * 100.f << ";" << cptRemarquable/cptHit * 100.f << ";" << cptRemarquable / cptAllBuilding * 100.f << std::endl;
	ofs << std::endl;
	ofs << "Emblematic View : ;% In Viewpoint;% In emblematic view;% Difference" << std::endl;
	ofs << "Ciel;" << (cptMiss/cpt * 100.f)  << ";" << view.sky << ";" << (cptMiss/cpt * 100.f) - view.sky << std::endl;
	ofs << "Terrain;" << (cptTerrain/cpt * 100.f)  << ";" << view.terrain << ";" << (cptTerrain/cpt * 100.f) - view.terrain << std::endl;
	ofs << "Vegetation;" << (cptVegetation/cpt * 100.f)<< ";" << view.vegetation << ";" << (cptVegetation/cpt * 100.f) - view.vegetation<< std::endl;
	ofs << "Water;"<< (cptWater/cpt * 100.f)<< ";" << view.water << ";" << (cptWater/cpt * 100.f) - view.water<< std::endl;
	ofs << "Building Misc;" << (cptBuilding/cpt * 100.f)<< ";" << view.building << ";" << (cptBuilding/cpt * 100.f) - view.building<<  std::endl;
	ofs << "Remarquable Building;" << (cptRemarquable/cpt * 100.f)<< ";" << view.remarquableBuilding << ";" << (cptRemarquable/cpt * 100.f) - view.remarquableBuilding << std::endl;
	ofs << std::endl;
	ofs << "ObjectId;Count;Global%;InHit%;ObjectType;SubObjectType" << std::endl;
	for(auto it = cptMap.begin(); it != cptMap.end(); it++)
	{
		if(it->second.first.objectType == citygml::COT_Building)
			ofs << it->first << ";" << it->second.second << ";" << it->second.second/cpt * 100.f << ";" << it->second.second/cptHit * 100.f  << ";" << COTToString(it->second.first.objectType) << ";" << COTToString(it->second.first.subObjectType) << std::endl;
		else if(it->second.first.objectType == citygml::COT_SolitaryVegetationObject)
		{
			if(it->second.first.objectId.find("_tree") != std::string::npos)
			{
				ofs << it->second.first.objectId.substr(0,it->second.first.objectId.size()-5) << ";" << it->second.second << ";" << it->second.second/cpt * 100.f << ";" << it->second.second/cptHit * 100.f  << ";" << COTToString(it->second.first.objectType) << ";" << "Alignement Tree" << std::endl;
			}
			else
				ofs << it->first << ";" << it->second.second << ";" << it->second.second/cpt * 100.f << ";" << it->second.second/cptHit * 100.f  << ";" << COTToString(it->second.first.objectType) << ";" << "Park" << std::endl;
		}
		else
			ofs << it->first << ";" << it->second.second << ";" << it->second.second/cpt * 100.f << ";" << it->second.second/cptHit * 100.f  << ";" << COTToString(it->second.first.objectType) << std::endl;
	}


	ofs.close();

	

	ofs.open ("./SkylineOutput/"+filePrefix+"skyline.csv", std::ofstream::out);
	ofs << viewpoint->skyline.Points.size() << std::endl;
	for(unsigned int i = 0; i < viewpoint->skyline.FragCoords.size(); i++)
	{
		TVec3d p = viewpoint->skyline.Points[i];
		std::pair<unsigned int,unsigned int> frag = viewpoint->skyline.FragCoords[i];
		Hit h = viewpoint->hits[frag.first][frag.second];
		if(h.triangle.objectType == citygml::COT_Building)
			ofs << p.x << ";" << p.y << ";" << p.z << ";" << h.triangle.objectId << ";"  << COTToString(h.triangle.objectType) << ";" << COTToString(h.triangle.subObjectType) << std::endl;
		else if(h.triangle.objectType == citygml::COT_SolitaryVegetationObject)
		{
			if(h.triangle.objectId.find("_tree") != std::string::npos)
			{
				ofs << p.x << ";" << p.y << ";" << p.z << ";" << h.triangle.objectId.substr(0,h.triangle.objectId.size()-5) << ";"  << COTToString(h.triangle.objectType) << ";" << "Alignement Tree" << std::endl;
			}
			else
				ofs << p.x << ";" << p.y << ";" << p.z << ";" << h.triangle.objectId << ";"  << COTToString(h.triangle.objectType) << ";" << "Park" << std::endl;
		}
		else
			ofs << p.x << ";" << p.y << ";" << p.z << ";" << h.triangle.objectId << ";"  << COTToString(h.triangle.objectType) << ";" << COTToString(h.triangle.subObjectType) << std::endl;
	}
	ofs.close();
}

TVec3d SkyShadeBlue(TVec3d d, TVec3d light)
{
  	// light direction
	TVec3d lig = light;
	float sun = (lig.dot(d)+1.0)/2.0;
	TVec3d color = TVec3d(0.35,0.45,0.75)*(0.75-0.5*d[2]);
	color = color + TVec3d(0.65,0.6,0.55)*pow( sun, 8.0 );
	return color;
}

void ExportImageRoofWall(ViewPoint* viewpoint, std::string filePrefix)
{
	QImage imageMurToit(viewpoint->width,viewpoint->height,QImage::Format::Format_ARGB32);//Wall/Roof Image
	TVec3d light1 = viewpoint->lightDir;
	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			if(!viewpoint->hits[i][j].intersect)
			{
				TVec3d col = SkyShadeBlue(viewpoint->hits[i][j].ray.dir,viewpoint->lightDir);
				imageMurToit.setPixel(i,j,qRgba(col.r*255,col.g*255,col.b*255,255));
			}
			else 
			{
				//Get its normal
				TVec3d normal = viewpoint->hits[i][j].triangle.GetNormal();

				float sundot = (normal.dot(light1) + 1.0)/2.0;
				if(sundot < 0.2)
				{
					sundot = (normal.dot(-light1) + 1.0)/2.0;
				}

				if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_TINRelief)
					imageMurToit.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255));
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_WaterBody)
					imageMurToit.setPixel(i,j,qRgba(0*sundot,0*sundot, 255*sundot,255));
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_SolitaryVegetationObject)
					imageMurToit.setPixel(i,j,qRgba(0*sundot,255*sundot,0*sundot,255));
				else
				{
					if (viewpoint->hits[i][j].triangle.subObjectType == citygml::COT_RoofSurface)//Check if roof or wall
						imageMurToit.setPixel(i,j,qRgba(255*sundot,0,0,255));
					else
						imageMurToit.setPixel(i,j,qRgba(205*sundot,183*sundot,158*sundot,255));
				}
			}
		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageMurToit = imageMurToit.mirrored(false,true);
	imageMurToit.save(std::string("./SkylineOutput/"+filePrefix+"raytraceMurToit.png").c_str());
}

void ExportImageZBuffer(ViewPoint* viewpoint, std::string filePrefix)
{
	QImage imageZBuffer(viewpoint->width,viewpoint->height,QImage::Format::Format_ARGB32);//Zbuffer image
	TVec3d light1 = viewpoint->lightDir;
	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			if(!viewpoint->hits[i][j].intersect)
			{
				imageZBuffer.setPixel(i,j,qRgba(255,255,255,255));
			}
			else 
			{
				float factor = (viewpoint->maxDistance - viewpoint->hits[i][j].distance)/(viewpoint->maxDistance - viewpoint->minDistance);
				factor *= factor;
				imageZBuffer.setPixel(i,j,qRgba(217 * factor, 1 * factor, 21 * factor, 255));//Set the color in the ZBuffer
			}

		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageZBuffer = imageZBuffer.mirrored(false,true);
	imageZBuffer.save(std::string("./SkylineOutput/"+filePrefix+"raytraceZBuffer.png").c_str());
}

void ExportImageObjectColor(ViewPoint* viewpoint, std::string filePrefix)
{
	QImage imageObject(viewpoint->width,viewpoint->height,QImage::Format::Format_ARGB32);//Each city object got a color
	TVec3d light1 = viewpoint->lightDir;
	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			if(!viewpoint->hits[i][j].intersect)
			{
				TVec3d col = SkyShadeBlue(viewpoint->hits[i][j].ray.dir,viewpoint->lightDir);
				imageObject.setPixel(i,j,qRgba(col.r*255,col.g*255,col.b*255,255));
			}
			else 
			{
				//Get its normal
				TVec3d normal = viewpoint->hits[i][j].triangle.GetNormal();

				float sundot = (normal.dot(light1) + 1.0)/2.0;
				if(sundot < 0.2)
				{
					sundot = (normal.dot(-light1) + 1.0)/2.0;
				}

				if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_TINRelief)
					imageObject.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255));
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_WaterBody)
					imageObject.setPixel(i,j,qRgba(0*sundot,0*sundot, 255*sundot,255));
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_SolitaryVegetationObject)
					imageObject.setPixel(i,j,qRgba(0*sundot,255*sundot,0*sundot,255));
				else
				{
					QColor color = viewpoint->objectToColor[viewpoint->hits[i][j].triangle.objectId];
					imageObject.setPixel(i,j,qRgba(color.red()*sundot,color.green()*sundot,color.blue()*sundot,255));//Set the color for the building
				}
			}

		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageObject = imageObject.mirrored(false,true);
	imageObject.save(std::string("./SkylineOutput/"+filePrefix+"raytraceObject.png").c_str());
}

void ExportImageHighlightRemarquable(ViewPoint* viewpoint, std::string filePrefix)
{
	QImage imageBuilding(viewpoint->width,viewpoint->height,QImage::Format::Format_ARGB32);//Important build in color, the rest in white
	TVec3d light1 = viewpoint->lightDir;
	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			if(!viewpoint->hits[i][j].intersect)
			{
				TVec3d col = SkyShadeBlue(viewpoint->hits[i][j].ray.dir,viewpoint->lightDir);
				imageBuilding.setPixel(i,j,qRgba(col.r*255,col.g*255,col.b*255,255));
			}
			else 
			{
				//Get its normal
				TVec3d normal = viewpoint->hits[i][j].triangle.GetNormal();

				float sundot = (normal.dot(light1) + 1.0)/2.0;
				if(sundot < 0.2)
				{
					sundot = (normal.dot(-light1) + 1.0)/2.0;
				}

				if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_TINRelief)
					imageBuilding.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255));
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_WaterBody)
					imageBuilding.setPixel(i,j,qRgba(0*sundot,0*sundot, 255*sundot,255));
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_SolitaryVegetationObject)
					imageBuilding.setPixel(i,j,qRgba(0*sundot,255*sundot,0*sundot,255));
				else
				{
					QColor color = viewpoint->objectToColor[viewpoint->hits[i][j].triangle.objectId];

					QString tempStr(viewpoint->hits[i][j].triangle.objectId.c_str());
					if(tempStr.startsWith("LYON"))//Check to see if this is an important building
						imageBuilding.setPixel(i,j,qRgba(205*sundot,183*sundot,158*sundot,255));
					else
						imageBuilding.setPixel(i,j,qRgba(color.red()*sundot,color.green()*sundot,color.blue()*sundot,255));
				}
			}
		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageBuilding = imageBuilding.mirrored(false,true);
	imageBuilding.save(std::string("./SkylineOutput/"+filePrefix+"raytraceBuilding.png").c_str());
}

void ExportImageSkyline(ViewPoint* viewpoint, std::string filePrefix)
{
	QImage imageSkyline(viewpoint->width,viewpoint->height,QImage::Format::Format_ARGB32);

	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			if(!viewpoint->hits[i][j].intersect)
			{
				imageSkyline.setPixel(i,j,qRgba(0,0,0,255));
			}
			else
			{
				imageSkyline.setPixel(i,j,qRgba(255,255,255,255));
			}
		}
	}

	for(unsigned int i = 0; i < viewpoint->skyline.FragCoords.size(); i++)
	{
		unsigned int x = viewpoint->skyline.FragCoords[i].first;
		unsigned int y = viewpoint->skyline.FragCoords[i].second;
		imageSkyline.setPixel(x,y,qRgba(255,0,0,255));
	}

	//Images needs to be mirrored because of qt's way of storing them
	imageSkyline = imageSkyline.mirrored(false,true);
	imageSkyline.save(std::string("./SkylineOutput/"+filePrefix+"raytraceSkyline.png").c_str());
}

void ExportImages(ViewPoint* viewpoint, std::string filePrefix)
{
	std::cout << "Saving image." << std::endl;

	ExportImageRoofWall(viewpoint,filePrefix);
	ExportImageZBuffer(viewpoint,filePrefix);
	ExportImageObjectColor(viewpoint,filePrefix);
	ExportImageHighlightRemarquable(viewpoint,filePrefix);
	ExportImageSkyline(viewpoint,filePrefix);


	std::cout << "Image saved !" << std::endl;
}

void ExportPanoramaSkyline(ViewPoint* front, ViewPoint* right, ViewPoint* back, ViewPoint* left, std::string filePrefix)
{
	std::ofstream ofs;
	ofs.open ("./SkylineOutput/"+filePrefix+"skyline.txt", std::ofstream::out);

	unsigned int cpt = front->skyline.FragCoords.size()+right->skyline.FragCoords.size()+back->skyline.FragCoords.size()+left->skyline.FragCoords.size();

	ofs << cpt << "\n";

	for(unsigned int i = 0; i < front->skyline.FragCoords.size(); i++)
	{
		ofs << front->skyline.FragCoords[i].first << "\n";
		ofs << front->skyline.FragCoords[i].second << "\n";
		ofs << front->skyline.Points[i].x << "\n";
		ofs << front->skyline.Points[i].y << "\n";
		ofs << front->skyline.Points[i].z << "\n";
	}

	for(unsigned int i = 0; i < right->skyline.FragCoords.size(); i++)
	{
		ofs << right->skyline.FragCoords[i].first << "\n";
		ofs << right->skyline.FragCoords[i].second << "\n";
		ofs << right->skyline.Points[i].x << "\n";
		ofs << right->skyline.Points[i].y << "\n";
		ofs << right->skyline.Points[i].z << "\n";
	}
	for(unsigned int i = 0; i < back->skyline.FragCoords.size(); i++)
	{
		ofs << back->skyline.FragCoords[i].first << "\n";
		ofs << back->skyline.FragCoords[i].second << "\n";
		ofs << back->skyline.Points[i].x << "\n";
		ofs << back->skyline.Points[i].y << "\n";
		ofs << back->skyline.Points[i].z << "\n";
	}
	for(unsigned int i = 0; i < left->skyline.FragCoords.size(); i++)
	{
		ofs << left->skyline.FragCoords[i].first << "\n";
		ofs << left->skyline.FragCoords[i].second << "\n";
		ofs << left->skyline.Points[i].x << "\n";
		ofs << left->skyline.Points[i].y << "\n";
		ofs << left->skyline.Points[i].z << "\n";
	}

	ofs.close();
}

bool functionCompareTVec3Z(TVec3d a, TVec3d b) { return a.z < b.z; }

void ProcessSkylineVolume()
{
	TVec3d offset = vcity::app().getSettings().getDataProfile().m_offset;

	std::cout << "Processing Plan..." << std::endl;
	for(unsigned int i = 0; i < 10; i++)
	{
		std::vector<TVec3d> points;
		char line[256];

		std::ifstream ifs ("C:/VCityBuild/SkylineTest/"+std::to_string(i)+"_skyline.txt", std::ifstream::in);

		ifs.getline(line,256);

		unsigned int count = atoi(line);

		for(unsigned int j = 0; j < count; j++)
		{
			double tx;
			double ty;
			double x;
			double y;
			double z;

			ifs.getline(line,256);tx = atof(line);
			ifs.getline(line,256);ty = atof(line);
			ifs.getline(line,256);x = atof(line);
			ifs.getline(line,256);y = atof(line);
			ifs.getline(line,256);z = atof(line);
			points.push_back(TVec3d(x,y,z)+offset);
		}

		std::string name = std::to_string(i)+"_skyline";

		citygml::CityModel* ModelOut = new citygml::CityModel;

		citygml::CityObject* BuildingCO = new citygml::Building(name);

		citygml::CityObject* RoofCO = new citygml::RoofSurface(name+"_Roof");
		citygml::Geometry* Roof = new citygml::Geometry(name+"_RoofGeometry", citygml::GT_Roof, 2);

		citygml::LinearRing* ring = new citygml::LinearRing(name+"_ring",true);
		for(unsigned int j = 0; j < points.size(); j++)
		{
			ring->addVertex(points[j]);
		}

		citygml::Polygon* poly = new citygml::Polygon(name+"_poly");
		poly->addRing(ring);

		Roof->addPolygon(poly);

		RoofCO->addGeometry(Roof);
		ModelOut->addCityObject(RoofCO);
		BuildingCO->insertNode(RoofCO);

		ModelOut->addCityObject(BuildingCO);
		ModelOut->addCityObjectAsRoot(BuildingCO);

		ModelOut->computeEnvelope();

		citygml::ExporterCityGML exporter("./ShpExtruded/"+name+".gml");
		exporter.exportCityModel(*ModelOut);
	}
	std::cout << "Done." << std::endl;

	std::cout << "Processing Volume..." << std::endl;

	std::vector<std::vector<TVec3d>> layers;

	for(unsigned int i = 0; i < 10; i++)
	{
		int lastCoord = -1;
		std::vector<TVec3d> points;
		char line[256];

		std::ifstream ifs ("C:/VCityBuild/SkylineTest/"+std::to_string(i)+"_skyline.txt", std::ifstream::in);

		ifs.getline(line,256);

		unsigned int count = atoi(line);

		for(unsigned int j = 0; j < count; j++)
		{
			int tx;
			int ty;
			double x;
			double y;
			double z;

			ifs.getline(line,256);tx = atoi(line);
			ifs.getline(line,256);ty = atoi(line);
			ifs.getline(line,256);x = atof(line);
			ifs.getline(line,256);y = atof(line);
			ifs.getline(line,256);z = atof(line);

			if(tx > lastCoord)
			{
				points.push_back(TVec3d(x,y,z)+offset);
				lastCoord = tx;
			}
			else if(tx == 0)
			{
				points.push_back(TVec3d(x,y,z)+offset);
				lastCoord = -1;
			}
		}

		layers.push_back(points);
	}

	for(unsigned int i = 0; i < layers.front().size(); i++)
	{
		std::vector<TVec3d> temp;

		for(unsigned int j = 0; j < layers.size(); j++)
		{
			temp.push_back(layers[j][i]);
		}

		std::sort(temp.begin(),temp.end(), functionCompareTVec3Z);

		for(unsigned int j = 0; j < layers.size(); j++)
		{
			layers[j][i] = temp[j];
		}
	}

	{
		std::string name = "_skyline";

		citygml::CityModel* ModelOut = new citygml::CityModel;

		citygml::CityObject* BuildingCO = new citygml::Building(name);

		citygml::CityObject* WallCO = new citygml::RoofSurface(name+"_Wall");
		citygml::Geometry* Wall = new citygml::Geometry(name+"_WallGeometry", citygml::GT_Wall, 2);

		for(unsigned int i = 0; i < layers.front().size(); i++)
		{
			if(i == layers.front().size()-1)
				std::cout << "incheckcheck" << std::endl;
			for(unsigned int j = 0; j < layers.size(); j++)
			{
				if(j == layers.size()-1)
					std::cout << "incheck" << std::endl;
				citygml::LinearRing* ring = new citygml::LinearRing(name+"_ring",true);

				ring->addVertex(layers[j][i]);
				ring->addVertex(layers[j][(i+1) % layers.front().size()]);
				ring->addVertex(layers[(j+1) % layers.size()][(i+1) % layers.front().size()]);
				ring->addVertex(layers[(j+1) % layers.size()][i]);

				citygml::Polygon* poly = new citygml::Polygon(name+"_poly");
				poly->addRing(ring);
				Wall->addPolygon(poly);
				if(j == layers.size()-1)
					std::cout << "outcheck" << std::endl;
			}
			if(i == layers.front().size()-1)
				std::cout << "outcheckcheck" << std::endl;
		}

		WallCO->addGeometry(Wall);
		ModelOut->addCityObject(WallCO);
		BuildingCO->insertNode(WallCO);


		ModelOut->addCityObject(BuildingCO);
		ModelOut->addCityObjectAsRoot(BuildingCO);

		ModelOut->computeEnvelope();

		citygml::ExporterCityGML exporter("./ShpExtruded/"+name+".gml");
		exporter.exportCityModel(*ModelOut);
	}

	std::cout << "Done." << std::endl;
}