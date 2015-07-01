#include "Export.hpp"

#include <QImage>
#include <QColor>

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

	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			cpt++;
			if(viewpoint->hits[i][j].intersect)
			{
				cptHit++;

				if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_Building)
				{
					cptAllBuilding++;

					if(viewpoint->hits[i][j].triangle.subObjectType == citygml::COT_RoofSurface)
						cptRoof++;
					else
						cptWall++;

					QString tempStr(viewpoint->hits[i][j].triangle.objectId.c_str());
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
	ofs << "Water;"<<cptWater << ";" << cptWater/cpt * 100.f << ";" << cptWater/cptHit * 100.f << std::endl;
	ofs << "All Building;"<<cptAllBuilding << ";" << cptAllBuilding/cpt * 100.f << ";" << cptAllBuilding/cptHit * 100.f << std::endl;
	ofs << "Roof;" << cptRoof << ";" << cptRoof/cpt * 100.f << ";" << cptRoof/cptHit * 100.f << std::endl;
	ofs << "Wall;" << cptWall << ";" << cptWall/cpt * 100.f << ";" << cptWall/cptHit * 100.f << std::endl;
	ofs << "Building Misc;" << cptBuilding << ";" << cptBuilding/cpt * 100.f << ";" << cptBuilding/cptHit * 100.f <<  ";" << cptBuilding / cptAllBuilding * 100.f<<  std::endl;
	ofs << "Remarquable Building;" << cptRemarquable << ";" << cptRemarquable/cpt * 100.f << ";" << cptRemarquable/cptHit * 100.f << ";" << cptRemarquable / cptAllBuilding * 100.f << std::endl;

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

void ExportImageTest(ViewPoint* viewpoint, std::string filePrefix)
{
	QImage imageMurToit(viewpoint->width,viewpoint->height,QImage::Format::Format_ARGB32);//Wall/Roof Image
	TVec3d light1 = viewpoint->lightDir;
	float min = FLT_MAX;
	float max = -FLT_MAX;

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
				min = std::min(min,sundot);
				max = std::max(max,sundot);

				if(sundot < 0.2)
				{
					sundot = (normal.dot(-light1) + 1.0)/2.0;
				}

				if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_TINRelief)
				{
					TVec3d col = SkyShadeBlue(viewpoint->hits[i][j].ray.dir,viewpoint->lightDir);
					imageMurToit.setPixel(i,j,qRgba(col.r*255,col.g*255,col.b*255,255));
				}
				else if(viewpoint->hits[i][j].triangle.objectType == citygml::COT_WaterBody)
					imageMurToit.setPixel(i,j,qRgba(0*sundot,0*sundot, 255*sundot,255));
				else
				{
						imageMurToit.setPixel(i,j,qRgba(255*sundot,255*sundot,255*sundot,255));
				}
			}
		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageMurToit = imageMurToit.mirrored(false,true);
	imageMurToit.save(std::string("./SkylineOutput/"+filePrefix+"raytraceTest.png").c_str());

	std::cout << "Min : " << min << " | " << "Max : " << max << std::endl;
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

	for(unsigned int i = 0; i < viewpoint->skyline.size(); i++)
	{
		unsigned int x = viewpoint->skyline[i].first;
		unsigned int y = viewpoint->skyline[i].second;
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
	//ExportImageTest(viewpoint,filePrefix);


	std::cout << "Image saved !" << std::endl;
}