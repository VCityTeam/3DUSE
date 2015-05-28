#include "Export.hpp"

#include <QImage>
#include <QColor>

void ExportData(GlobalData* globalData, RayTracingResult result)
{
	float cpt = 0.0f;
	float cptMiss = 0.0f;
	float cptHit = 0.0f;
	float cptRoof = 0.0f;
	float cptWall = 0.0f;
	float cptBuilding = 0.0f;
	float cptRemarquable = 0.0f;

	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			cpt++;
			if(result.hits[i][j].intersect)
			{
				cptHit++;

				if(result.hits[i][j].triangle->object->getType() == citygml::COT_Building)
				{
					if(result.hits[i][j].triangle->subObject->getType() == citygml::COT_RoofSurface)
						cptRoof++;
					else
						cptWall++;

					QString tempStr(result.hits[i][j].triangle->object->getId().c_str());
					if(tempStr.startsWith("LYON"))//Check to see if this is an important building
						cptBuilding++;
					else
						cptRemarquable++;
				}
			}
			else
				cptMiss++;
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

	ofs.close();
}

void ExportImageRoofWall(GlobalData* globalData, RayTracingResult result)
{
	QImage imageMurToit(result.width,result.height,QImage::Format::Format_ARGB32);//Wall/Roof Image
	TVec3d light1 = result.lightDir;
	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			if(!result.hits[i][j].intersect)
				imageMurToit.setPixel(i,j,qRgba(0,0,0,0));
			else 
			{
				//Get its normal
				TVec3d normal = result.hits[i][j].triangle->GetNormal();

				float sundot = (normal.dot(light1) + 1.0)/2.0;

				if(result.hits[i][j].triangle->object->getType() == citygml::COT_TINRelief)
					imageMurToit.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
				else
				{
					if (result.hits[i][j].triangle->subObject->getType() == citygml::COT_RoofSurface)//Check if roof or wall
						imageMurToit.setPixel(i,j,qRgba(255*sundot,0,0,255));
					else
						imageMurToit.setPixel(i,j,qRgba(205*sundot,183*sundot,158*sundot,255));
				}
			}
		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageMurToit = imageMurToit.mirrored(false,true);
	imageMurToit.save("./SkylineOutput/raytraceMurToit.png");
}

void ExportImageZBuffer(GlobalData* globalData, RayTracingResult result)
{
	QImage imageZBuffer(result.width,result.height,QImage::Format::Format_ARGB32);//Zbuffer image
	TVec3d light1 = result.lightDir;
	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			if(!result.hits[i][j].intersect)
			{
				imageZBuffer.setPixel(i,j,qRgba(255,255,255,0));
			}
			else 
			{
				float factor = (globalData->maxDistance - result.hits[i][j].distance)/(globalData->maxDistance - globalData->minDistance);
				factor *= factor;
				imageZBuffer.setPixel(i,j,qRgba(217 * factor, 1 * factor, 21 * factor, 255));//Set the color in the ZBuffer
			}

		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageZBuffer = imageZBuffer.mirrored(false,true);
	imageZBuffer.save("./SkylineOutput/raytraceZBuffer.png");
}

void ExportImageObjectColor(GlobalData* globalData, RayTracingResult result)
{
	QImage imageObject(result.width,result.height,QImage::Format::Format_ARGB32);//Each city object got a color
	TVec3d light1 = result.lightDir;
	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			if(!result.hits[i][j].intersect)
				imageObject.setPixel(i,j,qRgba(0,0,0,0));
			else 
			{
				TVec3d v1 = result.hits[i][j].triangle->a;
				TVec3d v2 = result.hits[i][j].triangle->b;
				TVec3d v3 = result.hits[i][j].triangle->c;
				//Get its normal
				TVec3d normal = Ray::Normalized((v2 - v1).cross(v3 - v1));

				float sundot = (normal.dot(light1) + 1.0)/2.0;

				if(result.hits[i][j].triangle->object->getType() == citygml::COT_TINRelief)
					imageObject.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
				else
				{
					QColor color = globalData->objectToColor[result.hits[i][j].triangle->object->getId()];
					imageObject.setPixel(i,j,qRgba(color.red()*sundot,color.green()*sundot,color.blue()*sundot,255));//Set the color for the building
				}
			}

		}
	}
	//Images needs to be mirrored because of qt's way of storing them
	imageObject = imageObject.mirrored(false,true);
	imageObject.save("./SkylineOutput/raytraceObject.png");
}

void ExportImageHighlightRemarquable(GlobalData* globalData, RayTracingResult result)
{
	QImage imageBuilding(result.width,result.height,QImage::Format::Format_ARGB32);//Important build in color, the rest in white
	TVec3d light1 = result.lightDir;
	for(unsigned int i = 0; i < result.width; i++)
	{
		for(unsigned int j = 0; j < result.height; j++)
		{
			if(!result.hits[i][j].intersect)
			{
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

				if(result.hits[i][j].triangle->object->getType() == citygml::COT_TINRelief)
					imageBuilding.setPixel(i,j,qRgba(78*sundot,61*sundot, 40*sundot,255*sundot));
				else
				{
					QColor color = globalData->objectToColor[result.hits[i][j].triangle->object->getId()];

					QString tempStr(result.hits[i][j].triangle->object->getId().c_str());
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
	imageBuilding.save("./SkylineOutput/raytraceBuilding.png");
}

void ExportImages(GlobalData* globalData,RayTracingResult result)
{
	ExportImageRoofWall(globalData,result);
	ExportImageZBuffer(globalData,result);
	ExportImageObjectColor(globalData,result);
	ExportImageHighlightRemarquable(globalData,result);


	std::cout << "Image saved !" << std::endl;
}