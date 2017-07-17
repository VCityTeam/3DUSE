// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "BelvedereDB.h"

#include <qfileinfo.h>
#include <qdir.h>
#include <cstdio>
#include <algorithm>

BelvedereDB BelvedereDB::instance;

BelvedereDB::~BelvedereDB()
{

}

BelvedereDB::BelvedereDB()
{

}

void BelvedereDB::Setup(std::string dirTile, std::string label, double deltaDistance)
{
	this->dirTile = dirTile;
	this->label = label;
	this->deltaDistance = deltaDistance;

	if(label != "")
	{
		QFileInfo info(dirTile.c_str());
		globalDataFilename = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_globalBVDB.dat";// File containing global data

		QFileInfo globalFileInfo(globalDataFilename.c_str());
		if(globalFileInfo.exists())//Check if it exists
		{
			char line[256];

			std::ifstream ifs (globalDataFilename, std::ifstream::in);

			ifs.getline(line,256);

			unsigned int count = atoi(line);
			global.ViewpointCaptureCount = count;//Get how many viewpoint capture we did

			for(unsigned int j = 0; j < count; j++)//Get all viewpoint position
			{
				double x;
				double y;
				double z;

				ifs.getline(line,256);x = atof(line);
				ifs.getline(line,256);y = atof(line);
				ifs.getline(line,256);z = atof(line);

				global.Viewpoints.push_back(TVec3d(x,y,z));
			}

			ifs.close();
		}
	}
}

void BelvedereDB::ExportViewpointData(ViewPoint* viewpoint)
{
	if(label != "")
	{
		bool foundSameViewpoint = false;

		for(TVec3d vec : global.Viewpoints)//We check that we are not too close to another viewpoint
		{
			if((viewpoint->position - vec).length() <= deltaDistance)
			{
				foundSameViewpoint = true;
				break;
			}
		}

		if(!foundSameViewpoint)
		{
			global.ViewpointCaptureCount++;
			global.Viewpoints.push_back(viewpoint->position);
			std::ofstream ofs (globalDataFilename, std::ofstream::out);//We rewrite the global data file to include the new viewpoint

			ofs << global.ViewpointCaptureCount << std::endl;

			for(TVec3d vec : global.Viewpoints)
				ofs << std::fixed << vec.x << std::endl << std::fixed << vec.y << std::endl << std::fixed << vec.z << std::endl;

			ofs.close();

			std::map<std::string,std::map<std::string,PolygonData>> tilePolyData;// Data about polygon hit in our capture, key = path of the tile, value = ( key = polygon id, value = polygon data)

			for(unsigned int i = 0; i < viewpoint->width; i++)//For every "pixel" get the polygon data
			{
				for(unsigned int j = 0; j < viewpoint->height; j++)
				{
					if(viewpoint->hits[i][j].intersect)
					{
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.polygonId].HitCount++;
            // Dans tilePolyData, creer element
            //    "viewpoint->hits[i][j].triangle.tileFile"
            // dans la premiere map, qui est lui meme une map et on lui cree
            // alors un element
            //    "viewpoint->hits[i][j].triangle.polygonId"
            // qui est un PolygonData et que l'on va donc remplir.
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.polygonId].CityObjectId = viewpoint->hits[i][j].triangle.objectId;
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.polygonId].PolygonId = viewpoint->hits[i][j].triangle.polygonId;
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.polygonId].Tile = viewpoint->hits[i][j].triangle.tileFile;
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.polygonId].Viewpoints.push_back(viewpoint->position);
					}
				}
			}

			for(auto it = tilePolyData.begin(); it != tilePolyData.end(); it++)//We are going to update the actual db with new data
			{
				QFileInfo info(it->first.c_str());
				std::string tileDBFileName = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_"+info.baseName().toStdString()+".dat";//We build the path to the db file from the tile path

				std::map<std::string,PolygonData> existingDB = GetTileData(it->first);//Get the current database of that tile
				std::map<std::string,PolygonData> currentDB = it->second;//Our new data

				for(auto ib = currentDB.begin(); ib != currentDB.end(); ib++)//Add or update data about a polygon
				{
					existingDB[ib->first].HitCount++;
					existingDB[ib->first].CityObjectId = ib->second.CityObjectId;
					existingDB[ib->first].PolygonId = ib->second.PolygonId;
					existingDB[ib->first].Tile = ib->second.Tile;
					existingDB[ib->first].Viewpoints.push_back(ib->second.Viewpoints.front());
				}

				ExportTileData(it->first,existingDB);//Write it on disk
			}
		}
	}
}

std::map<std::string,PolygonData> BelvedereDB::GetTileData(std::string tilePath)
{
	QFileInfo info(tilePath.c_str());
	std::string tileDBFileName = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_"+info.baseName().toStdString()+".dat";//We build the path to the db file from the tile path

	if(QFileInfo(tileDBFileName.c_str()).exists())
	{
		std::ifstream ifs (tileDBFileName, std::ifstream::in);

		char line[256];
		ifs.getline(line,256);

		unsigned int count = atoi(line);//Get how many polygon have been seen in that tile

		std::map<std::string,PolygonData> result;

		for(unsigned int i = 0; i < count; i++)
		{
			ifs.getline(line,256);
			std::string name = std::string(line);//polygon id
			ifs.getline(line,256);
			std::string cityObjectId = std::string(line);// id of the polygon city object
			ifs.getline(line,256);
			unsigned int cpt = atoi(line);//How many time it has been seen
			PolygonData data;
			data.HitCount = cpt;
			data.CityObjectId = cityObjectId;

			for(unsigned int j = 0; j < cpt; j++)//Get all viewpoint that have seen the polygon
			{
				double x;
				double y;
				double z;

				ifs.getline(line,256);x = atof(line);
				ifs.getline(line,256);y = atof(line);
				ifs.getline(line,256);z = atof(line);

				data.Viewpoints.push_back(TVec3d(x,y,z));
			}

			data.Tile = info.absoluteFilePath().toStdString();
			data.PolygonId = name;

			result[name] = data;
		}

		ifs.close();

		return result;
	}
	else 
	{
		return std::map<std::string,PolygonData>();
	}
}

void BelvedereDB::ExportTileData(std::string tilePath, std::map<std::string,PolygonData> data)
{
	//See GetTileData
	QFileInfo info(tilePath.c_str());
	std::string tileDBFileName = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_"+info.baseName().toStdString()+".dat";

	std::ofstream ofs (tileDBFileName, std::ofstream::out);

	ofs << data.size() << std::endl;

	for(auto it = data.begin(); it != data.end(); it++)
	{
		ofs << it->first << std::endl;
		ofs << it->second.CityObjectId << std::endl;
		ofs << it->second.HitCount << std::endl;

		for(unsigned int j = 0; j < it->second.HitCount; j++)
		{
			ofs << std::fixed << it->second.Viewpoints[j].x << std::endl;
			ofs << std::fixed << it->second.Viewpoints[j].y << std::endl;
			ofs << std::fixed << it->second.Viewpoints[j].z << std::endl;
		}
	}

	ofs.close();
}

void BelvedereDB::ResetDB(std::string dirTile, std::string label)
{
	if(label != "")
	{
		std::cout << "Resetting db category : " << label << std::endl;
		QFileInfo info(dirTile.c_str());
		std::string gbFilePathTemp = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_globalBVDB.dat";//Get global data path

		QFileInfo globalFileInfo(gbFilePathTemp.c_str());

		if(globalFileInfo.exists())
		{
			QFile gbFile(globalFileInfo.absoluteFilePath());
			if(!gbFile.remove())//Delete global data file
				std::cout << "Error while delete file : " << globalFileInfo.absoluteFilePath().toStdString() << " | " << gbFile.errorString().toStdString() << std::endl;
		}

		QDir dirfile = info.absoluteDir();
		for(QFileInfo f : dirfile.entryInfoList())//For each subdir in the db directory
		{
			if(f.isDir())
			{
				for(QFileInfo fbis : f.absoluteDir().entryInfoList())//Go through all file of that sub dir
				{
					if(fbis.isFile() && fbis.baseName().startsWith(label.c_str()+QString("_")))//Check if the file has the same label of our db
					{
						QFile tileFile(fbis.absoluteFilePath());
						tileFile.setPermissions(QFile::ReadOther | QFile::WriteOther);
						if(!tileFile.remove())//Delete it
							std::cout << "Error while delete file : " << tileFile.fileName().toStdString() << " | " << tileFile.errorString().toStdString() << std::endl;
					}
				}
			}
		}
		
		std::cout << "Done." << std::endl;
	}
}

///Used to sort two polygon data, a polygon is considered inferior if it has been seen less time
bool sortTopVectorFunc(PolygonData a, PolygonData b) { return b.HitCount < a.HitCount; }

std::map<std::string,std::vector<PolygonData>> BelvedereDB::GetTop(unsigned int t)
{
	if(label != "")
	{
		std::cout << "Extracting Top Polygon..." << std::endl;
		std::vector<std::string> fileToTreat;

		QFileInfo info(dirTile.c_str());

		//We are going to get all file of our database
		QDir dirfile = info.absoluteDir();
		for(QFileInfo f : dirfile.entryInfoList())
		{
			if(f.isDir())
			{
				for(QFileInfo fbis : QDir(f.absoluteFilePath()).entryInfoList())
				{
					if(fbis.isFile() && fbis.completeSuffix() == "gml")
					{
						fileToTreat.push_back(fbis.absoluteFilePath().toStdString());
					}
				}
			}
		}
		
		std::vector<PolygonData> top;

		for(std::string s : fileToTreat)//Go through all file
		{
			std::map<std::string,PolygonData> data = GetTileData(s);//Get polygon data about that file
			if(data.size() > 0)//Check if we have data
			{
				for(auto it = data.begin(); it != data.end(); it++)//Push all data in our result vector
				{
					top.push_back(it->second);
				}

				std::sort(top.begin(),top.end(),sortTopVectorFunc);//Sort the vector to have all top seen polygon at the beginning

				if(top.size() > t)//If we have top must result, we resize it
				{
					top.resize(t);
				}
			}
		}

		std::cout << "Done."  << std::endl;

		std::map<std::string,std::vector<PolygonData>> result;

		for(PolygonData p : top)
		{
			result[p.Tile].push_back(p);
		}

		return result;
	}
	else
		return std::map<std::string,std::vector<PolygonData>>();
}
