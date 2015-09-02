#include "BelvedereDB.h"

#include <qfileinfo.h>
#include <qdir.h>

extern Q_CORE_EXPORT int qt_ntfs_permission_lookup;

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
		globalDataFilename = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_globalBVDB.dat";

		QFileInfo globalFileInfo(globalDataFilename.c_str());
		if(globalFileInfo.exists())
		{
			char line[256];

			std::ifstream ifs (globalDataFilename, std::ifstream::in);

			ifs.getline(line,256);

			unsigned int count = atoi(line);
			global.ViewpointCaptureCount = count;

			for(unsigned int j = 0; j < count; j++)
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

		for(TVec3d vec : global.Viewpoints)
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
			std::ofstream ofs (globalDataFilename, std::ofstream::out);

			ofs << global.ViewpointCaptureCount << std::endl;

			for(TVec3d vec : global.Viewpoints)
				ofs << vec.x << std::endl << vec.y << std::endl << vec.z << std::endl;

			ofs.close();

			std::map<std::string,std::map<std::string,PolygonData>> tilePolyData;

			for(unsigned int i = 0; i < viewpoint->width; i++)
			{
				for(unsigned int j = 0; j < viewpoint->height; j++)
				{
					if(viewpoint->hits[i][j].intersect)
					{
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.objectId].HitCount++;
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.objectId].CityObjectId = viewpoint->hits[i][j].triangle.objectId;
						tilePolyData[viewpoint->hits[i][j].triangle.tileFile][viewpoint->hits[i][j].triangle.objectId].Viewpoints.push_back(viewpoint->position);
					}
				}
			}

			for(auto it = tilePolyData.begin(); it != tilePolyData.end(); it++)
			{
				QFileInfo info(it->first.c_str());
				std::string tileDBFileName = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_"+info.baseName().toStdString()+".dat";

				std::map<std::string,PolygonData> existingDB = GetTileData(it->first);
				std::map<std::string,PolygonData> currentDB = it->second;

				for(auto ib = currentDB.begin(); ib != currentDB.end(); ib++)
				{
					existingDB[ib->first].HitCount++;
					existingDB[ib->first].CityObjectId = ib->second.CityObjectId;
					existingDB[ib->first].Viewpoints.push_back(ib->second.Viewpoints.front());
				}

				ExportTileData(it->first,existingDB);
			}
		}
	}
}

std::map<std::string,PolygonData> BelvedereDB::GetTileData(std::string tilePath)
{
	QFileInfo info(tilePath.c_str());
	std::string tileDBFileName = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_"+info.baseName().toStdString()+".dat";

	if(QFileInfo(tileDBFileName.c_str()).exists())
	{
		std::ifstream ifs (tileDBFileName, std::ifstream::in);

		char line[256];
		ifs.getline(line,256);

		unsigned int count = atoi(line);

		std::map<std::string,PolygonData> result;

		for(unsigned int i = 0; i < count; i++)
		{
			ifs.getline(line,256);
			std::string name = std::string(line);
			ifs.getline(line,256);
			std::string cityObjectId = std::string(line);
			ifs.getline(line,256);
			unsigned int cpt = atoi(line);
			PolygonData data;
			data.HitCount = cpt;
			data.CityObjectId = cityObjectId;

			for(unsigned int j = 0; j < cpt; j++)
			{
				double x;
				double y;
				double z;

				ifs.getline(line,256);x = atof(line);
				ifs.getline(line,256);y = atof(line);
				ifs.getline(line,256);z = atof(line);

				data.Viewpoints.push_back(TVec3d(x,y,z));
			}

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
			ofs << it->second.Viewpoints[j].x << std::endl;
			ofs << it->second.Viewpoints[j].y << std::endl;
			ofs << it->second.Viewpoints[j].z << std::endl;
		}
	}

	ofs.close();
}

void BelvedereDB::ResetDB(std::string dirTile, std::string label)
{
	if(label != "")
	{
		std::cout << "Resetting db category : " << label << std::endl;
		qt_ntfs_permission_lookup++; // turn checking on
		QFileInfo info(dirTile.c_str());
		std::string gbFilePathTemp = info.absoluteDir().absolutePath().toStdString()+"/"+label+"_globalBVDB.dat";

		QFileInfo globalFileInfo(gbFilePathTemp.c_str());

		if(globalFileInfo.exists())
		{
			QFile gbFile(globalFileInfo.absolutePath());
			if(!gbFile.setPermissions(QFile::ReadOther | QFile::WriteOther))
				std::cout << "Error while setting permissions : " << gbFile.fileName().toStdString() << " | " << gbFile.errorString().toStdString() << std::endl;
			if(!gbFile.remove())
				std::cout << "Error while delete file : " << gbFile.fileName().toStdString() << " | " << gbFile.errorString().toStdString() << std::endl;
		}

		QDir dirfile = info.absoluteDir();
		for(QFileInfo f : dirfile.entryInfoList())
		{
			if(f.isDir())
			{
				for(QFileInfo fbis : f.absoluteDir().entryInfoList())
				{
					if(fbis.isFile() && fbis.baseName().startsWith(label.c_str()+QString("_")))
					{
						QFile tileFile(fbis.absolutePath());
						tileFile.setPermissions(QFile::ReadOther | QFile::WriteOther);
						if(!tileFile.remove())
							std::cout << "Error while delete file : " << tileFile.fileName().toStdString() << " | " << tileFile.errorString().toStdString() << std::endl;
					}
				}
			}
		}
		
		qt_ntfs_permission_lookup--; // turn it off again
		std::cout << "Done." << std::endl;
	}
}