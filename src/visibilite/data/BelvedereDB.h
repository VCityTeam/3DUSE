#ifndef __BELVEDEREDB_HPP__
#define __BELVEDEREDB_HPP__

#include <string>
#include "src/core/application.hpp"
#include "src/visibilite/data/ViewPoint.h"
#include <map>
#include <iostream>

struct GlobalData
{
	GlobalData() { ViewpointCaptureCount = 0; }
	unsigned int ViewpointCaptureCount;
	std::vector<TVec3d> Viewpoints;
};

struct PolygonData
{
	PolygonData() { HitCount = 0; }
	unsigned int HitCount;
	std::string CityObjectId;
	std::vector<TVec3d> Viewpoints;
};

class BelvedereDB
{
public:
	static inline BelvedereDB& Get() { return instance; }

	void Setup(std::string dirTile, std::string label, double deltaDistance = 0.0);
	void ResetDB(std::string dirTile, std::string label);
	void ExportViewpointData(ViewPoint* viewpoint);

	std::map<std::string,PolygonData> GetTileData(std::string tilePath);

	/*void LoadTile(std::string tile);
	void ExportTile();
	void Reset();*/

	inline std::string GetDirTile() { return dirTile; }
	inline std::string GetLabel() { return label; }
	inline GlobalData GetGlobalData() { return global; }

private:
	void ExportTileData(std::string tilePath, std::map<std::string,PolygonData> data);

	BelvedereDB();
	~BelvedereDB();
	static BelvedereDB instance;

	std::string dirTile;
	std::string label;
	double deltaDistance;
	std::string globalDataFilename;

	GlobalData global;
};

#endif