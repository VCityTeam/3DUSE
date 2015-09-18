#ifndef __BELVEDEREDB_HPP__
#define __BELVEDEREDB_HPP__

#include <string>
#include "../../../core/application.hpp"
#include "ViewPoint.h"
#include <map>
#include <iostream>

/**
*	@Brief Global data about a batch of viewpoint analysis
*/
struct GlobalData
{
	GlobalData() { ViewpointCaptureCount = 0; }
	unsigned int ViewpointCaptureCount; ///< How many analysis we did in the batch
	std::vector<TVec3d> Viewpoints; ///< List of viewpoints used for analysis
};

/**
*	@Brief Data about a CityGML polygon	
*/
struct PolygonData
{
	PolygonData() { HitCount = 0; }
	unsigned int HitCount;///< How many time the polygon has been seen in an analysis batch
	std::string CityObjectId;///< Id of the cityobject of this polygon
	std::string PolygonId;
	std::string Tile;
	std::vector<TVec3d> Viewpoints;///< List of viewpoints position that have seen this polygon
};

/**
*	@Brief Database for belvedere searching from a batch of viewpoint analysis. Is a singleton.
*/
class BelvedereDB
{
public:
	/**
	*	@Brief Get the unique instance of the Belvedere DB
	*/
	static inline BelvedereDB& Get() { return instance; }

	/**
	*	@brief Setup the database with a location and a label
	*	@param dirTile Directory where the database is located, must be where tile used for capture are
	*	@param label Label of our database
	*	@param deltaDistance Distance at which a viewpoint must at least be from the other to be taken in account
	*/
	void Setup(std::string dirTile, std::string label, double deltaDistance = 0.0);
	/**
	*	@brief Reset the database for a label
	*	@param dirTile Directory where the database is located, must be where tile used for capture are
	*	@param label Label of our database
	*/
	void ResetDB(std::string dirTile, std::string label);
	/**
	*	@brief Feed the database with the data from a viewpoint capture
	*	@param viewpoint Viewpoint capture
	*/
	void ExportViewpointData(ViewPoint* viewpoint);

	/**
	*	@brief Get the top t polygon seen. DB must have been setup before calling
	*	@param t How many polygon we want
	*	@return Key = tile Name, Value = list of polygon of that tile. Sum(Value.size()) = t
	*/
	std::map<std::string,std::vector<PolygonData>> GetTop(unsigned int t = 10);
	/**
	*	@brief Get the db data of a citygml tile. DB must have been setup before calling
	*	@param tilePath Path to the citygml tile
	*	@return key = polygon id, value = polygon data
	*/
	std::map<std::string,PolygonData> GetTileData(std::string tilePath);

	/// Get the current directory of the DB
	inline std::string GetDirTile() { return dirTile; }
	/// Get the current label of the DB
	inline std::string GetLabel() { return label; }
	/// Get the global data of the db
	inline GlobalData GetGlobalData() { return global; }

private:
	/**
	*	@brief Export the data of a tile
	*	@param tilePath Path to the citygml tile
	*	@param data Data about that tile, key = polygon id, value = polygon data
	*/
	void ExportTileData(std::string tilePath, std::map<std::string,PolygonData> data);

	BelvedereDB();
	~BelvedereDB();
	static BelvedereDB instance;///< Unique instance of our db

	std::string dirTile;///< Directory where the database is located, must be where tile used for capture are
	std::string label;///< Label of our database
	double deltaDistance;///< Distance at which a viewpoint must at least be from the other to be taken in account
	std::string globalDataFilename;///< Path to the file containing global data

	GlobalData global;///< Global data of the db
};

#endif