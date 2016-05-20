#ifndef __TILEDFILESLAYOUT_HPP__
#define __TILEDFILESLAYOUT_HPP__

#include <string>
#include <vector>

#include "vecs.hpp"

struct TiledLayer
{
	std::string Name;
	int TuileMinX;
	int TuileMinY;
	int TuileMaxX;
	int TuileMaxY;
};

class TiledFiles
{
public :
	TiledFiles(std::string Folderpath);

	std::string Folder; //Main folder containing all tiled files

	std::vector<TiledLayer> ListofLayers; //Contains all layers present in Folder

	void BuildListofLayers();

private :
	
};

#endif
