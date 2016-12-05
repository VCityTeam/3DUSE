#ifndef __TILEDFILESLAYOUT_HPP__
#define __TILEDFILESLAYOUT_HPP__

#include <string>
#include <vector>

#include "vecs.hpp"
#include "citygmlutils_export.h"
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
#pragma warning(disable: 4251) // export problem on STL members
#endif

struct TiledLayer
{
    std::string Name;
    int TuileMinX;
    int TuileMinY;
    int TuileMaxX;
    int TuileMaxY;
};

class CITYGMLUTILS_EXPORT TiledFiles
{
public:
  TiledFiles(std::string Folderpath);
  void BuildListofLayers();

  /// Main folder containing all tiled files
  std::string Folder;

  /// Contains all layers present in Folder
  std::vector<TiledLayer> ListofLayers;
};

#endif
