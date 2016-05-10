#ifndef _FLOODAR_TOOLS_
#define _FLOODAR_TOOLS_

#include "export/exportCityGML.hpp"
#include <QFileDialog>

/**
* Houses the methods called by the dialogFloodAR UI that fetch the input files and apply the various treatments to the data
*/
namespace FloodAR
{
  /**
   * Builds a texture List for the given CityModel with the given texture file, assuming that their bounding boxes are the same.
   */
  std::vector<TextureCityGML*> getTexturesList(citygml::CityModel* model, QDir tiledir, QFileInfo texturesPath);

  /**
   * To tile an ASC file  
   * Fetch an ASC file given in parameter, charge it into a MNT object, loops on x & y, calls `BuildTile()` (from ASCcut) and outputs the tiled ASC file in a "tmp" directory under the working directory. 
   */
  void cutASC(std::string filePath, std::string workingDir, int tileSizeX, int tileSizeY, bool isTerrain);

  /**
   * To tile an orthophoto.  
   * Fetch an picture file given in parameter, charge it into a QImageReader object, loops over x & y, crops the picture to the bounds of the tile and outputs the tiled picture file in a "tmp" directory under the working directory.
   */
  void cutPicture(std::string filename, std::string workingDir, int width, int height);

  /**
  * To tile a shapefile.  
  * Fetch the shapefile given in parameter, read it with OSG, loops on x & y, loops through the shape features and adds the ones that are in the current tile in a collection, and outputs a new shapefile for the tile.
  */
  void CutShapeFile(std::string workingDir, int tilesize_x, int tilesize_y, std::string filename);

  /**
   * Conversion from ASC to CityGML Water.  
   * The input files are tiled ASC files from the `tmp/_WATER` directory, which means cutASC must have been executed before. This methods loops through the tile directories, charge ASC into MNT objects, calls `ImporterASC::waterToCityGMLPolygons()` to convert them to CityGML, adds them in a new CityModel. This method also checks for a "_TXXX_" token in the filename to add temporal data. Finally exports the CityGML Model to the _WATER directory in the working directory.
   */
  void ASCtoWaterAuto(std::string workingDir, float prec, std::string startingDate);

  /**
   * Conversion from ASC to CityGML Terrain.  
   * The input files are tiled ASC files from the `tmp/_MNT` directory, which means cutASC must have been executed before. This methods loops through the tile directories, charge ASC into MNT objects, calls `ImporterASC::fusionResolutions()` if there is two files in the tile (or calls `reliefToCityGML()` if there is only one) to convert them to CityGML, adds them in a new CityModel. Finally exports the CityGML Model to the _MNT directory in the working directory.
   */
  void ASCtoTerrain(std::string workingDir);

  /**
   * Conversion from shapefile to CityGML Building.  
   * The input files are tiled shapefiles from the `tmp/_BATI` directory, which means CutShapeFile must have been executed before. This methods loops through the tile directories, charge shapefiles into OSG objects, calls [ShpExtrusion](https://github.com/MEPP-team/VCity/blob/master/src/processes/ShpExtrusion.hpp) to convert them to CityGML CityModels and exports the CityGML Models to the _BATI directory in the working directory.
   */
  void ShapeExtrusion(std::string workingDir);
}

#endif
