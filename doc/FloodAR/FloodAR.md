# FloodAR plugin documentation
## Overview
This plugin was realized for the project FloodAR of Université Jean Monnet Saint-Etienne for the DREAL. It adds to 3D-USE several new functions of conversion from several file types to CityGML.  
The basic conversion algorithms code has been stored in 3D-USE core, while code that more pertains to I/O, user interaction... is in the plugin.

### Added files

Core side

* New class: [ImporterASC](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/import/importerASC.hpp)
 * This class houses several functions for converting [ASCII Esri grids](https://en.wikipedia.org/wiki/Esri_grid) to CityGML
 *  more details [below](#ImporterASC)
* [ASCCut](https://github.com/MEPP-team/VCity/blob/master/src/libfilters/tiling/ASCCut.hpp)
 * Contains a method to extract from a larger ASC grid the data for a given tile 

Plugin side

* [dialogFloodAR](https://github.com/MEPP-team/VCity/blob/master/src/plugins/CityGMLFloodARQtPlugin/Dialogs/dialogFloodAR.hpp)
 * The plugin UI
* [FloodARtools](https://github.com/MEPP-team/VCity/blob/master/src/plugins/CityGMLFloodARQtPlugin/FloodARTools.hpp)
 * Methods called by UI that gather input files and call the treatement methods.
 *  more details [below](#FloodARTools)

### Notable modifications to existing code

* [MNT class](https://github.com/MEPP-team/VCity/blob/master/src/DataStructures/DEM/osgMnt.hpp)
 * Moved to `src/DataStructures/DEM`
 * Changed several attributes from `int` to `float`
 * Added several setters/getters
 * Added a `write()` method
* [dataprofile](https://github.com/MEPP-team/VCity/blob/master/src/core/dataprofile.hpp)
 * Added a new dataprofile for Sablons
* [ExporterCityGML](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/export/exportCityGML.hpp)
 * modified export for CityGML `TINRelief` and `WaterBody` city objects to be more compliant with the CityGML standard
* [importerAssimp](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/import/importerAssimp.hpp)
 * added a parameter to enable or disable roof detection when converting OBJ files to CityGML (this functionality is not yet used as of 2016-05-09)

## ImporterASC

This class houses several public methods for converting [ASCII Esri grids](https://en.wikipedia.org/wiki/Esri_grid) to CityGML.
Most of these methods are given one or several `MNT*`parameters and output a `citygml::CityObject*`

#####*Note about the handling of ASC data :*
In the ASCII Esri Grid definition, one data point represents the elevation of a square surface unit. This property is used in the `waterToCityGMLPolygons()` method. In the other methods a data point is considered the elevation value for one geographic point that is the southwestern point of the square unit.

### Public methods
#### `citygml::CityObject* reliefToCityGML(MNT*)`
This method converts the data given in a `MNT` object to a CityObject of type `TINRelief`. Calls `generateTriangles()` to create the geometries.

#### `citygml::CityObject* fusionResolutions(MNT* asc1, MNT* asc2)`
This method converts the data given in two `MNT` objects to a CityObject of type `TINRelief`.  
Use for a tile where data exists at two different resolutions, but the more precise data isn't available everywhere.  
For each MNT, it creates list a of OGRPolygons in the same way as the `reliefToCityGML()` method. It then replaces the least precise polygons by the more precise ones when available, and realizes the seam at the border. OGRPolygons are then converted to CityGML.

#### `citygml::CityObject* waterToCityGML(MNT*)`
*This method may be deprecated by the next one.*  
This method converts the data given in a `MNT` object to a CityObject of type `WaterBody`. Calls `generateTriangles()` to create the geometries.

#### `citygml::CityObject* waterToCityGMLPolygons(MNT*, float precision)`
This method converts the data given in a `MNT` object to a CityObject of type `WaterBody`. It is used for water surfaces where a lot of neighboring points have the same elevation, which creates lots of redundancy in the data.  
It creates horizontal, square OGRPolygon for each data point, and if two neighboring points have the same elevation (more or less the precision given in param), it merges the corresponding polygons. OGRPolygons are then converted to CityGML.

### Private methods
#### `citygml::Geometry* generateTriangles(MNT*)`
Creates a triangle regular network from a MNT object.  
Each elevation value of the MNT object is considered as the data for a point (and not for a square surface). Each cell of 4 points is used to create 2 triangles. Points having the "No Data" value defined in the MNT object are ignored.

#### `void propagateCategory(MNT*, queue<pair<int, int>>*, float alt, float zPrec)`
Called in `waterToCityGMLPolygons()`. Checks which points neighboring the one in front of the queue have the same elevation, and push them at the back of the queue.

#### `OGRPolygon* createPoly(MNT*, int x, int y, float prec)`
Called in `waterToCityGMLPolygons()`. Creates a horizontal square OGRPolygon corresponding to the data point designed by x,y.

#### `citygml::Polygon* OGRPolyToGMLPoly(OGRPolygon*)`
Converts an `OGRPolygon` into a `citygml::Polygon`  
*Note: This function may be a duplicate of `ConvertOGRPolytoGMLPoly()` in ToolAlgoCut (see Issue #104 ), but trying to use this last one resulted in crashes during `delete` operations.*

## FloodARTools

Houses the methods called by the UI that fetch the input files and apply the various treatements to the data

#### `vector<TextureCityGML*> getTexturesList(citygml::CityModel*, QFileInfo, QFileInfo)`
#### `void cutASC(std::string filePath, std::string workingDir, int tileSizeX, int tileSizeY, bool isTerrain)`
To tile an ASC file  
Fetch an ASC file given in parameter, charge it into a MNT object, loops on x & y, calls `BuildTile()` (from ASCcut) and outputs the tiled ASC file in a "tmp" directory under the working directory. 
#### `void cutPicture(std::string filename, std::string workingDir, int width, int height)`
To tile an orthophotograph.  
Fetch an picture file given in parameter, charge it into a QImageReader object, loops over x & y, crops the picture to the bounds of the tile and outputs the tiled picture file in a "tmp" directory under the working directory.
#### `void CutShapeFile(std::string workingDir, int tilesize_x, int tilesize_y, std::string filename)`
To tile a shapefile.  
Fetch the shapefile given in parameter, read it with OSG, loops on x & y, loops through the shape features and adds the ones that are in the current tile in a collection, and outputs a new shapefile for the tile.
#### `void ASCtoWaterAuto(std::string workingDir, float prec, std::string startingDate)`
Conversion from ASC to CityGML Water.  
The input files are tiled ASC files from the `tmp/_WATER` directory, which means cutASC must have been executed before. This methods loops through the tile directories, charge ASC into MNT objects, calls `ImporterASC::waterToCityGMLPolygons()` to convert them to CityGML, adds them in a new CityModel. This method also checks for a "_TXXX_" token in the filename to add temporal data. Finally exports the CityGML Model to the _WATER directory in the working directory.
#### `void ASCtoTerrain(std::string workingDir)`
Conversion from ASC to CityGML Terrain.  
The input files are tiled ASC files from the `tmp/_MNT` directory, which means cutASC must have been executed before. This methods loops through the tile directories, charge ASC into MNT objects, calls `ImporterASC::fusionResolutions()` if there is two files in the tile (or calls `reliefToCityGML()` if there is only one) to convert them to CityGML, adds them in a new CityModel. Finally exports the CityGML Model to the _MNT directory in the working directory.
#### `void ShapeExtrusion(std::string workingDir)`
Conversion from shapefile to CityGML Building.  
The input files are tiled shapefiles from the `tmp/_BATI` directory, which means CutShapeFile must have been executed before. This methods loops through the tile directories, charge shapefiles into OSG objects, calls [ShpExtruction](https://github.com/MEPP-team/VCity/blob/master/src/processes/ShpExtrusion.hpp) to convert them to CityGML CityModels and exports the CityGML Models to the _BATI directory in the working directory. 
