# FloodAR plugin documentation
## Overview
This plugin was realized for the project FloodAR of Université Jean Monnet Saint-Etienne for the DREAL. It adds to 3D-USE several new functions of conversion from several file types to CityGML.  
The basic conversion algorithms code has been stored in 3D-USE core, while code that more pertains to I/O, user interaction... is in the plugin.

The objective for this work is to help prepare a 3D model of Sablons' surrounding area, with the hydroelectric plant and dam, and a dynamic visualization of a flood using the temporal display.
The input data is:

* The terrain's digital elevation model. Two DEMs are actually given, one being a lot more precise on the important areas. [ASCII Esri grid](https://en.wikipedia.org/wiki/Esri_grid) (*.asc) files
* An [orthophoto](https://en.wikipedia.org/wiki/Orthophoto) to apply to the terrain model as texture. Most picture formats are compatible, but must be accompanied by a [world file](https://en.wikipedia.org/wiki/World_file).
* Several water elevation models, each one corresponding to the water height at a given time. ASCII Esri grid (*.asc) files
* The buildings outlines from IGN's BDTOPO. [Shapefiles](https://en.wikipedia.org/wiki/Shapefile)
* 3D models of remarkable buildings. 

The output is an ensemble of CityGML files resulting from the conversion of the above data. Due to the high quantity of data, it has been decided the the CityGML would be tiled into 500m*500m tiles. The CityGML are stored according to the (new as of now) file hierarchy used for 3D-USE. For a given tile, all water heights are gathered inside one single CityGML file.

## Design

It has been decided that this work would be designed as a 3D-USE plugin; but that basic operations on data structure (I/O, conversion) would be implemented in the core application so that they may be easily accessible for future projects. The plugin itself would be mostly the UI, the access to the files, and the calls to I/O and conversion methods. 

Three main data workflows that are mostly independents can be distinguished: 

* Tiling and conversion of Water and Terrain ASC files
 * Most of this work concerns this point.
* Tiling and conversion of Shapefiles
 * The conversion part was already done in plugin Visibilité (ShpExtrusion), but the tiling function was yet to be created. 
 * The conversion part **needs the conversion of the terrain** to be completed
* Conversion of buildings 3D models.
 * Most of this is already existing in 3D-USE (see [here](https://github.com/MEPP-team/VCity/blob/master/src/gui/mainWindow.cpp#L2057)), and due to fact that we were not given the 3D models until a late date, this workflow was largely untouched and is not detailed here.

### Tiling and conversion from ASC files to CityGML:

1. Open ASC files and store the data in memory. 
2. Tile data from ASC files and export the tiled data to a new ASC file.
3. Convert data from ASC files to CityGML data.
4. Export CityGML data to a CityGML file. 

Opening of ASC files was already realized by the [MNT class](https://github.com/MEPP-team/VCity/blob/master/src/DataStructures/DEM/osgMnt.hpp).
Export of CityGML data was already realized by [ExporterCityGML](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/export/exportCityGML.hpp) but the export of Terrain and Water was not respecting the CityGML standard.  

For point 2, it has been decided that a method would take a MNT object as a parameter as well as the "number" (defined by the coordinates of the southwestern point divided by the tile size) of the tile to extract and return a MNT object of the extracted data. The calling method would iterate through tile numbers. The MNT class has been modified so it now houses its own file writer.
  
Point 3 is the most consequent par of this work. Here are the main points:

* The input ASC data might be either terrain data or water data, so we need to distinguish this first. There's no way ton know this from the file, so we can only create to different conversion methods, and only the user can choose which one to execute.

* Terrain data actually come from two datasets. A more precise one on which only covers the most important area, and a less precise one on the surroundings. This again calls for two separate conversion methods.
 * On tiles where only one dataset exists (the surrounding area), there's no problem, we simply convert the elevation data from one file to CityGML surface data.
 * On tiles where the two datasets exist, we need to join the two to only output one CityGML surface.
* We want the terrain surface to have textures. 
 * As for the other data types, we create a method to tile an orthophoto (accompanied by a World file) to tiled textures of correct size.
 * During terrain conversion itself we assume that the original orthophoto has been tiled before, but we need to calculate and write the texture coordinates for every polygon of the terrain surface.

* Water data is highly redundant: some large areas have all their points at the same elevation (the water surface is mostly horizontal after all). To lower the memory usage, we want to create the CityGML surface conserving only the points on the edge of such areas.
* Water data is dynamic! One ASC file for water height actually represents the water height a certain number of hours after the beginning of a flood. We want to add this temporal information to the CityGML file. There is no way to store the information in the ASC file itself, but asking for user prompt each time has been rejected due to the high number of file. The temporal information will have to be present in the filename.
 * The pattern that will be searched in the filename is `#_T[0-9]*_#` in regex notation, the number being the number of hours after the beginning of the flood. 
 * As CityGML only allows "absolute" dates, the user will have to define their wanted starting date of the flood.

### Tiling and conversion from Shapefiles to CityGML:

1. Open Shapefiles and store data in memory
2. Tile data from Shapefiles and export the tiled data to a new Shapefile.
3. Convert data from Shapefiles to CityGML data.
4. Export CityGML data to a CityGML file. (same as ASC)

Opening Shapefiles and export to another Shapefile is already realized using GDAL library.  
The conversion from Shapefile to CityGML (buildings) and the export to a CityGML file was already realized in one go by ShpExtrusion in plugin Visibilité. The method as been moved to the 3D-USE libraries so it can be accessible from another plugin, but has otherwise not been modified.

For tiling Shapefiles, a new method has been created that takes a filepath to a Shapefile in parameter and creates all the tiled Shapefiles.

## Added files

Core side

* New class: [ImporterASC](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/import/importerASC.hpp)
 * This class houses several functions for converting ASCII grids to CityGML
* [ASCCut](https://github.com/MEPP-team/VCity/blob/master/src/libfilters/tiling/ASCCut.hpp)
 * Contains a method to extract from a larger ASCII grid the data for a given tile 

Plugin side

* [dialogFloodAR](https://github.com/MEPP-team/VCity/blob/master/src/plugins/CityGMLFloodARQtPlugin/Dialogs/dialogFloodAR.hpp)
 * The plugin UI
* [FloodARtools](https://github.com/MEPP-team/VCity/blob/master/src/plugins/CityGMLFloodARQtPlugin/FloodARTools.hpp)
 * New namespace `FloodAR`
 * Methods called by UI that gather input files and call the treatment methods.

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
