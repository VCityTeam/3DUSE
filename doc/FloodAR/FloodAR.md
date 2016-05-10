# FloodAR plugin documentation
## Overview
This plugin was realized for the project FloodAR of Université Jean Monnet Saint-Etienne for the DREAL. It adds to 3D-USE several new functions of conversion from several file types to CityGML.  
The basic conversion algorithms code has been stored in 3D-USE core, while code that more pertains to I/O, user interaction... is in the plugin.

The objective for this work is to help prepare a 3D model of Sablons' surrounding area, with the hydroelectric plant and dam, and a dynmaic visualisation of a flood using the temporal display.
The input data is:

* The terrain's digital elevation model. Two DEMs are actually given, one being a lot more precise on the important areas. [ASCII Esri grid](https://en.wikipedia.org/wiki/Esri_grid) (*.asc) files
* An [orthophoto](https://en.wikipedia.org/wiki/Orthophoto) to apply to the terrain model as texture. Most picture formats are compatible, but must be accompanied by a [world file](https://en.wikipedia.org/wiki/World_file).
* Several water elevation models, each one corresponding to the water height at a given time. ASCII Esri grid (*.asc) files
* The buildings outlines from IGN's BDTOPO. [Shapefiles](https://en.wikipedia.org/wiki/Shapefile)
* 3D models of remarkable buildings. 

The output is an ensemble of CityGML files resulting from the conversion of the above data. The CityGML are stored according to the (new as of now) file hierarchy used for 3D-USE. For a given tile, all water heights are gathered inside one single CityGML file.

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
 * Methods called by UI that gather input files and call the treatement methods.

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