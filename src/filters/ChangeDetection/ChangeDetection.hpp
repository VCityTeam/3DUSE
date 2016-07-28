// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __CHANGEDETECTION_HPP__
#define __CHANGEDETECTION_HPP__

#include <stdlib.h>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>
#include "libcitygml/citygml.hpp"
#include "filters_export.h"

struct ChangeDetectionRes { //Struct to return result of change detection process
    OGRMultiPolygon* EnveloppeCityU1; //Polygons representing footprints of buildings from first CityModel
    OGRMultiPolygon* EnveloppeCityU2; //Polygons representing footprints of buildings from second CityModel

    OGRMultiPolygon* BatiDetruits; //Footprints of destroyed buildings
    OGRMultiPolygon* BatiCrees; //Footprints of constructed buildings
    OGRMultiPolygon* BatiModifies1; //Footprints of modified buildings of the first CityModel
    OGRMultiPolygon* BatiModifies2; //Footprints of modified buildings of the second CityModel
    OGRMultiPolygon* BatiInchanges; //Footprints of unchanged buildings
};

FILTERS_EXPORT ChangeDetectionRes CompareTiles(
  std::string Folder,
  citygml::CityModel *City1,
  citygml::CityModel *City2
);

#endif // __CHANGEDETECTION_HPP__
