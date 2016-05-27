// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __LINKCITYGMLSHAPE_HPP__
#define __LINKCITYGMLSHAPE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include "ogrsf_frmts.h"
#include "core/tile.hpp"
#include "export/exportCityGML.hpp"
////////////////////////////////////////////////////////////////////////////////
citygml::CityModel* SplitBuildingsFromCityGML(vcity::Tile* Tile, std::vector<TextureCityGML*>* TexturesList);
citygml::CityModel* CutCityGMLwithShapefile(vcity::Tile* Tile, OGRDataSource* ShapeFile, std::vector<TextureCityGML*>* TexturesList);
citygml::CityModel* CutMNTwithShapefile(vcity::Tile* Tile, OGRDataSource* ShapeFile, std::vector<TextureCityGML*>* TexturesList);
////////////////////////////////////////////////////////////////////////////////
#endif // __LINKCITYGMLSHAPE_HPP__
