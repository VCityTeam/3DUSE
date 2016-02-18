// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __LINKCITYGMLSHAPE_HPP__
#define __LINKCITYGMLSHAPE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "src/core/URI.hpp"
#include "libcitygml/citygml.hpp"
#include "src/gui/osg/osgGDAL.hpp"
#include "gui/moc/mainWindow.hpp"
#include "export/exportCityGML.hpp"
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
citygml::CityModel* SplitBuildingsFromCityGML(vcity::Tile* Tile, std::vector<TextureCityGML*>* TexturesList);
citygml::CityModel* CutCityGMLwithShapefile(vcity::Tile* Tile, OGRDataSource* ShapeFile, std::vector<TextureCityGML*>* TexturesList);
citygml::CityModel* CutMNTwithShapefile(vcity::Tile* Tile, OGRDataSource* ShapeFile, std::vector<TextureCityGML*>* TexturesList);
////////////////////////////////////////////////////////////////////////////////
#endif // __LINKCITYGMLSHAPE_HPP__
