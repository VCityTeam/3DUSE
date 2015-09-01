// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __TILINGCITYGML_HPP__
#define __TILINGCITYGML_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "src/core/URI.hpp"
#include "libcitygml/citygml.hpp"
#include "src/gui/osg/osgGDAL.hpp"
#include "gui/moc/mainWindow.hpp"
#include "export/exportCityGML.hpp"
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
citygml::CityModel* TileCityGML(vcity::Tile* Tile, std::vector<TextureCityGML*>* TexturesList, TVec2d MinTile, TVec2d MaxTile, std::string PathFolder);
void MergingTile(vcity::Tile* OldTile, citygml::CityModel* NewTile, std::vector<TextureCityGML*>* TexturesList);
////////////////////////////////////////////////////////////////////////////////
#endif // __LINKCITYGMLSHAPE_HPP__
