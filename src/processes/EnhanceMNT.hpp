// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __ENHANCEMNT_HPP__
#define __ENHANCEMNT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "src/core/URI.hpp"
#include "libcitygml/citygml.hpp"
#include "src/gui/osg/osgGDAL.hpp"
#include "gui/moc/mainWindow.hpp"
#include "export/exportCityGML.hpp"
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
void CreateRoadsOnMNT(vcity::Tile* MNT, OGRDataSource* Roads, citygml::CityModel* MNT_roads, std::vector<TextureCityGML*>* TexturesList_roads, citygml::CityModel* MNT_ground, std::vector<TextureCityGML*>* TexturesList_ground);
void CreateVegetationOnMNT(vcity::Tile* MNT, OGRDataSource* Vegetation, citygml::CityModel* MNT_vegetation, std::vector<TextureCityGML*>* TexturesList_vegetation, citygml::CityModel* MNT_ground, std::vector<TextureCityGML*>* TexturesList_ground);
////////////////////////////////////////////////////////////////////////////////
#endif // __ENHANCEMNT_HPP__
