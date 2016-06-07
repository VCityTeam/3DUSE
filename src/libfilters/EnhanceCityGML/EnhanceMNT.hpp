// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __ENHANCEMNT_HPP__
#define __ENHANCEMNT_HPP__

#include <vector>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>

#include "core/tile.hpp"
#include "export/exportCityGML.hpp"

void CreateRoadsOnMNT( vcity::Tile* MNT,
                       OGRDataSource* Roads,
                       citygml::CityModel* MNT_roads,
                       std::vector<TextureCityGML*>* TexturesList_roads,
                       citygml::CityModel* MNT_ground,
                       std::vector<TextureCityGML*>* TexturesList_ground );

void CreateVegetationOnMNT( vcity::Tile* MNT,
                            OGRDataSource* Vegetation,
                            citygml::CityModel* MNT_vegetation,
                            std::vector<TextureCityGML*>* TexturesList_vegetation,
                            citygml::CityModel* MNT_ground,
                            std::vector<TextureCityGML*>* TexturesList_ground );

#endif // __ENHANCEMNT_HPP__
