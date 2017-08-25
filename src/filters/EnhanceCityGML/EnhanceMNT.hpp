// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __ENHANCEMNT_HPP__
#define __ENHANCEMNT_HPP__

#include <vector>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>

#include "libcitygml/utils/tile.hpp"
#include "export/exportCityGML.hpp"
#include "filters_export.h"

FILTERS_EXPORT void CreateRoadsOnMNT(
   vcity::Tile* MNT,
   GDALDataset* Roads,
   citygml::CityModel* MNT_roads,
   std::vector<TextureCityGML*>* TexturesList_roads,
   citygml::CityModel* MNT_ground,
   std::vector<TextureCityGML*>* TexturesList_ground
);

FILTERS_EXPORT void CreateVegetationOnMNT(
   vcity::Tile* MNT,
   GDALDataset* Vegetation,
   citygml::CityModel* MNT_vegetation,
   std::vector<TextureCityGML*>* TexturesList_vegetation,
   citygml::CityModel* MNT_ground,
   std::vector<TextureCityGML*>* TexturesList_ground
);

#endif // __ENHANCEMNT_HPP__
