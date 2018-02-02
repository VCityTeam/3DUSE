// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __LINKCITYGMLSHAPE_HPP__
#define __LINKCITYGMLSHAPE_HPP__

#include <vector>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>

#include "libcitygml/utils/tile.hpp"
#include "export/exportCityGML.hpp"
#include "filters_export.h"

FILTERS_EXPORT citygml::CityModel* SplitBuildingsFromCityGML(
  vcity::Tile* Tile,
  std::vector<TextureCityGML*>* TexturesList
);

FILTERS_EXPORT citygml::CityModel* CutCityGMLwithShapefile(
  vcity::Tile* Tile, GDALDataset* ShapeFile,
  std::vector<TextureCityGML*>* TexturesList
);

FILTERS_EXPORT citygml::CityModel* CutMNTwithShapefile(
  vcity::Tile* Tile,
  GDALDataset* ShapeFile,
  std::vector<TextureCityGML*>* TexturesList
);

#endif // __LINKCITYGMLSHAPE_HPP__
