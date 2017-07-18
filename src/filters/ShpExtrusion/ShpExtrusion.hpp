// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef _SHPEXTRUSION_
#define _SHPEXTRUSION_

#include <vector>
#include "core/application.hpp"
#include "filters_export.h"

typedef std::vector<TVec3d> LRing;

/**
*	@brief Extrude a chosen Shp file
*/
FILTERS_EXPORT citygml::CityModel* ShpExtrusion(
  OGRDataSource* poDS,
  std::string dir
);

/**
*	@brief Put a set of point a the height of the terrain
*/
LRing PutLRingOnTerrain(LRing ring, std::string dir);

#endif
