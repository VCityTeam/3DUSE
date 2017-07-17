// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef _ASCCUT_
#define _ASCCUT_

#include "DataStructures/DEM/osgMnt.hpp"
#include "filters_export.h"

FILTERS_EXPORT MNT* BuildTile(
  MNT* in,
  int size_x,
  int size_y,
  int tile_x,
  int tile_y
);

#endif //_ASCCUT_
