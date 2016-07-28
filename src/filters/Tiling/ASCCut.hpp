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
