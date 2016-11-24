// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __EXTRACTROOF_HPP__
#define __EXTRACTROOF_HPP__

#include <vector>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>
#include "libcitygml/utils/tile.hpp"
#include "filters_export.h"

FILTERS_EXPORT std::pair<OGRGeometry*, OGRGeometry*> sortRoofs( citygml::CityModel* city );

#endif // __EXTRACTROOF_HPP__
