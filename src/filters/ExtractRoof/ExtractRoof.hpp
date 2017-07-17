// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
