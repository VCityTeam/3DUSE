// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef CITYGMLFUSION_H
#define CITYGMLFUSION_H

#include "citygml.hpp"

void FusionLODs(citygml::CityModel * City1, citygml::CityModel * City2);
void CallFusion();

#endif // CITYGMLFUSION_H
