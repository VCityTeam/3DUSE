// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef __CONVERTTEXTURES_HPP__
#define __CONVERTTEXTURES_HPP__

#include "libcitygml/citygml.hpp"
#include "libcitygml/utils/tile.hpp"
#include "citygmlutils_export.h"

/**
*	@brief Convert a vector of georeferenced texture coordinates to local textures coordinates (between 0 and 1)
*  @param texUV : Input vector of georeferenced texture coordinates
*  @return Vectr of local texture coordinates
*/
CITYGMLUTILS_EXPORT std::vector<TVec2f> ConvertGeoreferencedTextures( std::vector<TVec2f> texUV );


#endif
