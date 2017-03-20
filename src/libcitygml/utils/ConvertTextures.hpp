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
