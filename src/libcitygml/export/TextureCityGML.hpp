// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __TEXTURECITYGML_HPP_
#define __TEXTURECITYGML_HPP_
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "vecs.hpp"
#include "citymodel.hpp"
////////////////////////////////////////////////////////////////////////////////
struct TexturePolygonCityGML {
   std::vector<TVec2f> TexUV;
   std::string Id;
   std::string IdRing;
};
struct TextureCityGML {
   std::string Url;
   citygml::Texture::WrapMode Wrap;
   std::vector<TexturePolygonCityGML> ListPolygons;
};
#endif // __TEXTURECITYGML_HPP_
