// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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
