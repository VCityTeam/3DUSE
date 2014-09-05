/* -*-c++-*- libcitygml - Copyright (c) 2010 Joachim Pouderoux, BRGM
*
* This file is part of libcitygml library
* http://code.google.com/p/libcitygml
*
* libcitygml is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 2.1 of the License, or
* (at your option) any later version.
*
* libcitygml is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*/
////////////////////////////////////////////////////////////////////////////////
#include "georeferencedtexture.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
GeoreferencedTexture::GeoreferencedTexture( const std::string& id )
    : Appearance( id, "GeoreferencedTexture" ), Texture( id ), m_initWParams(false), _preferWorldFile(true)
{}
////////////////////////////////////////////////////////////////////////////////
inline bool GeoreferencedTexture::getPreferWorldFile( void ) const
{
    return _preferWorldFile;
}
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& os, const GeoreferencedTexture::WorldParams& wp)
{
    os << "xPixelSize : " << wp.xPixelSize << std::endl;
    os << "yRotation  : " << wp.yRotation << std::endl;
    os << "xRotation  : " << wp.xRotation << std::endl;
    os << "yPixelSize : " << wp.yPixelSize << std::endl;
    os << "xOrigin    : " << wp.xOrigin << std::endl;
    os << "yOrigin    : " << wp.yOrigin << std::endl;
    return os;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
