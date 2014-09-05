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
#ifndef __CITYGML_GEOREFERENCEDTEXTURE_HPP__
#define __CITYGML_GEOREFERENCEDTEXTURE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "texture.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class GeoreferencedTexture : public Texture
{
    friend class CityGMLHandler;

public:
    GeoreferencedTexture( const std::string& id );

    bool getPreferWorldFile( void ) const;

    // TODO support referencePoint and orientation
    class WorldParams
    {
    public:
        WorldParams()
            : xPixelSize(0.0), yRotation(0.0), xRotation(0.0), yPixelSize(0.0), xOrigin(0.0), yOrigin(0.0)
        {

        }

        double xPixelSize;
        double yRotation;
        double xRotation;
        double yPixelSize;
        double xOrigin;
        double yOrigin;
    };

    bool m_initWParams;
    WorldParams m_wParams;

protected:
    bool _preferWorldFile;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& os, const GeoreferencedTexture::WorldParams& wp);
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_GEOREFERENCEDTEXTURE_HPP__
