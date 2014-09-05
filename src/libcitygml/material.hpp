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
#ifndef __CITYGML_MATERIAL_HPP__
#define __CITYGML_MATERIAL_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "appearance.hpp"
#include "vecs.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Material : virtual public Appearance
{
    friend class CityGMLHandler;
public:
    Material( const std::string& id );

    TVec3f getDiffuse( void ) const;
    TVec3f getEmissive( void ) const;
    TVec3f getSpecular( void ) const;
    float getAmbientIntensity( void ) const;
    float getShininess( void ) const;
    float getTransparency( void ) const;

protected:
    TVec3f _diffuse;
    TVec3f _emissive;
    TVec3f _specular;
    float _ambientIntensity;
    float _shininess;
    float _transparency;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_MATERIAL_HPP__
