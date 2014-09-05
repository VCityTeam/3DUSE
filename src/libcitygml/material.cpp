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
#include "material.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
Material::Material( const std::string& id )
    : Appearance( id, "Material" ), _ambientIntensity( 0.f ), _shininess( 0.f ), _transparency( 0.f )
{}
////////////////////////////////////////////////////////////////////////////////
TVec3f Material::getDiffuse( void ) const
{
    return _diffuse;
}
////////////////////////////////////////////////////////////////////////////////
TVec3f Material::getEmissive( void ) const
{
    return _emissive;
}
////////////////////////////////////////////////////////////////////////////////
TVec3f Material::getSpecular( void ) const
{
    return _specular;
}
////////////////////////////////////////////////////////////////////////////////
float Material::getAmbientIntensity( void ) const
{
    return _ambientIntensity;
}
////////////////////////////////////////////////////////////////////////////////
float Material::getShininess( void ) const
{
    return _shininess;
}
////////////////////////////////////////////////////////////////////////////////
float Material::getTransparency( void ) const
{
    return _transparency;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
