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
#include "object.hpp"
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
Object::Object(const std::string& id)
    : _id( id )
{
    if( _id == "" )
    {
        std::stringstream ss; ss << "PtrId_" << this; _id = ss.str();
    }
}
////////////////////////////////////////////////////////////////////////////////
Object::~Object()
{}
////////////////////////////////////////////////////////////////////////////////
const std::string& Object::getId() const
{
    return _id;
}
////////////////////////////////////////////////////////////////////////////////
std::string Object::getAttribute( const std::string& name ) const
{
    std::map< std::string, std::string >::const_iterator elt = _attributes.find( name );
    return elt != _attributes.end() ? elt->second : "";
}
////////////////////////////////////////////////////////////////////////////////
const AttributesMap& Object::getAttributes() const
{
    return _attributes;
}
////////////////////////////////////////////////////////////////////////////////
AttributesMap& Object::getAttributes()
{
    return _attributes;
}
////////////////////////////////////////////////////////////////////////////////
void Object::setAttribute( const std::string& name, const std::string& value, bool forceOnExist)
{
    if ( !forceOnExist )
    {
        std::map< std::string, std::string >::const_iterator elt = _attributes.find( name );
        if ( elt != _attributes.end() ) return;
    }
    _attributes[ name ] = value;
}
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& os, const Object& o )
{
    return os << o.getId();
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
