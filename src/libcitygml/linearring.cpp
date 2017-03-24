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
#include "linearring.hpp"
#include <limits>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
LinearRing::LinearRing( const std::string& id, bool isExterior )
    : Object( id ), _exterior( isExterior )
{}
////////////////////////////////////////////////////////////////////////////////
bool LinearRing::isExterior() const
{
    return _exterior;
}
////////////////////////////////////////////////////////////////////////////////
unsigned int LinearRing::size() const
{
    return (unsigned int)_vertices.size();
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<TVec3d>& LinearRing::getVertices() const
{
    return _vertices;
}
////////////////////////////////////////////////////////////////////////////////
void LinearRing::addVertex( const TVec3d& v )
{
    _vertices.push_back( v );
}
////////////////////////////////////////////////////////////////////////////////
// Return the envelope (ie. the bounding box) of the object
const Envelope& LinearRing::getEnvelope() const
{
    return _envelope;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<TVec3d>& LinearRing::getVertices()
{
    return _vertices;
}
////////////////////////////////////////////////////////////////////////////////
TVec3d LinearRing::computeNormal( void ) const
{
    unsigned int len = size();
    if ( len < 3 ) return TVec3d();

    // Tampieri, F. 1992. Newell's method for computing the plane equation of a polygon. In Graphics Gems III, pp.231-232.
    TVec3d n( 0., 0., 0. );
    for ( unsigned int i = 0; i < len; i++ )
    {
        const TVec3d& current = _vertices[i];
        const TVec3d& next = _vertices[ ( i + 1 ) % len];

        n.x += ( current.y - next.y ) * ( current.z + next.z );
        n.y += ( current.z - next.z ) * ( current.x + next.x );
        n.z += ( current.x - next.x ) * ( current.y + next.y );
    }
    return n.normal();
}
////////////////////////////////////////////////////////////////////////////////
void LinearRing::finish( TexCoords* texCoords )
{
    // Remove last vertex which is supposed to be the same as the first one

   unsigned int len = (unsigned int)_vertices.size();
   if ( len < 2 ) return;

   if ( ( _vertices[0] - _vertices[len - 1] ).sqrLength() <= std::numeric_limits<double>::epsilon() )
      _vertices.erase( _vertices.begin() + len - 1 );
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
