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
#include "geometry.hpp"
#include "polygon.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
Geometry::Geometry( const std::string& id, GeometryType type, unsigned int lod)
    : Object( id ), _type( type ), _lod( lod )
{}
////////////////////////////////////////////////////////////////////////////////
Geometry::~Geometry()
{
    for(Polygon* poly : _polygons)
        delete poly;
}
////////////////////////////////////////////////////////////////////////////////
// Get the geometry LOD
unsigned int Geometry::getLOD( void ) const
{
    return _lod;
}
////////////////////////////////////////////////////////////////////////////////
// Return the envelope (ie. the bounding box) of the object
const Envelope& Geometry::getEnvelope( void ) const
{
    return _envelope;
}
////////////////////////////////////////////////////////////////////////////////
// Get the polygons
size_t Geometry::size( void ) const
{
    return _polygons.size();
}
////////////////////////////////////////////////////////////////////////////////
Polygon* Geometry::operator[](size_t i )
{
    return _polygons[i];
}
////////////////////////////////////////////////////////////////////////////////
const Polygon* Geometry::operator[]( size_t i ) const
{
    return _polygons[i];
}
////////////////////////////////////////////////////////////////////////////////
const std::vector< Polygon* >& Geometry::getPolygons() const
{
    return _polygons;
}
////////////////////////////////////////////////////////////////////////////////
std::vector< Polygon* >& Geometry::getPolygons()
{
    return _polygons;
};
////////////////////////////////////////////////////////////////////////////////
GeometryType Geometry::getType( void ) const
{
    return _type;
}
////////////////////////////////////////////////////////////////////////////////
CityObject* Geometry::getParent()
{
    return _parent;
}
////////////////////////////////////////////////////////////////////////////////
void Geometry::addPolygon( Polygon* p )
{
    p->_geometry = this;
    _polygons.push_back( p );
}
////////////////////////////////////////////////////////////////////////////////
void Geometry::finish( AppearanceManager& appearanceManager, Appearance* defAppearance,  const ParserParams& params )
{
    Appearance* myappearance = appearanceManager.getAppearance( getId() );
    std::vector< Polygon* >::const_iterator it = _polygons.begin();
    for ( ; it != _polygons.end(); ++it ) (*it)->finish( appearanceManager, myappearance ? myappearance : defAppearance, params.tesselate );

    bool finish = false;
    while ( !finish && params.optimize )
    {
        finish = true;
        int len = (int)_polygons.size();
        for ( int i = 0; finish && i < len - 1; i++ )
        {
            for ( int j = i+1; finish && j < len - 1; j++ )
            {
                if ( !_polygons[i]->merge( _polygons[j] ) ) continue;
                delete _polygons[j];
                _polygons.erase( _polygons.begin() + j );
                finish = false;
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
bool Geometry::merge( Geometry* g )
{
    if ( !g || g->_lod != _lod || g->_type != _type ) return false;

    size_t pSize = g->_polygons.size();
    for ( size_t i = 0; i < pSize; i++ )
        _polygons.push_back( g->_polygons[i] );

    g->_polygons.clear();

    _id += "+" + g->_id;

    return true;
}
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& os, const citygml::Geometry& s )
{
    size_t count = 0;
    for ( size_t i = 0; i < s.size(); i++ )
    {
        os << *s[i];
        count += s[i]->getVertices().size();
    }

    os << "  @ " << s._polygons.size() << " polys [" << count << " vertices]" << std::endl;

    return os;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
