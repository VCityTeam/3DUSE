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
#ifndef __CITYGML_GEOMETRY_HPP__
#define __CITYGML_GEOMETRY_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
#include "envelope.hpp"
#include "polygon.hpp"
#include "parserparams.hpp"
#include "citygml_export.h"
#include <vector>
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Polygon;
class CityObject;
enum GeometryType
{
    GT_Unknown = 0,
    GT_Roof,
    GT_Wall,
    GT_Ground,
    GT_Closure,
    GT_Floor,
    GT_InteriorWall,
    GT_Ceiling,
};
////////////////////////////////////////////////////////////////////////////////
/// \brief Geometry class
///
////////////////////////////////////////////////////////////////////////////////
class CITYGML_EXPORT Geometry : public Object
{
    friend class CityGMLHandler;
    friend class CityObject;
    friend std::ostream& operator<<( std::ostream&, const citygml::Geometry& );
public:
    Geometry( const std::string& id, GeometryType type = GT_Unknown, unsigned int lod = 0 );

    virtual ~Geometry() override;

    // Get the geometry LOD
    unsigned int getLOD( void ) const;

    // Return the envelope (ie. the bounding box) of the object
    const Envelope& getEnvelope( void ) const;

    // Get the polygons
    size_t size( void ) const;
    Polygon* operator[]( size_t i );
    const Polygon* operator[]( size_t i ) const;

    const std::vector< Polygon* >& getPolygons() const;
    std::vector< Polygon* >& getPolygons();

    GeometryType getType( void ) const;

    const CityObject* getParent() const;
    CityObject* getParent();

    void addPolygon( Polygon* );

    void finish( AppearanceManager&, Appearance*, const ParserParams& );

    bool merge( Geometry* );

protected:
    GeometryType _type;
    CityObject* _parent;

    unsigned int _lod;

    Envelope _envelope;

    std::vector< Polygon* > _polygons;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream&, const citygml::Geometry& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_GEOMETRY_HPP__
