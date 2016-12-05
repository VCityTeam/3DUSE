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
#ifndef __CITYGML_POLYGON_HPP__
#define __CITYGML_POLYGON_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "object.hpp"
#include "vecs.hpp"
#include "appearance.hpp"
#include "appearancemanager.hpp"
#include "material.hpp"
#include "texture.hpp"
#include "linearring.hpp"
#include "geometry.hpp"
#include "envelope.hpp"
#include "citygmltypes.hpp"
#include "citygml_export.h"
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
#pragma warning(disable: 4251) // export problem on STL members
#endif

////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Geometry;
class CITYGML_EXPORT Polygon : public Object
{
    friend class CityGMLHandler;
    friend class Geometry;
    friend class Tesseletor;
    friend class CityModel;
public:
    enum AppearanceSide {
        FRONT = 0,
        BACK,
        _NUMBER_OF_SIDES
    };

    Polygon( const std::string& id );

    virtual ~Polygon( void ) override;

	Polygon* Clone();

    // Get the vertices
    const std::vector<TVec3d>& getVertices( void ) const;

    // Get the indices
    const std::vector<unsigned int>& getIndices( void ) const;

    // Get the normals
    const std::vector<TVec3f>& getNormals( void ) const;

    // Get texture coordinates
    TexCoords& getTexCoords( void );
    const TexCoords& getTexCoords( void ) const;

    // Get the appearance
    const Appearance* getAppearance( void ) const; // Deprecated! Use getMaterial and getTexture instead
    const Material* getMaterial( void ) const;
    const Texture* getTexture( void ) const;
    const Material* getMaterialFront( void ) const;
    const Material* getMaterialBack( void ) const;

    const std::vector<LinearRing*>& getInteriorRings() const;
    std::vector<LinearRing*>& getInteriorRings();

    const LinearRing* getExteriorRing() const;
    LinearRing* getExteriorRing();

    // Return the envelope (ie. the bounding box) of the object
    const Envelope& getEnvelope( void ) const;

//	protected:
    void finish( AppearanceManager&, bool doTesselate );
    void finish( AppearanceManager&, Appearance*,  bool doTesselate );

    void addRing( LinearRing* );

    void tesselate( AppearanceManager &, const TVec3d& );
    void mergeRings( AppearanceManager & );
    void clearRings( void );

    TVec3d computeNormal( void );

    bool merge( Polygon* );

protected:
    std::vector<TVec3d> _vertices;
    std::vector<TVec3f> _normals;
    std::vector<unsigned int> _indices;

    Appearance* _appearance;
    Material* _materials[ _NUMBER_OF_SIDES ];
    Texture* _texture;

    TexCoords _texCoords;

    LinearRing* _exteriorRing;
    std::vector<LinearRing*> _interiorRings;

    bool _negNormal;

    Geometry *_geometry;

    Envelope _envelope;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_POLYGON_HPP__
