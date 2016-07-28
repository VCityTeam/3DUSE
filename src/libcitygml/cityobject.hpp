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
#ifndef __CITYGML_CITYOBJECT_HPP__
#define __CITYGML_CITYOBJECT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <ostream>
#include "object.hpp"
#include "geometry.hpp"
#include "URI.hpp"
#include "citygml_export.h"
#pragma warning(disable: 4251) // VC++ DLL jejune complains on STL members

////////////////////////////////////////////////////////////////////////////////
//forward declaration
class ADEHandler;
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
enum CityObjectsType {
    COT_GenericCityObject           = 1 << 0,
    COT_Building                    = 1 << 1,
    COT_Room                        = 1 << 2,
    COT_BuildingInstallation        = 1 << 3,
    COT_BuildingFurniture           = 1 << 4,
    COT_Door                        = 1 << 5,
    COT_Window                      = 1 << 6,
    COT_CityFurniture               = 1 << 7,
    COT_Track                       = 1 << 8,
    COT_Road                        = 1 << 9,
    COT_Railway                     = 1 << 10,
    COT_Square                      = 1 << 11,
    COT_PlantCover                  = 1 << 12,
    COT_SolitaryVegetationObject    = 1 << 13,
    COT_WaterBody                   = 1 << 14,
    COT_TINRelief                   = 1 << 15,
    COT_LandUse                     = 1 << 16,
    COT_Tunnel						= 1 << 17,
    COT_Bridge						= 1 << 18,
    COT_BridgeConstructionElement	= 1 << 19,
    COT_BridgeInstallation			= 1 << 20,
    COT_BridgePart                  = 1 << 21,
    COT_BuildingPart                = 1 << 22,

    COT_WallSurface                 = 1 << 23,
    COT_RoofSurface                 = 1 << 24,
    COT_GroundSurface               = 1 << 25,
    COT_ClosureSurface              = 1 << 26,
    COT_FloorSurface                = 1 << 27,
    COT_InteriorWallSurface         = 1 << 28,
    COT_CeilingSurface              = 1 << 29,
    COT_All                         = 0xFFFFFFFF
};
typedef unsigned int CityObjectsTypeMask;

////////////////////////////////////////////////////////////////////////////////
class CITYGML_EXPORT CityObject : public Object
{
    friend class CityGMLHandler;
	friend class ADEHandler;
    friend class CityModel;
    friend std::ostream& operator<<( std::ostream&, const CityObject & );
public:
    CityObject( const std::string& id, CityObjectsType type );

    virtual ~CityObject() override;

    // Get the object type
    CityObjectsType getType( void ) const;

    std::string getTypeAsString( void ) const;

    // Return the envelope (ie. the bounding box) of the object
    const Envelope& getEnvelope( void ) const;

	//Check is CityObject has geometries
	bool IsEmpty();

    // Get the default diffuse color of this object class
    virtual TVec4f getDefaultColor( void ) const = 0;

    // Get the number of geometries contains in the object
    unsigned int size( void ) const;

    // Access the geometries
    const Geometry* getGeometry( unsigned int i ) const;

    std::vector< Geometry* >& getGeometries();
    const std::vector< Geometry* >& getGeometries() const;

    // Access the children
    unsigned int getChildCount( void ) const;

    const CityObject* getChild( unsigned int i ) const;

    CityObject* getChild( unsigned int i );

    const std::vector< CityObject* >& getChildren( void ) const;

    std::vector< CityObject* >& getChildren( void );

	//remove all the children of the CityObject (without deleting them)
	void clearChildren();

    void addGeometry(Geometry* geom);

    void computeEnvelope();

    void computeCentroid();

    CityObject* getParent();

    void deleteNode(const std::string& node);
    void deleteNode(CityObject* node);

    void insertNode(CityObject* node);

    /// Get a node from a uri
    /// \param uri uri pointing to requested node
	CityObject* getNode(const vcity::URI& uri);

//protected:
    void finish( AppearanceManager&, const ParserParams& );

protected:
    CityObjectsType _type;

    Envelope _envelope;
	bool _isEmpty;
    TVec3d _centroid;

    std::vector< Geometry* > _geometries;
    std::vector< CityObject* > _children;

public:
    CityObject* _parent; // MT (MAC OS X problem...)
    
    std::string m_path;
    bool m_temporalUse;
};
////////////////////////////////////////////////////////////////////////////////
CITYGML_EXPORT std::string getCityObjectsClassName( CityObjectsTypeMask mask );
CITYGML_EXPORT CityObjectsTypeMask getCityObjectsTypeMaskFromString( const std::string& stringMask );
std::ostream& operator<<( std::ostream&, const citygml::CityObject& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_CITYOBJECT_HPP__
