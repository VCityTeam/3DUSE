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
#include "object.hpp"
#include "geometry.hpp"
#include "temporalExt.hpp"
#include "core/URI.hpp"
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
//forward declaration
class ADEHandler;
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
enum CityObjectsType {
    COT_GenericCityObject, // JE 13/01/16 - removeing filtering feature to allow having more than 32 COT
    COT_Building,
    COT_Room,
    COT_BuildingInstallation,
    COT_BuildingFurniture,
    COT_Door,
    COT_Window,
    COT_CityFurniture,
    COT_Track,
    COT_Road,
    COT_Railway,
    COT_Square,
    COT_PlantCover,
    COT_SolitaryVegetationObject,
    COT_WaterBody,
    COT_TINRelief,
    COT_LandUse,
    COT_Tunnel,
    COT_Bridge,
    COT_BridgeConstructionElement,
    COT_BridgeInstallation,
    COT_BridgePart,
    COT_BuildingPart,

    COT_WallSurface,
    COT_RoofSurface,
    COT_GroundSurface,
    COT_ClosureSurface,
    COT_FloorSurface,
    COT_InteriorWallSurface,
    COT_CeilingSurface,
    COT_WaterSurface,

    COT_All
};
typedef unsigned int CityObjectsTypeMask;

////////////////////////////////////////////////////////////////////////////////
class CityObject : public Object
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

    /// Add a State
    void addState(CityObjectState* state);

    /// Get States vector
    std::vector<CityObjectState*>& getStates();

    /// Get States vector (const)
    const std::vector<CityObjectState*>& getStates() const;

    /// Get a State by name
    /// \param name State name
    CityObjectState* getState(const std::string& name);

    /// Add a TAg
    void addTag(CityObjectTag* tag);

    /// Get Tags vector
    std::vector<CityObjectTag*>& getTags();

    /// get Tags vector (const)
    const std::vector<CityObjectTag*>& getTags() const;

    /// Internal method to reorganize Tags, reorder them by date
    void checkTags();

    /// Temporal check : tell if the object is temporal
    /// \return true if has Tags of States
    bool isTemporal() const;

    /// Get an attribute for a specific date (use when temporal)
    /// \param attribName Attribute name
    /// \param date Date wanted
    std::string getAttributeTemporal(const std::string& attribName, const QDateTime& date) const;

//protected:
    void finish( AppearanceManager&, const ParserParams& );


protected:
    CityObjectsType _type;

    Envelope _envelope;
	bool _isEmpty;
    TVec3d _centroid;

    std::vector< Geometry* > _geometries;
    std::vector< CityObject* > _children;

    std::vector<CityObjectState*> m_states;
    std::vector<CityObjectTag*> m_tags;

public:
    CityObject* _parent; // MT (MAC OS X problem...)
    
    std::string m_path;
    bool m_temporalUse;
};
////////////////////////////////////////////////////////////////////////////////
LIBCITYGML_EXPORT std::string getCityObjectsClassName( CityObjectsTypeMask mask );
LIBCITYGML_EXPORT CityObjectsTypeMask getCityObjectsTypeMaskFromString( const std::string& stringMask );
std::ostream& operator<<( std::ostream&, const citygml::CityObject& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_CITYOBJECT_HPP__
