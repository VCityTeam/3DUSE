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

#ifndef __CITYGML_H__
#define __CITYGML_H__

#include "citymodel.hpp"
#include "cityobject.hpp"
#include "citygmlcommon.hpp"

/*#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include "vecs.hpp"
#include <QDateTime>

#include "temporalExt.hpp"*/

//#include <osg/Group>
/*namespace osg
{
    class Group;
};*/



class Tesselator;

namespace citygml 
{
enum State {
        Build = 1 << 0,
        Destroyed = 1 << 1,
        Modified = 1 << 2,
        Burn
};

	class CityModel;
    class CityObject;



	///////////////////////////////////////////////////////////////////////////////
	// Parsing routines

	// Parameters:
	// objectsMask: a string describing the objects types that must or must not be parsed
	//    examples: "All&~LandUse&~TINRelief" to parse everything but landuses and TIN reliefs
	//              "Road&Railway" to parse only roads & railways
	// minLOD: the minimal LOD that will be parsed
	// maxLOD: the maximal LOD that will be parsed
	// optimize: merge geometries & polygons that share the same appearance in the same object in order to reduce the global hierarchy
	// pruneEmptyObjects: remove the objects which do not contains any geometrical entity
	// tesselate: convert the interior & exteriors polygons to triangles
	// destSRS: the SRS (WKT, EPSG, OGC URN, etc.) where the coordinates must be transformed, default ("") is no transformation



	LIBCITYGML_EXPORT CityModel* load( std::istream& stream, const ParserParams& params );

    LIBCITYGML_EXPORT CityModel* load( const std::string& fileName, ParserParams& params );

	///////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////////////////




	///////////////////////////////////////////////////////////////////////////////





	///////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////////////////

	class Geometry;



	///////////////////////////////////////////////////////////////////////////////





	///////////////////////////////////////////////////////////////////////////////

    class Site
    {

    };

    class AbstractBuilding
    {

    };
    


    ///////////////////////////////////////////////////////////////////////////////

	LIBCITYGML_EXPORT std::string getCityObjectsClassName( CityObjectsTypeMask mask );

	LIBCITYGML_EXPORT CityObjectsTypeMask getCityObjectsTypeMaskFromString( const std::string& stringMask );



#define MAKE_RGBA( _r_, _g_, _b_, _a_ ) TVec4f( _r_/255.f, _g_/255.f, _b_/255.f, _a_/255.f )
#define MAKE_RGB( _r_, _g_, _b_ ) MAKE_RGBA( _r_, _g_, _b_, 255 )

	// Helper macro to declare a new CityObject type from its name & default color
#define DECLARE_SIMPLE_OBJECT_CLASS( _name_, _defcolor_ ) \
	class _name_ : public CityObject \
	{\
	public:\
	_name_( const std::string& id ) : CityObject( id, COT_ ## _name_ ) {}\
	inline TVec4f getDefaultColor( void ) const { return _defcolor_; }\
	};

	///////////////////////////////////////////////////////////////////////////////

	DECLARE_SIMPLE_OBJECT_CLASS( Building, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BuildingPart, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Room, MAKE_RGB( 181, 180, 163 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Door, MAKE_RGB( 145, 53, 13 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Window, MAKE_RGBA( 147, 170, 209, 60 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BuildingInstallation, MAKE_RGB( 186, 186, 177 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BuildingFurniture, MAKE_RGB( 227, 225, 157 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( CityFurniture, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( WaterBody, MAKE_RGB( 48, 133, 187 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( PlantCover, MAKE_RGB( 0, 184, 0 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( SolitaryVegetationObject, MAKE_RGB( 10, 184, 10 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Track, MAKE_RGB( 171, 131, 46 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Road, MAKE_RGB( 159, 159, 159 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Railway, MAKE_RGB( 180, 180, 180 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Square, MAKE_RGB( 159, 159, 159 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( TINRelief, MAKE_RGB( 100, 230, 10 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Tunnel, MAKE_RGB( 180, 180, 150 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( Bridge, MAKE_RGB( 245, 30, 30 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BridgeConstructionElement, MAKE_RGB( 245, 20, 20 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BridgeInstallation, MAKE_RGB( 245, 80, 80 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( BridgePart, MAKE_RGB( 245, 50, 50 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( GenericCityObject, MAKE_RGB( 100, 130, 0 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( WallSurface, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( RoofSurface, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( GroundSurface, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( ClosureSurface, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( FloorSurface, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( InteriorWallSurface, MAKE_RGB( 186, 184, 135 ) );

	DECLARE_SIMPLE_OBJECT_CLASS( CeilingSurface, MAKE_RGB( 186, 184, 135 ) );

    //DECLARE_SIMPLE_OBJECT_CLASS( TrafficArea, MAKE_RGB( 159, 159, 159 ) );


	class LandUse : public CityObject 
	{
	public:
		LandUse( const std::string& id ) : CityObject( id, COT_LandUse ) {}

		inline TVec4f getDefaultColor( void ) const
		{ 
            std::string c = getAttribute( "class" );
			if ( c != "" )
			{
				int cl = atoi( c.c_str() );
				switch ( cl ) 
				{
				case 1000: return MAKE_RGB( 150, 143, 134 );	// Settlement Area
				case 1100: return MAKE_RGB( 133, 83, 101 );		// Undeveloped Area
				case 2000: return MAKE_RGB( 159, 159, 159 );	// Traffic
				case 3000: return MAKE_RGB( 79, 212, 53 );		// Vegetation
				case 4000: return MAKE_RGB( 67, 109, 247 );		// Water
				}
			}
			return MAKE_RGB( 10, 230, 1 ); 
		}
	};

	///////////////////////////////////////////////////////////////////////////////





	///////////////////////////////////////////////////////////////////////////////






}

#endif // __CITYGML_H__
