/* -*-c++-*- libcitygml - Copyrigdht (c) 2010 Joachim Pouderoux, BRGM
 *
 * Contributors:
 *  - Manuel Garnier, BRGM - better normal computation
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
#include "tesselator.hpp"
#include "citygml.hpp"
#include "utils.hpp"
#include <string.h>
#include <limits>
#include <iterator>
#include <set>
#include <algorithm>

#include "gui/applicationGui.hpp"
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
CityModel::CityModel( const std::string& id)
    : Object( id )
{
}
////////////////////////////////////////////////////////////////////////////////
CityModel::~CityModel( void )
{
    /*CityObjectsMap::const_iterator it = _cityObjectsMap.begin();
    for ( ; it != _cityObjectsMap.end(); ++it )
        for ( unsigned int i = 0; i < it->second.size(); i++ )
            delete it->second[i];*/

    for(CityObject* obj : _roots)
    {
        delete obj;
    }
}
////////////////////////////////////////////////////////////////////////////////
// Return the envelope (ie. the bounding box) of the model
const Envelope& CityModel::getEnvelope( void ) const
{
    return _envelope;
}
////////////////////////////////////////////////////////////////////////////////
Envelope& CityModel::getEnvelope( void )
{
    return _envelope;
}
////////////////////////////////////////////////////////////////////////////////
// Return the translation parameters of the model
const TVec3d& CityModel::getTranslationParameters( void ) const
{
    return _translation;
}
////////////////////////////////////////////////////////////////////////////////
// Get the number of city objects
size_t CityModel::size( void ) const
{
    size_t count = 0;
    CityObjectsMap::const_iterator it = _cityObjectsMap.begin();
    for ( ; it != _cityObjectsMap.end(); ++it ) count += it->second.size();
    return count;
}
////////////////////////////////////////////////////////////////////////////////
const CityObjectsMap& CityModel::getCityObjectsMap( void ) const
{
    return _cityObjectsMap;
}
////////////////////////////////////////////////////////////////////////////////
CityObjectsMap& CityModel::getCityObjectsMap( void )
{
    return _cityObjectsMap;
}
////////////////////////////////////////////////////////////////////////////////
const CityObjects* CityModel::getCityObjectsByType( CityObjectsType type ) const
{
    CityObjectsMap::const_iterator it = _cityObjectsMap.find( type );
    return ( it != _cityObjectsMap.end() ) ? &it->second : 0;
}
////////////////////////////////////////////////////////////////////////////////
// Return the roots elements of the model. You can then navigate the hierarchy using object->getChildren().
const CityObjects& CityModel::getCityObjectsRoots( void ) const
{
    return _roots;
}
////////////////////////////////////////////////////////////////////////////////
CityObjects& CityModel::getCityObjectsRoots( void )
{
    return _roots;
}
////////////////////////////////////////////////////////////////////////////////
const std::string& CityModel::getSRSName( void ) const
{
    return _srsName;
}
////////////////////////////////////////////////////////////////////////////////
AppearanceManager* CityModel::getAppearanceManager()
{
    return &_appearanceManager;
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::addCityObjectAsRoot( CityObject* o )
{
    if ( o )
        _roots.push_back( o );
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::addCityObject( CityObject* o )
{
    CityObjectsMap::iterator it = _cityObjectsMap.find( o->getType() );
    if ( it == _cityObjectsMap.end() )
    {
        CityObjects v;
        v.push_back( o );
        _cityObjectsMap[ o->getType() ] = v;
    }
    else
        it->second.push_back( o );
}
////////////////////////////////////////////////////////////////////////////////
CityObject* CityModel::getNode(const vcity::URI& uri)
{
	for(CityObject* obj : _roots)
	{
		if(uri.getCurrentNode() == obj->getId())
		{
			uri.popFront();
			return obj->getNode(uri);
		}
	}

	return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::finish( const ParserParams& params )
{
    // Assign appearances to cityobjects => geometries => polygons
    CityObjectsMap::const_iterator it = _cityObjectsMap.begin();
    for ( ; it != _cityObjectsMap.end(); ++it )
        for ( unsigned int i = 0; i < it->second.size(); i++ )
            it->second[i]->finish( _appearanceManager, params );

    _appearanceManager.finish();
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::computeEnvelope()
{
    for(CityObject* obj : _roots)
    {
        obj->computeEnvelope();
        _envelope.merge(obj->getEnvelope());
    }
}
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& out, const CityModel& model )
{
    out << "  Envelope: " << model.getEnvelope() << std::endl;

    const CityObjectsMap& cityObjectsMap = model.getCityObjectsMap();

    CityObjectsMap::const_iterator it = cityObjectsMap.begin();

    for ( ; it != cityObjectsMap.end(); ++it )

        for ( unsigned int i = 0; i < it->second.size(); i++ ) out << *(it->second[i]);

    out << model.size() << " city objects." << std::endl;

    return out;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
