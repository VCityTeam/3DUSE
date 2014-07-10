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


	///////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////////////////


		
	///////////////////////////////////////////////////////////////////////////////







	///////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
CityModel::~CityModel( void )
{
    CityObjectsMap::const_iterator it = _cityObjectsMap.begin();
    for ( ; it != _cityObjectsMap.end(); ++it )
        for ( unsigned int i = 0; i < it->second.size(); i++ )
            delete it->second[i];
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
