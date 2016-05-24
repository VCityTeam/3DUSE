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
	for(temporal::Version* version : _versions) delete version;
	for(temporal::VersionTransition* trans : _versionTransitions) delete trans;
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
CityObject* getNodeByIdRec(CityObject* node, const std::string& id)
{
    CityObject* res = nullptr;

    if(node->getId() == id)
    {
        return node;
    }

    for(auto* child : node->getChildren())
    {
        res = getNodeByIdRec(child, id);
        if(res) break;
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
CityObject* CityModel::getNodeById(const std::string& id)
{
    CityObject* res = nullptr;
    for(auto* child : _roots)
    {
        res = getNodeByIdRec(child, id);
        if(res) break;
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
CityObject* CityModel::getNode(const vcity::URI& uri, bool inPickingMode)
{
	std::string sNode;	

	if (inPickingMode)
	{
		if (uri.getCurrentNodeType() == "Workspace")	
		{
			uri.popFront();
			//std::cout << "---> POP because Workspace" << std::endl;
		}
		if (uri.getCurrentNodeType() == "Version")	
		{
			uri.popFront();
			//std::cout << "---> POP because Version" << std::endl;
		}

		sNode = uri.getCurrentNode();

		//std::cout << "(Picking) uri.getStringURI: " << uri.getStringURI() << std::endl;
		//std::cout << "(Picking) uri.getLastNode: " << uri.getLastNode() << std::endl;
		//std::cout << "(Picking) -> uri.getCurrentNode: " << uri.getCurrentNode() << std::endl << std::endl;
	}
	else
	{
		sNode = uri.getLastNode();

		//std::cout << "uri.getStringURI: " << uri.getStringURI() << std::endl;
		//std::cout << " -> uri.getLastNode: " << uri.getLastNode() << std::endl;
		//std::cout << "uri.getCurrentNode: " << uri.getCurrentNode() << std::endl << std::endl;
	}

	for(CityObject* obj : _roots)
	{
		if(/*uri.getCurrentNode()*/sNode == obj->getId())
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
		if(obj->IsEmpty())
			continue;

        obj->computeEnvelope();

		if(obj->getEnvelope().getUpperBound().x > 1000000000) //Pour pas qu'un batiment qui bug gene le calcul de l'enveloppe
			continue;
		
		//TVec3d Low = _envelope.getLowerBound();
		//TVec3d Up = _envelope.getUpperBound();
        _envelope.merge(obj->getEnvelope());

		/*if(_envelope.getLowerBound() != Low || _envelope.getUpperBound() != Up)
		{
			std::cout << obj->getId() << std::endl;
			std::cout << obj->getEnvelope().getLowerBound() << std::endl;
			std::cout << obj->getEnvelope().getUpperBound() << std::endl;
			std::cout << _envelope.getLowerBound() << std::endl;
			std::cout << _envelope.getUpperBound() << std::endl;
			int a;
			std::cin >> a;
		}*/
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
void CityModel::setVersions(std::vector<temporal::Version*> versionsList,std::vector<temporal::VersionTransition*> transitionsList)
{
	_versions = versionsList;
	_versionTransitions = transitionsList;
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<temporal::Version*> CityModel::getVersions() const
{
	return _versions;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<temporal::Version*> CityModel::getVersions()
{
	return _versions;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<temporal::VersionTransition*> CityModel::getTransitions()
{
	return _versionTransitions;
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::setWorkspaces(std::map<std::string,temporal::Workspace> wrkspslist)
{
	_workspaces=wrkspslist; 
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::setDocuments(std::vector<documentADE::DocumentObject*> documentslist)
{
    _documents=documentslist;
}
////////////////////////////////////////////////////////////////////////////////
void CityModel::setReferences(std::vector<documentADE::Reference*> referencelist)
{
    _references=referencelist;
}
////////////////////////////////////////////////////////////////////////////////
const std::map<std::string,temporal::Workspace> CityModel::getWorkspaces() const
{
	return _workspaces;
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<documentADE::Reference*> CityModel::getReferences() const
{
    return _references;
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<documentADE::DocumentObject*> CityModel::getDocuments() const
{
    return _documents;
}
////////////////////////////////////////////////////////////////////////////////
std::map<std::string,temporal::Workspace> CityModel::getWorkspaces()
{
	return _workspaces;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
