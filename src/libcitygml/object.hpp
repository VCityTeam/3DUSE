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
#ifndef __CITYGML_OBJECT_HPP__
#define __CITYGML_OBJECT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
#include <map>
#include <ostream>
#include <vector>

#include "citygml_export.h"
#pragma warning(disable: 4251) // VC++ DLL crap complains on STL _Id member

////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
enum xLinkState { NONE, LINKED, UNLINKED, TARGET };
////////////////////////////////////////////////////////////////////////////////
typedef std::map< std::string, std::string > AttributesMap;

///////////////////////////////////////////////////////////////////////////////
// Base object associated with an unique id and a set of attributes (key-value pairs)
class CITYGML_EXPORT Object
{
    friend class CityGMLHandler;
    friend std::ostream& operator<<( std::ostream&, const Object & );
public:
    Object( const std::string& id );
    virtual ~Object( void );

    const std::string& getId( void ) const;

    std::string getAttribute( const std::string& name ) const;

    const AttributesMap& getAttributes() const;

    AttributesMap& getAttributes();

    //inline osg::ref_ptr<osg::Group> getOsgNode() { return m_osgNode; }
    //inline void setOsgNode(osg::ref_ptr<osg::Group> node) { m_osgNode = node; }

    //inline osg::Group* getOsgNode() { return m_osgNode; }
    //inline void setOsgNode(osg::Group* node) { m_osgNode->ref(); m_osgNode = node; }
    //inline void setOsgNode(osg::Group* node) { m_osgNode = node; }

    void setAttribute( const std::string& name, const std::string& value, bool forceOnExist = true );

	std::vector<Object*>& getXLinkTargets();
	void addXLinkTarget(Object*);
	xLinkState _isXlink;

	bool _isInVersion;
	bool _isInWorkspace;

protected:
    std::string _id;

    AttributesMap _attributes;

	std::vector<Object*> _xLinkTargets;

    //osg::ref_ptr<osg::Group> m_osgNode;
    //osg::Group* m_osgNode;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream&, const citygml::Object& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_OBJECT_HPP__
