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
#ifndef __CITYGML_CITYMODEL_HPP__
#define __CITYGML_CITYMODEL_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
#include "envelope.hpp"
#include "cityobject.hpp"
#include "appearancemanager.hpp"
#include "vecs.hpp"
#include "core/URI.hpp"
#include <vector>
#include <map>
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
typedef std::vector< CityObject* > CityObjects;
typedef std::map< CityObjectsType, CityObjects > CityObjectsMap;
////////////////////////////////////////////////////////////////////////////////
class CityModel : public Object
{
    friend class CityGMLHandler;
public:
    CityModel( const std::string& id = "CityModel" );

    LIBCITYGML_EXPORT ~CityModel( void ) override;

    // Return the envelope (ie. the bounding box) of the model
    const Envelope& getEnvelope( void ) const;
    Envelope& getEnvelope( void );

    // Return the translation parameters of the model
    const TVec3d& getTranslationParameters( void ) const;

    // Get the number of city objects
    size_t size( void ) const;

    const CityObjectsMap& getCityObjectsMap( void ) const;
    CityObjectsMap& getCityObjectsMap( void );

    const CityObjects* getCityObjectsByType( CityObjectsType type ) const;

    // Return the roots elements of the model. You can then navigate the hierarchy using object->getChildren().
    const CityObjects& getCityObjectsRoots( void ) const;
    CityObjects& getCityObjectsRoots( void );

    const std::string& getSRSName( void ) const;

    void computeEnvelope();

    AppearanceManager* getAppearanceManager();

    /// Add a direct child
    void addCityObjectAsRoot( CityObject* o );

    /// Add a CityObject to the model (used for finish method for example)
    ///
    /// A CityObject should always be added on the CityObject and on the master CityModel
    ///
    /// Example usage : model is the CityModel, obj is a CityObject to add to a Wall
    /// \code{.cpp}
    /// model->addCityObject(obj);
    /// wall->insertNode(obj);
    /// \endcode
    void addCityObject( CityObject* o );

    /// Get node by uri
	CityObject* getNode(const vcity::URI& uri);

    /// Get node by name
    CityObject* getNodeById(const std::string& id);

    void finish( const ParserParams& );

    std::string m_basePath;

protected:
    Envelope _envelope;

    CityObjects _roots;

    CityObjectsMap _cityObjectsMap;

    AppearanceManager _appearanceManager;

    std::string _srsName;

    TVec3d _translation;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream&, const citygml::CityModel & );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_CITYMODEL_HPP__
