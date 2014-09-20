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
#ifndef __CITYGML_APPEARANCEMANAGER_HPP__
#define __CITYGML_APPEARANCEMANAGER_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "appearance.hpp"
#include "texture.hpp"
#include "georeferencedtexture.hpp"
#include "material.hpp"
#include "tesselator.hpp"
#include "citygmltypes.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class AppearanceManager
{
    friend class CityGMLHandler;
    friend class CityModel;
public:
    AppearanceManager( void );

    ~AppearanceManager( void );

    enum ForSide
    {
        FS_ANY = 0,	// appearance for any side
        FS_FRONT,	// appearance for front side
        FS_BACK		// appearance for back side
    };

    Appearance* getAppearance( const std::string& nodeid ) const;

    Material* getMaterial( const std::string& nodeid ) const;

    Texture* getTexture( const std::string& nodeid ) const;

    GeoreferencedTexture* getGeoreferencedTexture( const std::string& nodeid ) const;

    // Getter for the front&back material if there is any.
    Material* getMaterialFront( const std::string& nodeid ) const;

    Material* getMaterialBack( const std::string& nodeid ) const;

    bool getTexCoords( const std::string& nodeid, TexCoords &texCoords) const;

    Tesselator* getTesselator( void );

    void refresh( void );

    template < typename AppType > AppType getAppearance( const std::string& nodeid, ForSide side = FS_ANY ) const;
    void addAppearance( Appearance* );
    void assignNode( const std::string& nodeid );
    bool assignTexCoords( TexCoords* );

    void finish( void );

    std::string m_basePath;

protected:
    std::string _lastId;
    TexCoords* _lastCoords;

    std::vector< Appearance* > _appearances;

    std::map< std::string, std::vector< Appearance* > > _appearancesMap;

    std::map<std::string, TexCoords*> _texCoordsMap;
    std::vector<TexCoords*> _obsoleteTexCoords;

    Tesselator* _tesselator;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_APPEARANCEMANAGER_HPP__
