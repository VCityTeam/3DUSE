////////////////////////////////////////////////////////////////////////////////
#include "appearancemanager.hpp"
#include <set>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
AppearanceManager::AppearanceManager( void ) : _lastId( "" ), _lastCoords( 0 )
{
    _tesselator = new ::Tesselator();
}
////////////////////////////////////////////////////////////////////////////////
AppearanceManager::~AppearanceManager( void )
{
    for ( unsigned int i = 0; i < _appearances.size(); i++ ) delete _appearances[i];

    std::set<TexCoords*> texCoords;
    for ( std::map<std::string, TexCoords*>::iterator it = _texCoordsMap.begin(); it != _texCoordsMap.end(); ++it )
    {
        if ( it->second && texCoords.find(it->second) == texCoords.end() )
        {
            texCoords.insert(it->second);
            delete it->second;
        }
    }

    for ( std::vector<TexCoords*>::iterator it = _obsoleteTexCoords.begin(); it != _obsoleteTexCoords.end(); it++ )
        if ( texCoords.find(*it) == texCoords.end() )
            delete *it;

    delete _tesselator;
}
////////////////////////////////////////////////////////////////////////////////
Appearance* AppearanceManager::getAppearance( const std::string& nodeid ) const
// Deprecated, use getMaterial and getTexture instead.
{
    return getAppearance< Appearance* >( nodeid );
}////////////////////////////////////////////////////////////////////////////////
Material* AppearanceManager::getMaterial( const std::string& nodeid ) const
{
    return getAppearance< Material* >( nodeid );
}
////////////////////////////////////////////////////////////////////////////////
Texture* AppearanceManager::getTexture( const std::string& nodeid ) const
{
    return getAppearance< Texture* >( nodeid );
}
////////////////////////////////////////////////////////////////////////////////
GeoreferencedTexture* AppearanceManager::getGeoreferencedTexture( const std::string& nodeid ) const
{
    return getAppearance< GeoreferencedTexture* >( nodeid );
}
////////////////////////////////////////////////////////////////////////////////
// Getter for the front&back material if there is any.
Material* AppearanceManager::getMaterialFront( const std::string& nodeid ) const
{
    return getAppearance< Material* >( nodeid, FS_FRONT );
}////////////////////////////////////////////////////////////////////////////////
Material* AppearanceManager::getMaterialBack( const std::string& nodeid ) const
{
    return getAppearance< Material* >( nodeid, FS_BACK );
}
////////////////////////////////////////////////////////////////////////////////
bool AppearanceManager::getTexCoords( const std::string& nodeid, TexCoords &texCoords) const
{
    texCoords.clear();
    std::map<std::string, TexCoords*>::const_iterator it = _texCoordsMap.find( nodeid );
    if ( it == _texCoordsMap.end() || !it->second ) return false;
    texCoords = *it->second;
    return true;
}
////////////////////////////////////////////////////////////////////////////////
Tesselator* AppearanceManager::getTesselator( void )
{
    return _tesselator;
}
////////////////////////////////////////////////////////////////////////////////
void AppearanceManager::refresh( void )
{
    _lastCoords = 0;
    _lastId = "";
}
////////////////////////////////////////////////////////////////////////////////
template <typename AppType>
AppType AppearanceManager::getAppearance( const std::string& nodeid, ForSide side /*= FS_ANY*/ ) const
{
    std::map< std::string, std::vector< Appearance* > >::const_iterator map_iterator = _appearancesMap.find( nodeid );
    if ( map_iterator == _appearancesMap.end() ) return 0;

    std::vector< Appearance* >::const_iterator vector_iterator = ( map_iterator->second ).begin();
    for( ; vector_iterator != ( map_iterator->second ).end(); ++vector_iterator ) {
        if ( AppType appType = dynamic_cast< AppType >( *vector_iterator ) ) {
            if ( side == FS_ANY ||
                ( side == FS_FRONT && appType->getIsFront() ) ||
                ( side == FS_BACK && !appType->getIsFront() ) )
            {
                return appType;
            }
        }
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
void AppearanceManager::addAppearance( Appearance* app )
{
    if ( app ) _appearances.push_back( app );
}
////////////////////////////////////////////////////////////////////////////////
void AppearanceManager::assignNode( const std::string& nodeid )
{
    _lastId = nodeid;

    if ( !getAppearance< Appearance * >( nodeid ) )
        _appearancesMap[ nodeid ] = std::vector< Appearance* >(0);

    Appearance* currentAppearance = _appearances[ _appearances.size() - 1 ];
    ForSide side = currentAppearance->getIsFront() ? FS_FRONT : FS_BACK;
    if ( (dynamic_cast< Texture* >( currentAppearance ) && !getAppearance< Texture* >( nodeid, side )) ||
         (dynamic_cast< Material* >( currentAppearance ) && !getAppearance< Material* >( nodeid, side )) )
    {
        (_appearancesMap[ nodeid ]).push_back( currentAppearance );
        if ( _lastCoords ) { assignTexCoords( _lastCoords ); _lastId = ""; }
    }
}
////////////////////////////////////////////////////////////////////////////////
bool AppearanceManager::assignTexCoords( TexCoords* tex )
{
    _lastCoords = tex;
    if ( _lastId == "" )
    {
        _obsoleteTexCoords.push_back( tex );
        return false;
    }
    _texCoordsMap[ _lastId ] = tex;
    _lastCoords = 0;
    _lastId = "";
    return true;
}
////////////////////////////////////////////////////////////////////////////////
void AppearanceManager::finish(void)
{
    std::set<TexCoords*> useLessTexCoords;

    for ( std::map<std::string, TexCoords*>::iterator it = _texCoordsMap.begin(); it != _texCoordsMap.end(); ++it )
    {
        if ( it->second && useLessTexCoords.find( it->second ) == useLessTexCoords.end() )
        {
            useLessTexCoords.insert( it->second );
            delete it->second;
        }
    }

    for ( std::vector<TexCoords*>::iterator it = _obsoleteTexCoords.begin(); it != _obsoleteTexCoords.end(); it++ )
        if ( useLessTexCoords.find( *it ) == useLessTexCoords.end() )
            delete *it;

    _appearancesMap.clear();
    _texCoordsMap.clear();
    _obsoleteTexCoords.clear();
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
