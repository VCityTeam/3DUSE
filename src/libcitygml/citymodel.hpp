#ifndef __CITYGML_CITYMODEL_HPP__
#define __CITYGML_CITYMODEL_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
#include "envelope.hpp"
#include "cityobject.hpp"
#include "appearancemanager.hpp"
#include "vecs.hpp"
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
    CityModel( const std::string& id = "CityModel" ) : Object( id ) {}

    LIBCITYGML_EXPORT ~CityModel( void );

    // Return the envelope (ie. the bounding box) of the model
    inline const Envelope& getEnvelope( void ) const { return _envelope; }
    inline Envelope& getEnvelope( void ) { return _envelope; }

    // Return the translation parameters of the model
    inline const TVec3d& getTranslationParameters( void ) const { return _translation; }

    // Get the number of city objects
    inline unsigned int size( void ) const
    {
        unsigned int count = 0;
        CityObjectsMap::const_iterator it = _cityObjectsMap.begin();
        for ( ; it != _cityObjectsMap.end(); ++it ) count += it->second.size();
        return count;
    }

    inline const CityObjectsMap& getCityObjectsMap( void ) const { return _cityObjectsMap; }
    inline CityObjectsMap& getCityObjectsMap( void ) { return _cityObjectsMap; }

    inline const CityObjects* getCityObjectsByType( CityObjectsType type ) const
    {
        CityObjectsMap::const_iterator it = _cityObjectsMap.find( type );
        return ( it != _cityObjectsMap.end() ) ? &it->second : 0;
    }

    // Return the roots elements of the model. You can then navigate the hierarchy using object->getChildren().
    inline const CityObjects& getCityObjectsRoots( void ) const { return _roots; }
    inline CityObjects& getCityObjectsRoots( void ) { return _roots; }

    inline const std::string& getSRSName( void ) const { return _srsName; }

    void computeEnvelope();

    AppearanceManager* getAppearanceManager() { return &_appearanceManager; }

    std::string m_basePath;

    inline void addCityObjectAsRoot( CityObject* o ) { if ( o ) _roots.push_back( o ); }

    void addCityObject( CityObject* o );

    void finish( const ParserParams& );

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
