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

    std::string m_basePath;

    void addCityObjectAsRoot( CityObject* o );

    void addCityObject( CityObject* o );

	CityObject* getNode(const vcity::URI& uri);

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
