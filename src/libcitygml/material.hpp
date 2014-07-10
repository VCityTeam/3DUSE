#ifndef __CITYGML_MATERIAL_HPP__
#define __CITYGML_MATERIAL_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "appearance.hpp"
#include "vecs.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Material : virtual public Appearance
{
    friend class CityGMLHandler;
public:
    Material( const std::string& id );

    TVec3f getDiffuse( void ) const;
    TVec3f getEmissive( void ) const;
    TVec3f getSpecular( void ) const;
    float getAmbientIntensity( void ) const;
    float getShininess( void ) const;
    float getTransparency( void ) const;

protected:
    TVec3f _diffuse;
    TVec3f _emissive;
    TVec3f _specular;
    float _ambientIntensity;
    float _shininess;
    float _transparency;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_MATERIAL_HPP__
