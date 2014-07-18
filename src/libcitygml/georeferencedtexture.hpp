#ifndef __CITYGML_GEOREFERENCEDTEXTURE_HPP__
#define __CITYGML_GEOREFERENCEDTEXTURE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "texture.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class GeoreferencedTexture : public Texture
{
    friend class CityGMLHandler;

public:
    GeoreferencedTexture( const std::string& id );

    bool getPreferWorldFile( void ) const;

    // TODO support referencePoint and orientation
    class WorldParams
    {
    public:
        WorldParams()
            : xPixelSize(0.0), yRotation(0.0), xRotation(0.0), yPixelSize(0.0), xOrigin(0.0), yOrigin(0.0)
        {

        }

        double xPixelSize;
        double yRotation;
        double xRotation;
        double yPixelSize;
        double xOrigin;
        double yOrigin;
    };

    bool m_initWParams;
    WorldParams m_wParams;

protected:
    bool _preferWorldFile;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& os, const GeoreferencedTexture::WorldParams& wp);
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_GEOREFERENCEDTEXTURE_HPP__
