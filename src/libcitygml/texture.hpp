#ifndef __CITYGML_TEXTURE_HPP__
#define __CITYGML_TEXTURE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "appearance.hpp"
#include "vecs.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Texture : virtual public Appearance
{
    friend class CityGMLHandler;

public:
    typedef enum WrapMode
    {
        WM_NONE = 0,	// the resulting color is fully transparent
        WM_WRAP,		// the texture is repeated
        WM_MIRROR,		// the texture is repeated and mirrored
        WM_CLAMP,		// the texture is clamped to its edges
        WM_BORDER		// the resulting color is specified by the borderColor element (RGBA)
    } WrapMode;

    Texture( const std::string& id );

    std::string getUrl( void ) const;

    void setUrl(const std::string& url);

    bool getRepeat( void ) const;

    WrapMode getWrapMode( void ) const;

    TVec4f getBorderColor( void ) const;

    std::string toString( void ) const;

protected:
    std::string _url;
    bool _repeat;
    WrapMode _wrapMode;
    TVec4f _borderColor;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_TEXTURE_HPP__
