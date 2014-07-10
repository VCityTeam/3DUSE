#ifndef __CITYGML_LINEARRING_HPP__
#define __CITYGML_LINEARRING_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
#include "vecs.hpp"
#include "envelope.hpp"
#include "citygmlcommon.hpp"
#include "citygmltypes.hpp"
#include <vector>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class LinearRing : public Object
{
    friend class CityGMLHandler;
    friend class Polygon;
public:
    LinearRing( const std::string& id, bool isExterior );

    bool isExterior( void ) const;

    unsigned int size( void ) const;

    const std::vector<TVec3d>& getVertices( void ) const;

    void addVertex( const TVec3d& v );

    LIBCITYGML_EXPORT TVec3d computeNormal( void ) const;

    // Return the envelope (ie. the bounding box) of the object
    const Envelope& getEnvelope( void ) const;

    std::vector<TVec3d>& getVertices( void );

    void finish( TexCoords* );

protected:
    bool _exterior;

    std::vector<TVec3d> _vertices;

    Envelope _envelope;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif //  __CITYGML_LINEARRING_HPP__
