#ifndef __CITYGML_GEOMETRY_HPP__
#define __CITYGML_GEOMETRY_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
#include "envelope.hpp"
#include "polygon.hpp"
#include "parserparams.hpp"
#include "citygmlcommon.hpp"
#include <vector>
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Polygon;
class CityObject;
enum GeometryType
{
    GT_Unknown = 0,
    GT_Roof,
    GT_Wall,
    GT_Ground,
    GT_Closure,
    GT_Floor,
    GT_InteriorWall,
    GT_Ceiling,
};
////////////////////////////////////////////////////////////////////////////////
/// \brief Geometry class
///
////////////////////////////////////////////////////////////////////////////////
class Geometry : public Object
{
    friend class CityGMLHandler;
    friend class CityObject;
    friend std::ostream& operator<<( std::ostream&, const citygml::Geometry& );
public:
    Geometry( const std::string& id, GeometryType type = GT_Unknown, unsigned int lod = 0 );

    LIBCITYGML_EXPORT virtual ~Geometry() override;

    // Get the geometry LOD
    unsigned int getLOD( void ) const;

    // Return the envelope (ie. the bounding box) of the object
    const Envelope& getEnvelope( void ) const;

    // Get the polygons
    unsigned int size( void ) const;
    Polygon* operator[]( unsigned int i );
    const Polygon* operator[]( unsigned int i ) const;

    const std::vector< Polygon* >& getPolygons() const;
    std::vector< Polygon* >& getPolygons();

    GeometryType getType( void ) const;

    CityObject* getParent();

    void addPolygon( Polygon* );

    void finish( AppearanceManager&, Appearance*, const ParserParams& );

    bool merge( Geometry* );

protected:
    GeometryType _type;
    CityObject* _parent;

    unsigned int _lod;

    Envelope _envelope;

    std::vector< Polygon* > _polygons;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream&, const citygml::Geometry& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_GEOMETRY_HPP__
