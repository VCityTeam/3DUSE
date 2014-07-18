////////////////////////////////////////////////////////////////////////////////
#include "linearring.hpp"
#include <limits>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
LinearRing::LinearRing( const std::string& id, bool isExterior )
    : Object( id ), _exterior( isExterior )
{}
////////////////////////////////////////////////////////////////////////////////
bool LinearRing::isExterior() const
{
    return _exterior;
}
////////////////////////////////////////////////////////////////////////////////
unsigned int LinearRing::size() const
{
    return _vertices.size();
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<TVec3d>& LinearRing::getVertices() const
{
    return _vertices;
}
////////////////////////////////////////////////////////////////////////////////
void LinearRing::addVertex( const TVec3d& v )
{
    _vertices.push_back( v );
}
////////////////////////////////////////////////////////////////////////////////
// Return the envelope (ie. the bounding box) of the object
const Envelope& LinearRing::getEnvelope() const
{
    return _envelope;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<TVec3d>& LinearRing::getVertices()
{
    return _vertices;
}
////////////////////////////////////////////////////////////////////////////////
TVec3d LinearRing::computeNormal( void ) const
{
    unsigned int len = size();
    if ( len < 3 ) return TVec3d();

    // Tampieri, F. 1992. Newell's method for computing the plane equation of a polygon. In Graphics Gems III, pp.231-232.
    TVec3d n( 0., 0., 0. );
    for ( unsigned int i = 0; i < len; i++ )
    {
        const TVec3d& current = _vertices[i];
        const TVec3d& next = _vertices[ ( i + 1 ) % len];

        n.x += ( current.y - next.y ) * ( current.z + next.z );
        n.y += ( current.z - next.z ) * ( current.x + next.x );
        n.z += ( current.x - next.x ) * ( current.y + next.y );
    }
    return n.normal();
}
////////////////////////////////////////////////////////////////////////////////
void LinearRing::finish( TexCoords* texCoords )
{
    // Remove duplicated vertex
    unsigned int len = _vertices.size();
    if ( len < 2 ) return;

    for ( unsigned int i = 0; i < len; i++ )
    {
        if ( ( _vertices[i] - _vertices[ ( i + 1 ) % len ] ).sqrLength() <= std::numeric_limits<float>::epsilon() )
        {
            _vertices.erase( _vertices.begin() + i );
            if ( texCoords && texCoords->size() > i) texCoords->erase( texCoords->begin() + i );
            finish( texCoords );
            return;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
