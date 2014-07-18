#ifndef __CITYGML_CITYOBJECT_HPP__
#define __CITYGML_CITYOBJECT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
#include "geometry.hpp"
#include "temporalExt.hpp"
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
enum CityObjectsType {
    COT_GenericCityObject           = 1 << 0,
    COT_Building                    = 1 << 1,
    COT_Room                        = 1 << 2,
    COT_BuildingInstallation        = 1 << 3,
    COT_BuildingFurniture           = 1 << 4,
    COT_Door                        = 1 << 5,
    COT_Window                      = 1 << 6,
    COT_CityFurniture               = 1 << 7,
    COT_Track                       = 1 << 8,
    COT_Road                        = 1 << 9,
    COT_Railway                     = 1 << 10,
    COT_Square                      = 1 << 11,
    COT_PlantCover                  = 1 << 12,
    COT_SolitaryVegetationObject    = 1 << 13,
    COT_WaterBody                   = 1 << 14,
    COT_TINRelief                   = 1 << 15,
    COT_LandUse                     = 1 << 16,
    COT_Tunnel						= 1 << 17,
    COT_Bridge						= 1 << 18,
    COT_BridgeConstructionElement	= 1 << 19,
    COT_BridgeInstallation			= 1 << 20,
    COT_BridgePart                  = 1 << 21,
    COT_BuildingPart                = 1 << 22,

    COT_WallSurface                 = 1 << 23,
    COT_RoofSurface                 = 1 << 24,
    COT_GroundSurface               = 1 << 25,
    COT_ClosureSurface              = 1 << 26,
    COT_FloorSurface                = 1 << 27,
    COT_InteriorWallSurface         = 1 << 28,
    COT_CeilingSurface              = 1 << 29,

    COT_All                         = 0xFFFFFFFF
};
typedef unsigned int CityObjectsTypeMask;
////////////////////////////////////////////////////////////////////////////////
class CityObject : public Object
{
    friend class CityGMLHandler;
    friend class CityModel;
    friend std::ostream& operator<<( std::ostream&, const CityObject & );
public:
    CityObject( const std::string& id, CityObjectsType type );

    virtual ~CityObject() override;

    // Get the object type
    CityObjectsType getType( void ) const;

    std::string getTypeAsString( void ) const;

    // Return the envelope (ie. the bounding box) of the object
    const Envelope& getEnvelope( void ) const;

    // Get the default diffuse color of this object class
    virtual TVec4f getDefaultColor( void ) const = 0;

    // Get the number of geometries contains in the object
    unsigned int size( void ) const;

    // Access the geometries
    const Geometry* getGeometry( unsigned int i ) const;

    std::vector< Geometry* >& getGeometries();
    const std::vector< Geometry* >& getGeometries() const;

    // Access the children
    unsigned int getChildCount( void ) const;

    const CityObject* getChild( unsigned int i ) const;

    CityObject* getChild( unsigned int i );

    const std::vector< CityObject* >& getChildren( void ) const;

    std::vector< CityObject* >& getChildren( void );

    void addGeometry(Geometry* geom);

    void computeEnvelope();

    void computeCentroid();

    CityObject* getParent();

    void deleteNode(const std::string& node);
    void deleteNode(CityObject* node);

    void insertNode(CityObject* node);

    void addState(CityObjectState* state);
    std::vector<CityObjectState*>& getStates();
    const std::vector<CityObjectState*>& getStates() const;
    CityObjectState* getState(const std::string& name);

    void addTag(CityObjectTag* tag);
    std::vector<CityObjectTag*>& getTags();
    const std::vector<CityObjectTag*>& getTags() const;

    void checkTags();

    bool isTemporal() const;

    //const std::string& getAttribute(const std::string& attribName, const QDateTime& date) const;

//protected:
    void finish( AppearanceManager&, const ParserParams& );

protected:
    CityObjectsType _type;
    CityObject* _parent;

    Envelope _envelope;
    TVec3d _centroid;

    std::vector< Geometry* > _geometries;
    std::vector< CityObject* > _children;

    std::vector<CityObjectState*> m_states;
    std::vector<CityObjectTag*> m_tags;

public:
    std::string m_path;
};
////////////////////////////////////////////////////////////////////////////////
LIBCITYGML_EXPORT std::string getCityObjectsClassName( CityObjectsTypeMask mask );
LIBCITYGML_EXPORT CityObjectsTypeMask getCityObjectsTypeMaskFromString( const std::string& stringMask );
std::ostream& operator<<( std::ostream&, const citygml::CityObject& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_CITYOBJECT_HPP__
