#ifndef __TILE_HPP__
#define __TILE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "citygml.hpp"
#include "URI.hpp"
//#include <osg/Node>
//#include <vector>
#include <memory>
#include <map>
///////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
///////////////////////////////////////////////////////////////////////////////////
/// \brief The Tile class
/// Holds citygml, shp data
class Tile
{
public:
    Tile();
    Tile(const std::string& filepath);
    //Tile(const TVec3d& pMin, const TVec3d& pMax);

    citygml::Envelope& getEnvelope();
    const citygml::Envelope& getEnvelope() const;

    void computeEnvelope();

    void load(const std::string& filepath);
    //osg::ref_ptr<osg::Node> buildOsgData();

    citygml::CityModel* getCityModel();
    const citygml::CityModel* getCityModel() const;

    const std::string& getName() const;
    //osg::ref_ptr<osg::Node> getOsgRoot();

    //citygml::CityObject* findNode(const std::string& name);

    void deleteNode(const std::string& name);
    void insertNode(citygml::CityObject* node);
    void replaceNode(const std::string& name, citygml::CityObject* node);

    /// \brief getNode Get a CityGML node
    /// \param uri URI pointing to the CityGML node
    /// \return Ptr to CityGML node or nullptr
    citygml::CityObject* getNode(const URI& uri);

private:
    //citygml::Envelope m_envelope;
    //std::map<std::string, citygml::CityObject*> m_cityObjects;
    std::map<std::string, citygml::CityModel*> m_citygml;   ///< citygml data
    std::map<std::string, int> m_shape;    ///< shape data
    std::string m_name; ///< tile name

    citygml::CityModel* m_root; // TODO : remove

    //osg::ref_ptr<osg::Node> m_rootOsg;
};
////////////////////////////////////////////////////////////////////////////////
typedef std::shared_ptr<Tile> TilePtr;
////////////////////////////////////////////////////////////////////////////////
} // namespace vity
////////////////////////////////////////////////////////////////////////////////
#endif // __TILE_HPP__
