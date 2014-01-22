#ifndef __OSGSCENE_HPP__
#define __OSGSCENE_HPP__
////////////////////////////////////////////////////////////////////////////////
/// Tools for osg tree
///
/// osg scene design :
///
/// root - OsgScene instance           osg::Group - root
///                                            |
/// effect                             osg::Group - effect
///                                            |
/// layers                      -------osg::Group - layers-------
///                             |                               |
/// layer child         osg::Switch - layer0        osg::Switch - layer1
///                                     |
/// LODs                   ------------osg::Group - LODs
///                        |                   |
/// LOD child     osg::Switch - LOD0   osg::Switch - LOD1    ...
///                        |                   |
/// Tiles         osg::Group - Tiles   osg::Group - Tiles   ...
///                     |           |
/// Tile child  osg::Group - Tile0  osg::Group - Tile1 ...
///
////////////////////////////////////////////////////////////////////////////////
#include "core/tile.hpp"
#include "core/URI.hpp"
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
////////////////////////////////////////////////////////////////////////////////
class OsgScene : public osg::Group
{
public:
    OsgScene();

    void init();

    /// \brief addTile Add a tile in a layer of the osg scene
    /// \param uriLayer URI pointing to the layer
    /// \param tile Tile to add
    void addTile(const vcity::URI& uriLayer, const vcity::Tile& tile);

    /// \brief setTileName Set the name of a tile
    /// \param uriTile URI pointing to the tile
    /// \param name Tile name
    void setTileName(const vcity::URI& uriTile, const std::string& name);

    /// \brief deleteTile Delete a layer in the osg scene
    /// \param uriTile URI pointing to the tile
    void deleteTile(const vcity::URI& uriTile);

    /// \brief addLayer Add a layer to the osg scene
    /// \param name Layer name
    void addLayer(const std::string& name);

    /// \brief setLayerName Set the name of a layer in the osg scene
    /// \param uriLayer URI pointing to the layer
    /// \param name Layer name
    void setLayerName(const vcity::URI& uriLayer, const std::string& name);

    /// \brief deleteLayer Delete a layer in the osg scene
    /// \param uriLayer URI pointing to the layer
    void deleteLayer(const vcity::URI& uriLayer);

    int getNbTiles() const;

    osg::ref_ptr<osg::Group> getTile(int id);

    osg::ref_ptr<osg::Group> getTile(const std::string& name);

    osg::ref_ptr<osg::Node> findNode(const std::string& name);

    osg::ref_ptr<osg::Node> findNode(const vcity::URI& uri);

    void deleteNode(const std::string& name);

    void setShadow(bool shadow);

    void setYear(int year);

    void reset();

    void dump(std::ostream& out = std::cout, osg::ref_ptr<osg::Node> node = NULL, int depth=0);

    osg::ref_ptr<osg::Geode> buildGrid(osg::Vec3 origin, float step, int n);
    //osg::ref_ptr<osg::Geode> buildBBox(osg::Vec3 lowerBound, osg::Vec3 upperBound);

    osg::ref_ptr<osg::Node> getNode(const vcity::URI& uri);

public:
    osg::ref_ptr<osg::Node> buildTile(const vcity::Tile& tile);
    void buildTileRec(osg::ref_ptr<osg::Group> nodeOsg, citygml::CityObject* node, int depth=0);

    bool m_shadow;
    osg::Vec4 m_shadowVec;

    osg::ref_ptr<osg::Group> m_layers;      ///< Group holding all layers, and below, all tiles, ...

    osg::ref_ptr<osg::Group> m_effectNone;
    osg::ref_ptr<osg::Group> m_effectShadow;
};
////////////////////////////////////////////////////////////////////////////////
/// \brief osgSceneBuild will create an osg tree (for rendering with osg) from a tile
/// \param tile : input tile
/// \return The osg scene tree
//osg::ref_ptr<osg::Node> osgSceneBuild(const Tile& tile);
////////////////////////////////////////////////////////////////////////////////
/// \brief osgSceneDeleteNode will delete the node name in osg tree root
/// \param root : osg tree
/// \param name : node to delete
//void osgSceneDeleteNode(osg::ref_ptr<osg::Group> root, const std::string& name);
////////////////////////////////////////////////////////////////////////////////
/// \brief osgSceneSetShadow
/// \param root : osg tree
/// \param shadow : bool to set or unset shadow effect
//void osgSceneSetShadow(osg::ref_ptr<osg::Group> root, bool shadow);
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGSCENE_HPP__
