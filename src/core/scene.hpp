#ifndef __SCENE_HPP__
#define __SCENE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "layer.hpp"
#include "tile.hpp"
#include "URI.hpp"
#include <memory>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief Scene class : it holds all the scene data
/// It is a set of layers which finally contains the tiles
class Scene
{
public:
    /// \brief Scene Build an empty scene
    Scene();
    ~Scene();

    /// \brief addLayer Add a layer
    /// \param layer Layer to add
    void addLayer(Layer* layer);

    /// \brief getLayer Get a layer
    /// \param uri URI pointing to the layer
    /// \return The layer
    Layer* getLayer(const URI& uri);

    /// \brief getLayer Get a layer (const)
    /// \param uri URI pointing to the layer
    /// \return The const layer
    const Layer* getLayer(const URI& uri) const;

    /// \brief getDefaultLayer Get default layer
    /// \return The default layer
    Layer* getDefaultLayer();

    /// \brief getLayers Get all layer
    /// \return A vector of Layer
    std::vector<Layer*>& getLayers();

    /// \brief getLayers Get all layers (const)
    /// \return A const vector of Layer
    const std::vector<Layer*>& getLayers() const;

    /// \brief deleteLayer Delete a layer
    /// \param uri URI pointing to the layer
    void deleteLayer(const URI& uri);


    /// \brief addTile Add a tile in a layer
    /// \param uriLayer URI pointing to the layer
    /// \param tile The tile to add
    void addTile(const URI& uriLayer, Tile* tile);

    /// \brief getTile Get a tile
    /// \param uri URI pointing to the tile
    /// \return The tile
    Tile* getTile(const URI& uri);

    /// \brief getTiles Get all tiles in a layer
    /// \param uriLayer URI pointing to the layer
    /// \return A vector of Tile
    std::vector<Tile*>& getTiles(const URI& uriLayer);

    /// \brief getTiles Get all tiles in a layer (const)
    /// \param uriLayer URI pointing to the layer
    /// \return A const vector of Tile
    const std::vector<Tile*>& getTiles(const URI& uriLayer) const;

    /// \brief deleteTile Delete a tile in a layer
    /// \param uri URI pointing to the tile
    void deleteTile(const URI& uri);

    /// \brief getNode Get a CityGML node
    /// \param uri URI pointing to the CityGML node
    /// \return Ptr to CityGML node or nullptr
    citygml::CityObject* getNode(const URI& uri);

    /// \brief deleteNode Delete a CityGML node
    /// \param uri URI pointing to the CityGML node
    void deleteNode(const URI& uri);

    void reset();

    void dump();

    //citygml::CityObject* findNode(const std::string& name);

    //citygml::CityObject* getNode(const URI& uri);

private:
    //std::vector<Tile*> m_tiles;
    std::vector<Layer*> m_layers;   ///< all the layers
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __SCENE_HPP__
