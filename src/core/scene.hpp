// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __SCENE_HPP__
#define __SCENE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "abstractlayer.hpp"
#include "layerCityGML.hpp"
#include "layerAssimp.hpp"
#include "layerMnt.hpp"
#include "layerLas.hpp"
#include "layerShp.hpp"
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

    /// \brief addLayer Add a abstractlayer
    /// \param abstractlayer abstractLayer to add
    void addLayer(abstractLayer* abstractlayer);
    //void addLayer(const std::string& name);

    /// \brief getLayer Get a layer
    /// \param uri URI pointing to the layer
    /// \return The layer
    abstractLayer* getLayer(const URI& uri);

    /// \brief getLayer Get a layer (const)
    /// \param uri URI pointing to the layer
    /// \return The const layer
    const abstractLayer* getLayer(const URI& uri) const;

    /// \brief getDefaultLayer Get default layer
    /// \param type layer type
    /// \return The default layer
    abstractLayer* getDefaultLayer(const std::string& type);

    /// \brief getLayers Get all layer
    /// \return A vector of Layer
    std::vector<abstractLayer*>& getLayers();

    /// \brief getLayers Get all layers (const)
    /// \return A const vector of Layer
    const std::vector<abstractLayer*>& getLayers() const;

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
    std::vector<Tile*>* getTiles(const URI& uriLayer);

    /// \brief getTiles Get all tiles in a layer (const)
    /// \param uriLayer URI pointing to the layer
    /// \return A const vector of Tile
    const std::vector<Tile*>* getTiles(const URI& uriLayer) const;

    /// \brief deleteTile Delete a tile in a layer
    /// \param uri URI pointing to the tile
    void deleteTile(const URI& uri);

    /// \brief getCityObjectNode Get a CityGML node
    /// \param uri URI pointing to the CityGML node
    /// \return Ptr to CityGML node or nullptr
    citygml::CityObject* getCityObjectNode(const URI& uri);

    /// \brief deleteNode Delete a CityGML node
    /// \param uri URI pointing to the CityGML node
    void deleteNode(const URI& uri);

    /// \brief reset Reset the scene
    void reset();

    void dump();

    //citygml::CityObject* findNode(const std::string& name);

    //citygml::CityObject* getNode(const URI& uri);

private:
    std::vector<abstractLayer*> m_layers;   ///< all the layers
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __SCENE_HPP__
