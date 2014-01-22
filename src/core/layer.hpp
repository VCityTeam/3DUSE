#ifndef __LAYER_HPP__
#define __LAYER_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "tile.hpp"
#include "URI.hpp"
#include <string>
#include <memory>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief Layer class : it holds all the tiles
/// A layer contains a set of tiles. Tiles are identified by a name and a position on a grid.
/// Grid info is in the data profile
class Layer
{
public:
    /// \brief Layer Build empty layer
    /// \param name Layer name
    Layer(const std::string& name);

    /// \brief setName Set layer name
    /// \param name Layer name
    void setName(const std::string& name);

    /// \brief getName Get layer name
    /// \return Layer name string
    const std::string& getName() const;

    /// \brief addTile Add a tile in a layer
    /// \param tile The tile to add
    void addTile(Tile* tile);

    /// \brief getTile Get a tile
    /// \param uri URI pointing to the tile
    /// \return The tile
    Tile* getTile(const URI& uri);

    /// \brief getTiles Get all tiles in the layer
    /// \return A vector of Tile
    std::vector<Tile*>& getTiles();

    /// \brief getTiles Get all tiles in the layer (const)
    /// \return A const vector of Tile
    const std::vector<Tile*>& getTiles() const;

    /// \brief deleteTile Delete a tile in a layer
    /// \param uri URI pointing to the tile
    void deleteTile(const URI& uri);

    /// \brief getNode Get a CityGML node
    /// \param uri URI pointing to the CityGML node
    /// \return Ptr to CityGML node or nullptr
    citygml::CityObject* getNode(const URI& uri);

    void dump();

private:
    std::string m_name;             ///< layer name
    std::vector<Tile*> m_tiles;     ///< Tiles
};
////////////////////////////////////////////////////////////////////////////////
typedef std::shared_ptr<Layer> LayerPtr;
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __LAYER_HPP__
