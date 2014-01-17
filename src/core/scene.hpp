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
    Scene();
    ~Scene();

    void addLayer(Layer* layer);
    void deleteLayer(const URI& uri);
    std::vector<Layer*>& getLayers();

    void addTile(Tile* tile);
    std::vector<Tile*>& getTiles();

    void deleteNode(const std::string URI);

    void reset();

    void dump();

    citygml::CityObject* findNode(const std::string& name);

private:
    std::vector<Tile*> m_tiles;
    std::vector<Layer*> m_layers;   ///< all the layers
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __SCENE_HPP__
