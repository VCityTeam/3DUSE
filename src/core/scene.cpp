////////////////////////////////////////////////////////////////////////////////
#include "scene.hpp"
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
Scene::Scene()
    : m_layers()
{
    addLayer(new Layer("layer_CityGML"));
}
////////////////////////////////////////////////////////////////////////////////
Scene::~Scene()
{
    reset();
}
////////////////////////////////////////////////////////////////////////////////
void Scene::addLayer(Layer* layer)
{
    // check that layer is not already in the scene
    //URI uri
    //getLayer();
    //layer->getName();
    m_layers.push_back(layer);
}
////////////////////////////////////////////////////////////////////////////////
void Scene::addLayer(const std::string& name)
{
    // check that layer is not already in the scene
    //URI uri
    //getLayer();
    //layer->getName();
    Layer* layer = new Layer(name);
    m_layers.push_back(layer);
}
////////////////////////////////////////////////////////////////////////////////
Layer* Scene::getLayer(const URI& uri)
{
    if(uri.getDepth() > 0)
    {
        for(std::vector<Layer*>::iterator it = m_layers.begin(); it < m_layers.end(); ++it)
        {
            if(uri.getNode(0) == (*it)->getName())
            {
                return *it;
            }
        }
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
const Layer* Scene::getLayer(const URI& uri) const
{
    if(uri.getDepth() > 0)
    {
        for(std::vector<Layer*>::const_iterator it = m_layers.begin(); it < m_layers.end(); ++it)
        {
            if(uri.getNode(0) == (*it)->getName())
            {
                return *it;
            }
        }
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
Layer* Scene::getDefaultLayer()
{
    if(m_layers.size() > 0)
    {
        return m_layers[0];
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<Layer*>& Scene::getLayers()
{
    return m_layers;
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<Layer*>& Scene::getLayers() const
{
    return m_layers;
}
////////////////////////////////////////////////////////////////////////////////
void Scene::deleteLayer(const URI& uri)
{
    if(uri.getType() == "Layer")
    {
        for(std::vector<Layer*>::iterator it = m_layers.begin(); it < m_layers.end(); ++it)
        {
            if(uri.getNode(0) == (*it)->getName())
            {
                m_layers.erase(it);
                delete *it;
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void Scene::addTile(const URI& uriLayer, Tile* tile)
{
    Layer* layer = getLayer(uriLayer);
    if(layer)
    {
        layer->addTile(tile);
    }
}
////////////////////////////////////////////////////////////////////////////////
Tile* Scene::getTile(const URI& uri)
{
    Layer* layer = getLayer(uri);
    if(layer)
    {
        return layer->getTile(uri);
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<Tile*>& Scene::getTiles(const URI& uriLayer)
{
    Layer* layer = getLayer(uriLayer);
    if(layer)
    {
        return layer->getTiles();
    }

    //return 0; // fail
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<Tile*>& Scene::getTiles(const URI& uriLayer) const
{
    const Layer* layer = getLayer(uriLayer);
    if(layer)
    {
        return layer->getTiles();
    }

    //return 0; // fail
}
////////////////////////////////////////////////////////////////////////////////
void Scene::deleteTile(const URI& uri)
{
    Layer* layer = getLayer(uri);
    if(layer)
    {
        return layer->deleteTile(uri);
    }
}
////////////////////////////////////////////////////////////////////////////////
citygml::CityObject* Scene::getNode(const URI& uri)
{
    if(uri.getDepth() > 2)
    {
        //URI uriLayer;
        //uriLayer.append(uri.getNode(0));
        //log() << "debug uri : " << uri.getNode(0) << "\n";
        //uriLayer.setType("Layer");
        Layer* layer = getLayer(uri);
        if(layer)
        {
            return layer->getNode(uri);
        }
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
void Scene::reset()
{
    // clear layers
    std::vector<Layer*>::iterator it;
    for(it=m_layers.begin(); it<m_layers.end(); ++it)
    {
        // this will this the layer, including the tiles inside
        delete *it;
    }
    m_layers.clear();
}
////////////////////////////////////////////////////////////////////////////////
void Scene::dump()
{
    log() << "root" << "\n";
    for(std::vector<Layer*>::iterator it=m_layers.begin(); it<m_layers.end(); ++it)
    {
        log() << "  " << (*it)->getName() << "\n";
        (*it)->dump();
    }
}
////////////////////////////////////////////////////////////////////////////////
/*citygml::CityObject* Scene::findNode(const std::string& name)
{
    citygml::CityObject* res = NULL;
    std::vector<Tile*>::iterator it;
    for(it=m_tiles.begin(); it<m_tiles.end(); ++it)
    {
        res = (*it)->findNode(name);
        if(res) break;
    }

    return res;
}*/
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
