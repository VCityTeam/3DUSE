// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
        reset();
    }
    ////////////////////////////////////////////////////////////////////////////////
    Scene::~Scene()
    {
        reset();
    }
    ////////////////////////////////////////////////////////////////////////////////
    void Scene::addLayer(abstractLayer* abstractlayer)
    {
        // check that layer is not already in the scene
        //URI uri
        //getLayer();
        //layer->getName();
        m_layers.push_back(abstractlayer);
    }
    ////////////////////////////////////////////////////////////////////////////////
    /*void Scene::addLayer(const std::string& name)
    {
        // check that layer is not already in the scene
        //URI uri
        //getLayer();
        //layer->getName();
        Layer* layer = new Layer(name);
        m_layers.push_back(layer);
    }*/
    ////////////////////////////////////////////////////////////////////////////////
    abstractLayer* Scene::getLayer(const URI& uri)
    {
        if (uri.getDepth() > 0)
        {
            for (std::vector<abstractLayer*>::iterator it = m_layers.begin(); it < m_layers.end(); ++it)
            {
                if (uri.getCurrentNode() == (*it)->getName())
                {
                    uri.popFront();
                    return *it;
                }
            }
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    const abstractLayer* Scene::getLayer(const URI& uri) const
    {
        if (uri.getDepth() > 0)
        {
            for (std::vector<abstractLayer*>::const_iterator it = m_layers.begin(); it < m_layers.end(); ++it)
            {
                if (uri.getCurrentNode() == (*it)->getName())
                {
                    uri.popFront();
                    return *it;
                }
            }
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    abstractLayer* Scene::getDefaultLayer(const std::string& type)
    {
        if (m_layers.size() > 0)
        {
            for (std::vector<abstractLayer*>::const_iterator it = m_layers.begin(); it < m_layers.end(); ++it)
            {
                if (type == (*it)->getType())
                {
                    return *it;
                }
            }
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<abstractLayer*>& Scene::getLayers()
    {
        return m_layers;
    }
    ////////////////////////////////////////////////////////////////////////////////
    const std::vector<abstractLayer*>& Scene::getLayers() const
    {
        return m_layers;
    }
    ////////////////////////////////////////////////////////////////////////////////
    void Scene::deleteLayer(const URI& uri)
    {
        for (std::vector<abstractLayer*>::iterator it = m_layers.begin(); it < m_layers.end(); ++it)
        {
            if (uri.getCurrentNode() == (*it)->getName())
            {
                uri.popFront();
                //m_layers.erase(it); // MT : remove
                delete *it;
                m_layers.erase(it); // MT : add
                break; // MT : add
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    void Scene::addTile(const URI& uriLayer, Tile* tile)
    {
        abstractLayer* abstractlayer = getLayer(uriLayer);
        if (abstractlayer)
        {
            LayerCityGML* layer = dynamic_cast<LayerCityGML*>(abstractlayer);
            if (layer)
                layer->addTile(tile);
            else std::cout << "layer is NULL in addTile" << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    void Scene::addInfo(const URI& uriLayer, std::vector<osgInfo*> info)
    {
        abstractLayer* abstractlayer = getLayer(uriLayer);
        if(abstractlayer)
        {
            LayerInfo* layer = dynamic_cast<LayerInfo*>(abstractlayer);
            if (layer)
                layer->setInfo(info);
            else std::cout << "layer is NULL in Scene::addInfo" << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    Tile* Scene::getTile(const URI& uri)
    {
        abstractLayer* abstractlayer = getLayer(uri);
        if (abstractlayer)
        {
            LayerCityGML* layer = dynamic_cast<LayerCityGML*>(abstractlayer);
            if (layer)
                return layer->getTile(uri);
            else std::cout << "layer is NULL in getTile" << std::endl;
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<Tile*>* Scene::getTiles(const URI& uriLayer)
    {
        abstractLayer* abstractlayer = getLayer(uriLayer);
        if (abstractlayer)
        {
            LayerCityGML* layer = dynamic_cast<LayerCityGML*>(abstractlayer);
            if (layer)
                return &layer->getTiles();
            else std::cout << "layer is NULL in getTiles" << std::endl;
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    const std::vector<Tile*>* Scene::getTiles(const URI& uriLayer) const
    {
        const abstractLayer* abstractlayer = getLayer(uriLayer);
        if (abstractlayer)
        {
            const LayerCityGML* layer = dynamic_cast<const LayerCityGML*>(abstractlayer);
            if (layer)
                return &layer->getTiles();
            else std::cout << "layer is NULL in const getTiles" << std::endl;
        }

        return nullptr;
    }

    ////////////////////////////////////////////////////////////////////////////////
    const std::vector<osgInfo*>* Scene::getInfo(const URI& uri)
    {
        abstractLayer* abstractlayer = getLayer(uri);
        if(abstractlayer)
        {
            LayerInfo* layer = dynamic_cast<LayerInfo*>(abstractlayer);
            if (layer)
                return &layer->getInfo();
            else std::cout << "layer is NULL in Scene::getInfo" << std::endl;
        }
        return nullptr;
    }

    ////////////////////////////////////////////////////////////////////////////////
    void Scene::deleteTile(const URI& uri)
    {
        abstractLayer* abstractlayer = getLayer(uri);
        if (abstractlayer)
        {
            LayerCityGML* layer = dynamic_cast<LayerCityGML*>(abstractlayer);
            if (layer)
                return layer->deleteTile(uri);
            else std::cout << "layer is NULL in deleteTile" << std::endl;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    citygml::CityObject* Scene::getCityObjectNode(const URI& uri, bool inPickingMode)
    {
        if (uri.getDepth() > 2)
        {
            //URI uriLayer;
            //uriLayer.append(uri.getNode(0));
            //log() << "debug uri : " << uri.getNode(0) << "\n";
            abstractLayer* abstractlayer = getLayer(uri);
            if (abstractlayer)
            {
                LayerCityGML* layer = dynamic_cast<LayerCityGML*>(abstractlayer);
                if (layer)
                    return layer->getCityObjectNode(uri, inPickingMode);
                else std::cout << "layer is NULL in getCityObjectNode" << std::endl;
            }
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    void Scene::reset()
    {
        // clear layers
        for (abstractLayer* layer : m_layers)
        {
            // this will this the layer, including the tiles inside
            delete layer;
        }
        m_layers.clear();

        addLayer(new LayerCityGML("layer_CityGML"));
        addLayer(new LayerAssimp("layer_Assimp"));
        addLayer(new LayerMnt("layer_Mnt"));
        addLayer(new LayerLas("layer_Las"));
        addLayer(new LayerShp("layer_Shp"));
	addLayer(new LayerInfo("layer_Info"));
    }
    ////////////////////////////////////////////////////////////////////////////////
    void Scene::dump()
    {
        log() << "root" << "\n";
        for (std::vector<abstractLayer*>::iterator it = m_layers.begin(); it < m_layers.end(); ++it)
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
