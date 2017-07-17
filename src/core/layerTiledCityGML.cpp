// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "layerTiledCityGML.hpp"
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    ////////////////////////////////////////////////////////////////////////////////
    LayerTiledCityGML::LayerTiledCityGML(const std::string& name)
        : abstractLayer(name)
    {

    }
    ////////////////////////////////////////////////////////////////////////////////
    LayerTiledCityGML::~LayerTiledCityGML()
    {

    }
    ////////////////////////////////////////////////////////////////////////////////
    void LayerTiledCityGML::addTile(Tile* tile)
    {
        m_tiles.push_back(tile);
    }
    ////////////////////////////////////////////////////////////////////////////////
    Tile* LayerTiledCityGML::getTile(const URI& uri)
    {
        if (uri.getDepth() > 1)
        {
            for (std::vector<Tile*>::iterator it = m_tiles.begin(); it < m_tiles.end(); ++it)
            {
                if (uri.getNode(1) == (*it)->getName())
                {
                    return *it;
                }
            }
        }

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<Tile*>& LayerTiledCityGML::getTiles()
    {
        return m_tiles;
    }
    ////////////////////////////////////////////////////////////////////////////////
    const std::vector<Tile*>& LayerTiledCityGML::getTiles() const
    {
        return m_tiles;
    }
    ////////////////////////////////////////////////////////////////////////////////
    void LayerTiledCityGML::deleteTile(const URI& uri)
    {
        Tile* tile = getTile(uri);

        for (std::vector<Tile*>::iterator it = m_tiles.begin(); it < m_tiles.end(); ++it)
        {
            if ((*it)->getName() == tile->getName())
            {
                log() << "Tile " << tile->getName() << " removed from layer " << uri.getNode(0) << "\n";
                m_tiles.erase(it);
                delete tile;
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    citygml::CityObject* LayerTiledCityGML::getCityObjectNode(const URI& uri, bool inPickingMode)
    {
        Tile* tile = getTile(uri);
        if (tile)
        {
            return tile->getNode(uri, inPickingMode);
        }
        else std::cout << "tile is NULL in LayerTiledCityGML::getCityObjectNode" << std::endl;

        return nullptr;
    }
    ////////////////////////////////////////////////////////////////////////////////
    const std::string LayerTiledCityGML::getType() const
    {
        return "LayerCityGML";
    }
    ////////////////////////////////////////////////////////////////////////////////
    URI LayerTiledCityGML::getURI() const
    {
        URI uri;
        uri.append(getName(), getType());
        uri.setType(getType());

        return uri;
    }
    ////////////////////////////////////////////////////////////////////////////////
    void LayerTiledCityGML::dump()
    {
        for (std::vector<Tile*>::iterator it = m_tiles.begin(); it < m_tiles.end(); ++it)
        {
            log() << "    " << (*it)->getName() << "\n";
            //(*it)->dump();
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
