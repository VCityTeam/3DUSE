////////////////////////////////////////////////////////////////////////////////
#include "layerCityGML.hpp"
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
LayerCityGML::LayerCityGML(const std::string& name)
    : abstractLayer(name)
{

}
////////////////////////////////////////////////////////////////////////////////
LayerCityGML::~LayerCityGML()
{
    for(Tile* tile : m_tiles)
    {
        delete tile;
    }
}
////////////////////////////////////////////////////////////////////////////////
void LayerCityGML::addTile(Tile* tile)
{
    m_tiles.push_back(tile);
}
////////////////////////////////////////////////////////////////////////////////
Tile* LayerCityGML::getTile(const URI& uri)
{
    if(uri.getDepth() > 1)
    {
        for(std::vector<Tile*>::iterator it = m_tiles.begin(); it < m_tiles.end(); ++it)
        {
            if(uri.getCurrentNode() == (*it)->getName())
            {
				uri.popFront();
                return *it;
            }
        }
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
std::vector<Tile*>& LayerCityGML::getTiles()
{
    return m_tiles;
}
////////////////////////////////////////////////////////////////////////////////
const std::vector<Tile*>& LayerCityGML::getTiles() const
{
    return m_tiles;
}
////////////////////////////////////////////////////////////////////////////////
void LayerCityGML::deleteTile(const URI& uri)
{
    Tile* tile = getTile(uri);

    for(std::vector<Tile*>::iterator it=m_tiles.begin(); it<m_tiles.end(); ++it)
    {
        if((*it)->getName() == tile->getName())
        {
            log() << "Tile " << tile->getName() << " removed from layer " << uri.getNode(0) << "\n";
            m_tiles.erase(it);
            delete tile;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
citygml::CityObject* LayerCityGML::getCityObjectNode(const URI& uri)
{
    Tile* tile = getTile(uri);
    if(tile)
    {
        return tile->getNode(uri);
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
const std::string LayerCityGML::getType() const
{
    return "LayerCityGML";
}
////////////////////////////////////////////////////////////////////////////////
URI LayerCityGML::getURI() const
{
    URI uri;
    uri.append(getName(), getType());
    uri.setType(getType());

    return uri;
}
////////////////////////////////////////////////////////////////////////////////
void LayerCityGML::dump()
{
    for(std::vector<Tile*>::iterator it=m_tiles.begin(); it<m_tiles.end(); ++it)
    {
        log() << "    " << (*it)->getName() << "\n";
        //(*it)->dump();
    }
}
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
