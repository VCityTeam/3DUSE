#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
#include "URI.hpp"
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
class Controller
{
public:
    Controller();

    // reset app
    virtual void reset();

    // generic nodes actions
    virtual void addNode(const URI& URI);
    virtual void deleteNode(const URI& URI);

    // layer
    virtual void addLayer(const std::string& name);
    virtual void deleteLayer(const vcity::URI& uri);
    virtual void setLayerName(const vcity::URI& uri, const std::string& name);

    // tile
    virtual void addTile(const vcity::URI& uriLayer, vcity::Tile& tile);
    virtual void deleteTile(const vcity::URI& uri);
    virtual void setTileName(const vcity::URI& uri, const std::string& name);
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLER_HPP__
