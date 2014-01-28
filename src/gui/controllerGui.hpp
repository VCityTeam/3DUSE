#ifndef __CONTROLLERGUI_HPP__
#define __CONTROLLERGUI_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "core/controller.hpp"
////////////////////////////////////////////////////////////////////////////////
class ControllerGui : public vcity::Controller
{
public:
    ControllerGui();

    virtual void reset();

    virtual void addNode(const vcity::URI& uri);
    virtual void deleteNode(const vcity::URI& uri);

    // layer
    virtual void addLayer(const std::string& name);
    virtual void deleteLayer(const vcity::URI& uri);
    virtual void setLayerName(const vcity::URI& uri, const std::string& name);

    // tile
    virtual void addTile(const vcity::URI& uriLayer, vcity::Tile& tile);
    virtual void deleteTile(const vcity::URI& uri);
    virtual void setTileName(const vcity::URI& uri, const std::string& name);

    // selection
    virtual void resetSelection();
    virtual void addSelection(const vcity::URI& uri);

private:
};
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLERGUI_HPP__
