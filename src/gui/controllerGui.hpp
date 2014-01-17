#ifndef __CONTROLLERGUI_HPP__
#define __CONTROLLERGUI_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "core/controller.hpp"
////////////////////////////////////////////////////////////////////////////////
class ControllerGui : public vcity::Controller
{
public:
    ControllerGui(vcity::Application* app);

    virtual void reset();

    virtual void addNode(const vcity::URI& uri);
    virtual void deleteNode(const vcity::URI& uri);

    // layer
    virtual void addLayer(const std::string& name);
    virtual void deleteLayer(const vcity::URI& uri);
    virtual void setLayerName(const vcity::URI& uri, const std::string& name);

private:
};
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLERGUI_HPP__
