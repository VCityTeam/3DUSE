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
    virtual void addNode(const std::string& URI);
    virtual void deleteNode(const std::string& URI);

    virtual void addNode(const vcity::URI& URI);
    virtual void deleteNode(const vcity::URI& URI);

    //virtual void editLayer();

private:
};
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLERGUI_HPP__
