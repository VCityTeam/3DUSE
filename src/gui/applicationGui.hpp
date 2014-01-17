#ifndef __APPLICATIONGUI_HPP__
#define __APPLICATIONGUI_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "core/application.hpp"
#include "controllerGui.hpp"
////////////////////////////////////////////////////////////////////////////////
/// \brief The ApplicationGui class
/// Stores Common gui application data
class ApplicationGui : public vcity::Application
{
public:
    ApplicationGui();

    ControllerGui* getControllerGui();
    void setControllerGui(ControllerGui* cont);

private:
    ControllerGui* m_controllerGui;   ///< controller, needs to be allocated outside
};
////////////////////////////////////////////////////////////////////////////////
#endif // __APPLICATIONGUI_HPP__
