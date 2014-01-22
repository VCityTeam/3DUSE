#ifndef __APPLICATIONGUI_HPP__
#define __APPLICATIONGUI_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "core/application.hpp"
#include "controllerGui.hpp"
#include "osg/osgScene.hpp"
#include "moc/treeView.hpp"
////////////////////////////////////////////////////////////////////////////////
/// \brief The ApplicationGui class
/// Stores Common gui application data
class ApplicationGui : public vcity::Application
{
public:
    ApplicationGui();
    ~ApplicationGui();

    ControllerGui& getControllerGui();
    void setControllerGui(ControllerGui* cont);

    osg::ref_ptr<OsgScene> getOsgScene();
    TreeView* getTreeView();

    void setOsgScene(osg::ref_ptr<OsgScene> scene);
    void setTreeView(TreeView* treeview);


private:
    ControllerGui* m_controllerGui;     ///< controller, needs to be allocated outside
    osg::ref_ptr<OsgScene> m_osgScene;  ///< osg scene (for rendering)
    TreeView* m_treeView;              ///< Qt treeview
};
////////////////////////////////////////////////////////////////////////////////
ApplicationGui& appGui();
////////////////////////////////////////////////////////////////////////////////
#endif // __APPLICATIONGUI_HPP__
