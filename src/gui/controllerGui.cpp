////////////////////////////////////////////////////////////////////////////////
#include "controllerGui.hpp"
#include "applicationGui.hpp"
#include "moc/mainWindow.hpp"
#include "osg/osgPicking.hpp"
////////////////////////////////////////////////////////////////////////////////
ControllerGui::ControllerGui()
{

}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::reset()
{
    Controller::reset();
    // reset all
    // unload scene
    // unload treeview
    // unload osg

    // add first layer
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addNode(const vcity::URI& uri)
{
    Controller::addNode(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::deleteNode(const vcity::URI& uri)
{
    Controller::deleteNode(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addLayer(const std::string& name)
{
    vcity::URI uri;
    uri.append(name);
    Controller::addLayer(name);

    // add layer in treeview
    appGui().getTreeView()->addLayer(uri);

    // add layer in osg
    appGui().getOsgScene()->addLayer(name);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::deleteLayer(const vcity::URI& uri)
{
    Controller::deleteLayer(uri);

    // delete layer in treeview
    appGui().getTreeView()->deleteLayer(uri);

    // delete layer in osg scene
    appGui().getOsgScene()->deleteLayer(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::setLayerName(const vcity::URI& uri, const std::string& name)
{
    Controller::setLayerName(uri, name);

    // set name in treeview
    appGui().getTreeView()->setLayerName(uri, name);

    // set name in osg scene
    appGui().getOsgScene()->getNode(uri)->setName(name);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addTile(const vcity::URI& uriLayer, vcity::Tile& tile)
{
    Controller::addTile(uriLayer, tile);

    // fill treeview
    appGui().getTreeView()->addTile(uriLayer, tile);

    // fill osg scene
    appGui().getOsgScene()->addTile(uriLayer, tile);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::deleteTile(const vcity::URI& uri)
{
    Controller::deleteTile(uri);

    // delete in treeview
    appGui().getTreeView()->deleteTile(uri);

    // delete in osg scene
    appGui().getOsgScene()->deleteTile(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::setTileName(const vcity::URI& uri, const std::string& name)
{
    Controller::setTileName(uri, name);
    appGui().getTreeView()->getCurrentItem()->setText(0, name.c_str());
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::resetSelection()
{
    // reset in treeview
    appGui().getTreeView()->resetSelection();

    // reset in osg scene
    appGui().getPickHandler()->resetPicking();

    // must be done last
    Controller::resetSelection();
}
////////////////////////////////////////////////////////////////////////////////
bool ControllerGui::addSelection(const vcity::URI& uri)
{
    if(Controller::addSelection(uri))
    {
        // select in treeview
        appGui().getTreeView()->selectItem(uri);

        // select in osg
        appGui().getPickHandler()->toggleSelected(uri);

        return true;
    }

    return false;
}
////////////////////////////////////////////////////////////////////////////////
