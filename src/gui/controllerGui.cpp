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
void ControllerGui::addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    //Controller::addAssimpNode(uriLayer, node);

    // fill treeview
    appGui().getTreeView()->addAssimpNode(uriLayer, node);

    // fill osg scene
    appGui().getOsgScene()->addAssimpNode(uriLayer, node);
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
        //appGui().getPickHandler()->toggleSelected(uri);
        appGui().getPickHandler()->selectNode(uri);

        return true;
    }

    return false;
}
////////////////////////////////////////////////////////////////////////////////
void finish(citygml::CityModel* model, citygml::ParserParams &params, citygml::CityObject* obj)
{
    obj->finish(*model->getAppearanceManager(), params);

    std::vector<citygml::CityObject*> objs = obj->getChildren();
    for(std::vector<citygml::CityObject*>::iterator it = objs.begin(); it < objs.end(); ++it)
    {
        finish(model, params, *it);
    }
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::update(const vcity::URI& uri_)
{
    vcity::URI uri = uri_;

    appGui().resetSelectedNodes();

    // delete in treeview
    appGui().getTreeView()->deleteItem(uri);
    // delete in osg
    appGui().getOsgScene()->deleteNode(uri);

    // refill treeview
    vcity::URI uriTile = uri;
    while(uriTile.getDepth() > 2)
    {
        uriTile.pop();
    }
    uriTile.setType("Tile");
    //std::cout << uriTile.getStringURI() << std::endl;

    vcity::Tile* tile = vcity::app().getScene().getTile(uriTile);
    citygml::CityModel* model = tile->getCityModel();

    citygml::CityObject* obj = appGui().getScene().getNode(uri);
    citygml::ParserParams params;
    //obj->finish(*model->getAppearanceManager(), params);
    finish(model, params, obj);

    appGui().getTreeView()->addCityObject(appGui().getTreeView()->getNode(uriTile), obj);

    // refill osg

    // create osg geometry builder
    size_t pos = tile->getCityGMLfilePath().find_last_of("/\\");
    std::string path = tile->getCityGMLfilePath().substr(0, pos);
    ReaderOsgCityGML readerOsgGml(path);
    readerOsgGml.m_settings.m_useTextures = vcity::app().getSettings().m_loadTextures;

    appGui().getOsgScene()->buildCityObject(appGui().getOsgScene()->getNode(uriTile)->asGroup(), obj, readerOsgGml);
}
////////////////////////////////////////////////////////////////////////////////
