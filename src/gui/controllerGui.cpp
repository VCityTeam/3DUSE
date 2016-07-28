// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "controllerGui.hpp"
#include "applicationGui.hpp"
#include "moc/mainWindow.hpp"
#include "osg/osgPicking.hpp"
#include "osg/osgGDAL.hpp"
#include "osg/osgTools.hpp"
#include <osg/MatrixTransform>
#include <osg/ValueObject>
////////////////////////////////////////////////////////////////////////////////
ControllerGui::ControllerGui()
{

}
////////////////////////////////////////////////////////////////////////////////
ControllerGui::~ControllerGui()
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
    uri.resetCursor();
    appGui().getTreeView()->addLayer(uri);

    // add layer in osg
    uri.resetCursor();
    appGui().getOsgScene()->addLayer(name);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::deleteLayer(const vcity::URI& uri)
{
    Controller::deleteLayer(uri);

    // delete layer in treeview
    uri.resetCursor();
    appGui().getTreeView()->deleteLayer(uri);

    // delete layer in osg scene
    uri.resetCursor();
    appGui().getOsgScene()->deleteLayer(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::setLayerName(const vcity::URI& uri, const std::string& name)
{
    // fix info bubbles bug when renaming MM
    resetSelection();

    Controller::setLayerName(uri, name);

    // set name in treeview
    uri.resetCursor();
    appGui().getTreeView()->setLayerName(uri, name);

    // set name in osg scene
    uri.resetCursor();
    appGui().getOsgScene()->getNode(uri)->setName(name);

    // restore selection MM
    uri.resetCursor();
    addSelection(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addTile(const vcity::URI& uriLayer, vcity::Tile& tile)
{
    Controller::addTile(uriLayer, tile);

    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addTile(uriLayer, tile);

    // fill osg scene
    uriLayer.resetCursor();
    appGui().getOsgScene()->addTile(uriLayer, tile);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addInfo(const vcity::URI& uriLayer, std::vector<osgInfo *> info)
{
    // fill osg scene
    uriLayer.resetCursor();
    appGui().getOsgScene()->initInfo(uriLayer, info);


    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addInfo(uriLayer, info);

}


////////////////////////////////////////////////////////////////////////////////
void ControllerGui::deleteTile(const vcity::URI& uri)
{
    Controller::deleteTile(uri);

    // delete in treeview
    uri.resetCursor();
    appGui().getTreeView()->deleteTile(uri);

    // delete in osg scene
    uri.resetCursor();
    appGui().getOsgScene()->deleteTile(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::setTileName(const vcity::URI& uri, const std::string& name)
{
    // fix info bubbles bug when renaming MM
    resetSelection();

    uri.resetCursor();
    Controller::setTileName(uri, name);

    //appGui().getTreeView()->getCurrentItem()->setText(0, name.c_str()); // MT
    uri.resetCursor();
    appGui().getTreeView()->setTileName(uri, name); // MT

    uri.resetCursor();
    appGui().getOsgScene()->setTileName(uri, name); // MT

    // restore selection MM
    uri.resetCursor();
    addSelection(uri);
}
////////////////////////////////////////////////////////////////////////////////
void loadRecTest(citygml::CityObject* node, osg::ref_ptr<osg::Group> parent, ReaderOsgCityGML& reader)
{
    osg::ref_ptr<osg::Group> grp = reader.createCityObject(node);
    parent->addChild(grp);
    citygml::CityObjects& cityObjects = node->getChildren();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for (; it != cityObjects.end(); ++it)
    {
        loadRecTest(*it, grp, reader);
    }
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    //Controller::addAssimpNode(uriLayer, node);

    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addAssimpNode(uriLayer, node);

    // fill osg scene
    uriLayer.resetCursor();
    appGui().getOsgScene()->addAssimpNode(uriLayer, node);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::deleteAssimpNode(const vcity::URI& uri)
{
    //Controller::deleteAssimpNode(uri);

    // delete in treeview
    uri.resetCursor();
    appGui().getTreeView()->deleteAssimpNode(uri);

    // delete in osg scene
    uri.resetCursor();
    appGui().getOsgScene()->deleteAssimpNode(uri);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::setAssimpNodeName(const vcity::URI& uri, const std::string& name)
{
    //Controller::setAssimpNodeName(uri, name);

    uri.resetCursor();
    appGui().getTreeView()->setAssimpNodeName(uri, name);

    uri.resetCursor();
    appGui().getOsgScene()->setAssimpNodeName(uri, name);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addMntAscNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    //Controller::addMntAscNode(uriLayer, node);

    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addMntAscNode(uriLayer, node);

    // fill osg scene
    uriLayer.resetCursor();
    appGui().getOsgScene()->addMntAscNode(uriLayer, node);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addLasNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    //Controller::addLasNode(uriLayer, node);

    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addLasNode(uriLayer, node);

    // fill osg scene
    uriLayer.resetCursor();
    appGui().getOsgScene()->addLasNode(uriLayer, node);
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addShpNode(const vcity::URI& uriLayer, OGRDataSource* poDS)
{
    Controller::addShpNode(uriLayer, poDS);

    std::string name = poDS->GetName();
    name = name.substr(name.rfind('/') + 1);

    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addShpNode(uriLayer, name);

    // fill osg scene
    uriLayer.resetCursor();
    osg::ref_ptr<osg::Node> osgNode = buildOsgGDAL(poDS);

    //We use a MatrixTransform instead of a basic group to be able to move the shp in the scene
    osg::ref_ptr<osg::Group> grp = new osg::MatrixTransform();
    grp->addChild(osgNode);
    grp->setName(name);
    //grp->getOrCreateStateSet();
    appGui().getOsgScene()->addShpNode(uriLayer, grp);
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
    if (Controller::addSelection(uri))
    {
        // select in treeview
        uri.resetCursor();
        appGui().getTreeView()->selectItem(uri);

        // select in osg
        //appGui().getPickHandler()->toggleSelected(uri);
        uri.resetCursor();
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
    for (std::vector<citygml::CityObject*>::iterator it = objs.begin(); it < objs.end(); ++it)
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
    uri.resetCursor();
    appGui().getTreeView()->deleteItem(uri);
    // delete in osg
    uri.resetCursor();
    appGui().getOsgScene()->deleteNode(uri);

    // refill treeview
    uri.resetCursor();
    vcity::URI uriTile = uri;
    while (uriTile.getDepth() > 2)
    {
        uriTile.popBack();
    }
    uriTile.setType("File");
    //std::cout << uriTile.getStringURI() << std::endl;

    uriTile.resetCursor();
    vcity::Tile* tile = vcity::app().getScene().getTile(uriTile);
    citygml::CityModel* model = tile->getCityModel();

    uri.resetCursor();
    citygml::CityObject* obj = appGui().getScene().getCityObjectNode(uri);
    citygml::ParserParams params;
    //obj->finish(*model->getAppearanceManager(), params);
    finish(model, params, obj);

    uriTile.resetCursor();
    appGui().getTreeView()->addCityObject(appGui().getTreeView()->getNode(uriTile), obj);

    // refill osg

    // create osg geometry builder
    size_t pos = tile->getCityGMLfilePath().find_last_of("/\\");
    std::string path = tile->getCityGMLfilePath().substr(0, pos);
    ReaderOsgCityGML readerOsgGml(path);
    readerOsgGml.m_settings.m_useTextures = vcity::app().getSettings().m_loadTextures;

    uriTile.resetCursor();
    appGui().getOsgScene()->buildCityObject(uriTile, appGui().getOsgScene()->getNode(uriTile)->asGroup(), obj, readerOsgGml);
}
////////////////////////////////////////////////////////////////////////////////
