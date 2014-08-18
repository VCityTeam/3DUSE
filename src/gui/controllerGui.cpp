////////////////////////////////////////////////////////////////////////////////
#include "controllerGui.hpp"
#include "applicationGui.hpp"
#include "moc/mainWindow.hpp"
#include "osg/osgPicking.hpp"
#include "osg/osgGDAL.hpp"
#include "osg/osgTools.hpp"
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
    for( ; it != cityObjects.end(); ++it)
    {
        loadRecTest(*it, grp, reader);
    }
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addTag(const vcity::URI& uri, citygml::CityObjectTag* tag)
{
    // add in osg
    if(tag->getGeom())
    {
        // get parent osg geom
        uri.resetCursor();
        osg::ref_ptr<osg::Node> osgNode = appGui().getOsgScene()->getNode(uri);

        uri.resetCursor();
        std::cout << uri.getStringURI() << std::endl;
        osgNode->setUserValue("TAGGED", 1);
        std::cout << "osg parent tagged : " << osgTools::getURI(osgNode).getStringURI() << std::endl;

        // build osg geom for tag
        size_t pos = tag->getGeom()->m_path.find_last_of("/\\");
        std::string path = tag->getGeom()->m_path.substr(0, pos);
        uri.resetCursor();
        vcity::Tile* tile = appGui().getScene().getTile(uri);
        uri.resetCursor();
        //path = "/mnt/docs/data/dd_backup/Donnees_IGN_unzip/EXPORT_1296-13731/export-CityGML/";
        path = tile->getCityModel()->m_basePath;
        ReaderOsgCityGML readerOsgGml(path);
        readerOsgGml.m_settings.m_useTextures = vcity::app().getSettings().m_loadTextures;
        osg::ref_ptr<osg::Group> grp = readerOsgGml.createCityObject(tag->getGeom());

        citygml::CityObjects& cityObjects = tag->getGeom()->getChildren();
        citygml::CityObjects::iterator it = cityObjects.begin();
        for( ; it != cityObjects.end(); ++it)
        {
            loadRecTest(*it, grp, readerOsgGml);
        }

        grp->setName(tag->getStringId()+tag->getGeom()->getId());
        grp->getChild(0)->setName(tag->getStringId()+tag->getGeom()->getId());
        grp->setUserValue("TAG", 1);
        double ptr;
        memcpy(&ptr, &tag, sizeof(tag));
        grp->setUserValue("TAGPTR", ptr);

        std::cout << "insert osg geom" << std::endl;
        osgNode->getParent(0)->addChild(grp);

        tag->setOsg(grp);
    }

    std::cout << "tag date : " << tag->m_date.toString().toStdString() << std::endl;

    // add in treeview
    //uri.resetCursor();
    //appGui().getTreeView()->addItemGeneric(uri, tag->getStringId().c_str(), "Tag");
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addState(const vcity::URI& uri, citygml::CityObjectState* state)
{
    // add in treeview
    uri.resetCursor();
    appGui().getTreeView()->addItemGeneric(uri, state->getStringId().c_str(), "State");
}
////////////////////////////////////////////////////////////////////////////////
void ControllerGui::addDynState(const vcity::URI& uri, citygml::CityObjectDynState* state)
{
    // add in treeview
    uri.resetCursor();
    appGui().getTreeView()->addItemGeneric(uri, state->getStringId().c_str(), "DynState");
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
void ControllerGui::addShpNode(const vcity::URI& uriLayer, OGRDataSource* poDS)
{
    Controller::addShpNode(uriLayer, poDS);

    std::string name = poDS->GetName();
    name = name.substr(name.rfind('/')+1);

    // fill treeview
    uriLayer.resetCursor();
    appGui().getTreeView()->addShpNode(uriLayer, name);

    // fill osg scene
    uriLayer.resetCursor();
    osg::ref_ptr<osg::Node> osgNode = buildOsgGDAL(poDS);
    osg::ref_ptr<osg::Group> grp = new osg::Group;
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
    if(Controller::addSelection(uri))
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
    uri.resetCursor();
    appGui().getTreeView()->deleteItem(uri);
    // delete in osg
    uri.resetCursor();
    appGui().getOsgScene()->deleteNode(uri);

    // refill treeview
    uri.resetCursor();
    vcity::URI uriTile = uri;
    while(uriTile.getDepth() > 2)
    {
        uriTile.popBack();
    }
    uriTile.setType("Tile");
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
