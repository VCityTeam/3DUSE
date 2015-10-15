// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/treeView.hpp"
#include "moc/mainWindow.hpp"
#include "moc/dialogAddLayer.hpp"
#include "moc/dialogEditLayer.hpp"
#include "moc/dialogEditTile.hpp"
#include "moc/dialogEditAssimpNode.hpp"
#include "moc/dialogEditBldg.hpp"
#include "moc/dialogDynState.hpp"
#include "moc/dialogState.hpp"
#include "moc/dialogTag.hpp"
#include "moc/dialogDoc.hpp"
#include "moc/dialogYearOfConst.hpp"
#include "moc/dialogYearOfDemol.hpp"
#include "moc/dialogLink.hpp"
#include "core/application.hpp"
#include <iostream>
#include <QMenu>
#include <QLineEdit>
#include <osg/PositionAttitudeTransform>
#include "moc/dialogShpTool.hpp"
////////////////////////////////////////////////////////////////////////////////
TreeView::TreeView(QTreeWidget* tree, MainWindow* widget)
    : m_tree(tree), m_mainWindow(widget)
{
	dialogShpTool = new DialogShpTool(widget,widget);
	dialogShpTool->setModal(false);
}
////////////////////////////////////////////////////////////////////////////////
TreeView::~TreeView()
{
    delete m_actionAddTile;
    delete m_actionEditTile;
    delete m_actionDeleteTile;
    delete m_actionEditAssimpNode;
    delete m_actionDeleteAssimpNode;
    delete m_actionAddLayer;
    delete m_actionEditLayer;
    delete m_actionDeleteLayer;
    delete m_actionAddBuilding;
    delete m_actionEditBuilding;
    delete m_actionDeleteBuilding;
    delete m_actionAddState;
    delete m_actionAddDynState;
    delete m_actionAddTag;
    delete m_actionEditState;
    delete m_actionEditDynState;
    delete m_actionEditTag;
    delete m_actionDeleteState;
    delete m_actionDeleteDynState;
    delete m_actionDeleteTag;
    delete m_actionSelectAll;
    delete m_actionDeSelectAll;
    delete m_actionAddDoc;
	delete m_actionExportJSON;
	delete m_actionEditShp;
	delete m_actionAddYearOfConst;
	delete m_actionAddYearOfDemol;
	delete m_actionAddLink;
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::init()
{
    m_tree->setContextMenuPolicy(Qt::ActionsContextMenu);

    // right click menu actions
    m_actionAddTile = new QAction("Add tile", NULL);
    m_actionEditTile = new QAction("Edit tile", NULL);
    m_actionDeleteTile = new QAction("Delete tile", NULL);
	m_actionEditAssimpNode = new QAction("Edit assimp node", NULL);
    m_actionDeleteAssimpNode = new QAction("Delete assimp node", NULL);
    m_actionAddLayer = new QAction("Add layer", NULL);
    m_actionEditLayer = new QAction("Edit layer", NULL);
    m_actionDeleteLayer = new QAction("Delete layer", NULL);
    m_actionAddBuilding = new QAction("Add building", NULL);
    m_actionEditBuilding = new QAction("Edit building", NULL);
    m_actionDeleteBuilding = new QAction("Delete building", NULL);
    m_actionAddState = new QAction("Add State", NULL);
    m_actionAddDynState = new QAction("Add dynamic State", NULL);
    m_actionAddTag = new QAction("Add Tag", NULL);
    m_actionEditState = new QAction("Edit State", NULL);
    m_actionEditDynState = new QAction("Edit dynamic State", NULL);
    m_actionEditTag = new QAction("Edit Tag", NULL);
    m_actionDeleteState = new QAction("Delete State", NULL);
    m_actionDeleteDynState = new QAction("Delete dynamic State", NULL);
    m_actionDeleteTag = new QAction("Delete Tag", NULL);
    m_actionSelectAll = new QAction("Check all", NULL);
    m_actionDeSelectAll = new QAction("Check none", NULL);
    m_actionAddDoc = new QAction("Add document", NULL);
	m_actionExportJSON = new QAction("Export JSON", NULL);
	m_actionEditShp = new QAction("Edit Shp", NULL);
	//ajout yearOfConstruction/yearOfDemolition/creationDate/terminationDate
	m_actionAddYearOfConst = new QAction("Add creationDate",NULL);
	m_actionAddYearOfDemol = new QAction("Add terminationDate",NULL);
	//add xLink
	m_actionAddLink = new QAction("Add link to a new object", NULL);

    // connect right click menu actions
    connect(m_actionAddTile, SIGNAL(triggered()), this, SLOT(slotAddTile()));
    connect(m_actionEditTile, SIGNAL(triggered()), this, SLOT(slotEditTile()));
    connect(m_actionDeleteTile, SIGNAL(triggered()), this, SLOT(slotDeleteTile()));
	connect(m_actionEditAssimpNode, SIGNAL(triggered()), this, SLOT(slotEditAssimpNode()));
	connect(m_actionDeleteAssimpNode, SIGNAL(triggered()), this, SLOT(slotDeleteAssimpNode()));
    connect(m_actionAddLayer, SIGNAL(triggered()), this, SLOT(slotAddLayer()));
    connect(m_actionEditLayer, SIGNAL(triggered()), this, SLOT(slotEditLayer()));
    connect(m_actionDeleteLayer, SIGNAL(triggered()), this, SLOT(slotDeleteLayer()));
    connect(m_actionAddBuilding, SIGNAL(triggered()), this, SLOT(slotAddBuilding()));
    connect(m_actionEditBuilding, SIGNAL(triggered()), this, SLOT(slotEditBuilding()));
    connect(m_actionDeleteBuilding, SIGNAL(triggered()), this, SLOT(slotDeleteBuilding()));
    connect(m_actionAddState, SIGNAL(triggered()), this, SLOT(slotAddState()));
    connect(m_actionAddDynState, SIGNAL(triggered()), this, SLOT(slotAddDynState()));
    connect(m_actionAddTag, SIGNAL(triggered()), this, SLOT(slotAddTag()));
    connect(m_actionEditState, SIGNAL(triggered()), this, SLOT(slotEditState()));
    connect(m_actionEditDynState, SIGNAL(triggered()), this, SLOT(slotEditDynState()));
    connect(m_actionEditTag, SIGNAL(triggered()), this, SLOT(slotEditTag()));
    connect(m_actionDeleteState, SIGNAL(triggered()), this, SLOT(slotDeleteState()));
    connect(m_actionDeleteDynState, SIGNAL(triggered()), this, SLOT(slotDeleteDynState()));
    connect(m_actionDeleteTag, SIGNAL(triggered()), this, SLOT(slotDeleteTag()));
    connect(m_actionSelectAll, SIGNAL(triggered()), this, SLOT(slotCheckAll()));
    connect(m_actionDeSelectAll, SIGNAL(triggered()), this, SLOT(slotUnCheckAll()));
    connect(m_actionAddDoc, SIGNAL(triggered()), this, SLOT(slotAddDoc()));
	connect(m_actionExportJSON, SIGNAL(triggered()), this, SLOT(slotExportJSON()));
	connect(m_actionEditShp, SIGNAL(triggered()), this, SLOT(slotEditShp()));
	//ajout yearOfConstruction/yearOfDemolition
	connect(m_actionAddYearOfConst, SIGNAL(triggered()), this, SLOT(slotAddYearOfConst()));
	connect(m_actionAddYearOfDemol, SIGNAL(triggered()), this, SLOT(slotAddYearOfDemol()));
	connect(m_actionAddLink, SIGNAL(triggered()), this, SLOT(slotAddLink()));

    /*connect(m_actionEditLayer, SIGNAL(triggered()), this, SLOT(slotEditLayer()));
    connect(m_actionEditBuilding, SIGNAL(triggered()), this, SLOT(slotEditBldg()));

    connect(m_actionAddState, SIGNAL(triggered()), m_mainWindow, SLOT(optionAddState()));
    connect(m_actionAddDynState, SIGNAL(triggered()), m_mainWindow, SLOT(optionAddDynState()));
    connect(m_actionAddTag, SIGNAL(triggered()), m_mainWindow, SLOT(optionAddTag()));*/


    //connect(select, SIGNAL(triggered()), this, SLOT(selectNodeHandler()));
    //connect(insertChild, SIGNAL(triggered()), this, SLOT(insertChildHandler()));
    //connect(deleteNode, SIGNAL(triggered()), this, SLOT(deleteNodeHandler()));
    //connect(showInfo, SIGNAL(triggered()), this, SLOT(showInfoHandler()));


    // connect other signals
    // checkbox signal
    connect(m_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(slotItemChanged(QTreeWidgetItem*, int)));

    // right click handling : to activate actions depending on node type
    connect(m_tree, SIGNAL(itemPressed(QTreeWidgetItem*, int)), this, SLOT(slotSelectNode(QTreeWidgetItem*,int)));

    // show info
    //connect(m_tree, SIGNAL(itemClicked(QTreeWidgetItem*,int)), m_mainWindow, SLOT(showInfoHandler()));
    connect(m_tree, SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(slotItemClicked(QTreeWidgetItem*,int)));

    // double click
    connect(m_tree, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), this, SLOT(slotItemDoubleClicked(QTreeWidgetItem*,int)));

    reset();
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::reset()
{
    m_tree->clear();
    m_tree->reset();

    QTreeWidgetItem* root = createItemRoot();
    m_tree->addTopLevelItem(root);

    QTreeWidgetItem* grid = createItemGeneric("grid", "Grid");
    root->addChild(grid);

	QTreeWidgetItem* layer0 = createItemLayer("layer_CityGML", "LayerCityGML");
    root->addChild(layer0);

    QTreeWidgetItem* layer1 = createItemLayer("layer_Assimp", "LayerAssimp");
    root->addChild(layer1);

    QTreeWidgetItem* layer2 = createItemLayer("layer_Mnt", "LayerMnt");
    root->addChild(layer2);

	QTreeWidgetItem* layer3 = createItemLayer("layer_Las", "LayerLas");
    root->addChild(layer3);

    QTreeWidgetItem* layer4 = createItemLayer("layer_Shp", "LayerShp");
    root->addChild(layer4);
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::createItemGeneric(const QString& name, const QString& type, const bool checkable)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(name));
	if (checkable)
	{
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
	}
    item->setText(1, type);

    return item;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::addItemGeneric(const vcity::URI& uri, const QString& name, const QString& type)
{
    QTreeWidgetItem* item = getNode(uri);
    if(item)
    {
        item->addChild(createItemGeneric(name, type));
    }

    return item;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::createItemRoot()
{
    QTreeWidgetItem* item = createItemGeneric("root", "Root");
    return item;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::createItemLayer(const QString& name, const QString& type)
{
    QTreeWidgetItem* item = createItemGeneric(name, type);
    return item;
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::deleteItem(const vcity::URI& uri)
{
    QTreeWidgetItem* tile = getNode(uri);
    if(tile)
    {
        QTreeWidgetItem* parent = tile->parent();
        parent->removeChild(tile);
    }
}
////////////////////////////////////////////////////////////////////////////////
vcity::URI TreeView::getURI(QTreeWidgetItem* item) const
{
    vcity::URI uri;
    uri.setType(item->text(1).toStdString());
    QTreeWidgetItem* current = item;

    while(current && current->text(1) != "Root")
    {
        uri.prepend(current->text(0).toStdString(), current->text(1).toStdString());
        current = current->parent();
    }

    return uri;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidget* TreeView::getTree()
{
    return m_tree;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::getCurrentItem()
{
    return m_tree->currentItem();
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addLayer(const vcity::URI& uri)
{
    /*QTreeWidgetItem* layer = createItemLayer(uri.getNode(0).c_str());
    m_tree->topLevelItem(0)->addChild(layer);*/	// MT TODO
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::setLayerName(const vcity::URI& uri, const std::string& name)
{
    QTreeWidgetItem* item = getNode(uri);
    if(item)
    {
        item->setText(0, name.c_str());
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::deleteLayer(const vcity::URI& uri)
{
    QTreeWidgetItem* tile = getNode(uri);
    if(tile)
    {
        QTreeWidgetItem* parent = tile->parent();
        parent->removeChild(tile);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addCityObject(QTreeWidgetItem* parent, citygml::CityObject* node)
{
	std::string type = (node->_isXlink==citygml::xLinkState::LINKED) ? "xLink:"+node->getTypeAsString() : node->getTypeAsString();
    QTreeWidgetItem* item = createItemGeneric(node->getId().c_str(), type.c_str());
    parent->addChild(item);

    citygml::CityObjects& cityObjects = node->getChildren();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        addCityObject(item, *it);
    }
	for(std::vector<citygml::Object*>::iterator it = node->getXLinkTargets().begin();it != node->getXLinkTargets().end(); ++it)
	{
        addCityObject(item,(citygml::CityObject*) *it);
    }
    // add temporal elements after
    for(auto* tag : node->getTags())
    {
        item->addChild(createItemGeneric(tag->getStringId().c_str(), "Tag"));
    }

    for(auto* state : node->getStates())
    {
        if(dynamic_cast<citygml::CityObjectDynState*>(state))
        {
            item->addChild(createItemGeneric(state->getStringId().c_str(), "DynState"));
        }
        else
        {
            item->addChild(createItemGeneric(state->getStringId().c_str(), "State"));
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addTile(const vcity::URI& uriLayer, vcity::Tile& tile)
{
    m_tree->blockSignals(true);

    //QTreeWidgetItem* root = m_tree->topLevelItem(0);
    QTreeWidgetItem* layer = getNode(uriLayer);

    QTreeWidgetItem* item = createItemGeneric(tile.getName().c_str(), "Tile");
    layer->addChild(item);

    citygml::CityModel* citymodel = tile.getCityModel();
    citygml::CityObjects& cityObjects = citymodel->getCityObjectsRoots();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        addCityObject(item, *it);
    }

    m_tree->expandToDepth(1);

    m_tree->blockSignals(false);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::setTileName(const vcity::URI& uri, const std::string& name)
{
    QTreeWidgetItem* item = getNode(uri);
    if(item)
    {
        item->setText(0, name.c_str());
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::deleteTile(const vcity::URI& uri)
{
    QTreeWidgetItem* tile = getNode(uri);
    if(tile)
    {
        QTreeWidgetItem* parent = tile->parent();
        parent->removeChild(tile);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addAssimpNodeRecursively(QTreeWidgetItem* parent, const osg::ref_ptr<osg::Node> node, std::string strLevel)
{
	if (node->asGroup())
	{
		//std::cout << strLevel << node->className() << ": " << node->asGroup()->getNumChildren() << " children" << std::endl;

		QString className = node->className();
		if (strLevel == "")
			className = "AssimpNode";

		QTreeWidgetItem* item = createItemGeneric(node->getName().c_str(), className);
		parent->addChild(item);

		for ( unsigned int i=0; i<node->asGroup()->getNumChildren(); ++i )
			addAssimpNodeRecursively(item, node->asGroup()->getChild(i), strLevel+"-");
	}
	else
	{
		if (node->className() == std::string("Geode"))
		{
			if (node->asGeode()->getNumDrawables())
			{
				//std::cout << strLevel << node->className() << ": drawable(s): ";

				QTreeWidgetItem* item = createItemGeneric(node->getName().c_str(), node->className());
				parent->addChild(item);

				for ( unsigned int i=0; i<node->asGeode()->getNumDrawables(); ++i )
				{
					//std::cout << node->asGeode()->getDrawable(i)->getName() << " ";

					QTreeWidgetItem* itemD = createItemGeneric(node->asGeode()->getDrawable(i)->getName().c_str(), node->asGeode()->getDrawable(i)->className(), false);
					item->addChild(itemD);
				}
				//std::cout << std::endl;
			}
			else
			{
				//std::cout << strLevel << node->className() << ": no drawable" << std::endl;

				/*QTreeWidgetItem* item = createItemGeneric("empty", node->className());
				parent->addChild(item);*/
			}
		}
	}
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    m_tree->blockSignals(true);

    //QTreeWidgetItem* root = m_tree->topLevelItem(0);
    QTreeWidgetItem* layer = getNode(uriLayer);

    /*QTreeWidgetItem* item = createItemGeneric(node->getName().c_str(), "AssimpNode");
    layer->addChild(item);*/

	addAssimpNodeRecursively(layer/*item*/, node, "");

    m_tree->expandToDepth(1);

    m_tree->blockSignals(false);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::setAssimpNodeName(const vcity::URI& uri, const std::string& name)
{
    QTreeWidgetItem* item = getNode(uri);
    if(item)
    {
        item->setText(0, name.c_str());
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::deleteAssimpNode(const vcity::URI& uri)
{
    QTreeWidgetItem* assimpNode = getNode(uri);
    if(assimpNode)
    {
        QTreeWidgetItem* parent = assimpNode->parent();
        parent->removeChild(assimpNode);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addMntAscNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    m_tree->blockSignals(true);

    //QTreeWidgetItem* root = m_tree->topLevelItem(0);
    QTreeWidgetItem* layer = getNode(uriLayer);

    QTreeWidgetItem* item = createItemGeneric(node->getName().c_str(), "MntAscNode");
    layer->addChild(item);

    m_tree->expandToDepth(1);

    m_tree->blockSignals(false);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addLasNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    m_tree->blockSignals(true);

    //QTreeWidgetItem* root = m_tree->topLevelItem(0);
    QTreeWidgetItem* layer = getNode(uriLayer);

    QTreeWidgetItem* item = createItemGeneric(node->getName().c_str(), "LasNode");
    layer->addChild(item);

    m_tree->expandToDepth(1);

    m_tree->blockSignals(false);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addShpNode(const vcity::URI& uriLayer, const std::string& nodeName)
{
    m_tree->blockSignals(true);

    //QTreeWidgetItem* root = m_tree->topLevelItem(0);
    QTreeWidgetItem* layer = getNode(uriLayer);

    QTreeWidgetItem* item = createItemGeneric(QString(nodeName.c_str()), "ShpNode");
    layer->addChild(item);

    m_tree->expandToDepth(1);

    m_tree->blockSignals(false);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::selectItem(const vcity::URI& uri)
{
    QTreeWidgetItem* item = getNode(uri);
    if(item)
    {
        item->setSelected(true);
        getTree()->scrollToItem(item);
    }
}
////////////////////////////////////////////////////////////////////////////////
void resetSelectionRec(QTreeWidgetItem* item)
{
    item->setSelected(false);

    int count = item->childCount();
    for(int i=0; i<count; ++i)
    {
        QTreeWidgetItem* current = item->child(i);
        resetSelectionRec(current);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::resetSelection()
{
    QTreeWidgetItem* root = m_tree->topLevelItem(0);
    resetSelectionRec(root);
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::getNode(const vcity::URI& uri)
{
    QTreeWidgetItem* res = nullptr;
    QTreeWidgetItem* current = m_tree->topLevelItem(0);

    while(uri.getCursor() < uri.getDepth())
    {
        int count = current->childCount();
        for(int i=0; i<count; ++i)
        {
            QTreeWidgetItem* item = current->child(i);
            //vcity::log() << item->text(0).toStdString() << " -> " << uri.getNode(maxDepth-depth) << "\n";
            if(item->text(0).toStdString() == uri.getCurrentNode())
            {
                current = item;
                res = current;
                break;
            }
        }
        uri.popFront();
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotSelectNode(QTreeWidgetItem* item, int /*column*/)
{
    //std::cout << "select node : " << item->text(0).toStdString() << "," << item->text(1).toStdString() << std::endl;

    // insane brute force hack :
    // every right click, remove all actions on the QTreeWidget
    // and depending on the type of the node clicked, activate needed actions

    resetActions();

    if(item->text(1) == "Root")
    {
        //std::cout << "Root" << std::endl;
        m_tree->addAction(m_actionAddLayer);
    }
    else if(item->text(1) == "LayerCityGML" || item->text(1) == "LayerAssimp" || item->text(1) == "LayerMnt" || item->text(1) == "LayerLas" || item->text(1) == "LayerShp")
    {
        std::cout << item->text(1).toStdString() << std::endl;
        m_tree->addAction(m_actionDeleteLayer);
        m_tree->addAction(m_actionEditLayer);
		if(item->text(1) == "LayerCityGML")
			m_tree->addAction(m_actionAddTile);
		if(item->text(1) == "LayerLas")
			m_tree->addAction(m_actionExportJSON);
    }
    else if(item->text(1) == "Tile")
    {
        //std::cout << "Tile" << std::endl;
        m_tree->addAction(m_actionDeleteTile);
        m_tree->addAction(m_actionEditTile);
        m_tree->addAction(m_actionAddBuilding);
		m_tree->addAction(m_actionAddYearOfConst);
		m_tree->addAction(m_actionAddYearOfDemol);
    }
	else if(item->text(1) == "AssimpNode")
    {
        //std::cout << "AssimpNode" << std::endl;
        m_tree->addAction(m_actionDeleteAssimpNode);
		m_tree->addAction(m_actionEditAssimpNode);
    }
    else if(item->text(1) == "Building")
    {
        //std::cout << "Building" << std::endl;
        m_tree->addAction(m_actionDeleteBuilding);
        m_tree->addAction(m_actionEditBuilding);
        m_tree->addAction(m_actionAddDoc);
    }
    else if(item->text(1) == "Tag")
    {
        //std::cout << "Tag" << std::endl;
        m_tree->addAction(m_actionDeleteTag);
        m_tree->addAction(m_actionEditTag);
        m_tree->addAction(m_actionAddDoc);
    }
    else if(item->text(1) == "State")
    {
        //std::cout << "State" << std::endl;
        m_tree->addAction(m_actionDeleteState);
        m_tree->addAction(m_actionEditState);
        m_tree->addAction(m_actionAddDoc);
    }
    else if(item->text(1) == "DynState")
    {
        //std::cout << "DynState" << std::endl;
        m_tree->addAction(m_actionDeleteDynState);
        m_tree->addAction(m_actionEditDynState);
        m_tree->addAction(m_actionAddDoc);
    }
	else if(item->text(1) == "ShpNode")
    {
        //std::cout << "ShpNode" << std::endl;
        m_tree->addAction(m_actionEditShp);
    }

    if(item->text(1) == "GenericCityObject" || item->text(1) == "Building" ||
       item->text(1) == "Room" || item->text(1) == "BuildingInstallation" ||
       item->text(1) == "BuildingFurniture" || item->text(1) == "Door" ||
       item->text(1) == "Window" || item->text(1) == "CityFurniture" ||
       item->text(1) == "Track" || item->text(1) == "Road" ||
       item->text(1) == "Railway" || item->text(1) == "Square" ||
       item->text(1) == "PlantCover" || item->text(1) == "SolitaryVegetationObject" ||
       item->text(1) == "WaterBody" || item->text(1) == "TINRelief" ||
       item->text(1) == "LandUse" || item->text(1) == "Tunnel" ||
       item->text(1) == "Bridge" || item->text(1) == "BridgeConstructionElement" ||
       item->text(1) == "BridgeInstallation" || item->text(1) == "BridgePart" ||
       item->text(1) == "BuildingPart" || item->text(1) == "WallSurface" ||
       item->text(1) == "RoofSurface" || item->text(1) == "GroundSurface" ||
       item->text(1) == "ClosureSurface" || item->text(1) == "FloorSurface" ||
       item->text(1) == "InteriorWallSurface" || item->text(1) == "CeilingSurface")
    {
        m_tree->addAction(m_actionAddTag);
        m_tree->addAction(m_actionAddState);
        m_tree->addAction(m_actionAddDynState);
        m_tree->addAction(m_actionAddDoc);
		m_tree->addAction(m_actionAddYearOfConst);
		m_tree->addAction(m_actionAddYearOfDemol);
		//m_tree->addAction(m_actionAddLink);
    }

    // actions on all types
    m_tree->addAction(m_actionSelectAll);
    m_tree->addAction(m_actionDeSelectAll);
}
////////////////////////////////////////////////////////////////////////////////
void setItemStateRec(QTreeWidgetItem* item, bool state)
{
    if(!item) return;

    int count = item->childCount();
    for(int i=0; i<count; ++i)
    {
        Qt::CheckState s = state?Qt::CheckState::Checked:Qt::CheckState::Unchecked;
        item->child(i)->setCheckState(0, s);
        setItemStateRec(item->child(i), state);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotItemChanged(QTreeWidgetItem* item, int /*column*/)
{
    Qt::CheckState state = item->checkState(0);
    bool show = true;
    if(state == Qt::CheckState::Unchecked) show = false;
    appGui().getOsgScene()->showNode(getURI(item), show);

    if(show)
    {
        while((item = item->parent()))
        {
            item->setCheckState(0, Qt::CheckState::Checked);
            appGui().getOsgScene()->showNode(getURI(item), true);
        }
    }

    //setItemStateRec(item, show);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotItemClicked(QTreeWidgetItem* item, int)
{
    vcity::URI uri = getURI(item);

    if(uri.getDepth() > 0)
    {
        m_mainWindow->updateTextBox(uri);

        appGui().getControllerGui().resetSelection();
        appGui().getControllerGui().addSelection(uri);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotItemDoubleClicked(QTreeWidgetItem* item, int)
{
    appGui().getOsgScene()->centerOn(getURI(item));
}
////////////////////////////////////////////////////////////////////////////////
void searchNode(TreeView* tv, QTreeWidgetItem* node, const QString& filter)
{
    if(node)
    {
        int count = node->childCount();
        for(int i=0; i<count; ++i)
        {
            QTreeWidgetItem* item = node->child(i);
            if(item->text(0).contains(filter, Qt::CaseSensitivity::CaseInsensitive))
            {
                // select node
                appGui().getControllerGui().addSelection(tv->getURI(item));
            }
            searchNode(tv, item, filter);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotFilter()
{
    //std::cout << "filter : " << appGui().getMainWindow()->getFilter()->text().toStdString() << std::endl;

    appGui().getControllerGui().resetSelection();
    searchNode(this, m_tree->topLevelItem(0), appGui().getMainWindow()->getFilter()->text());
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddTile()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditTile()
{
    DialogEditTile diag;
    diag.editTile(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteTile()
{
    appGui().getControllerGui().deleteTile(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditAssimpNode()
{
    DialogEditAssimpNode diag;
    diag.editAssimpNode(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteAssimpNode()
{
    appGui().getControllerGui().deleteAssimpNode(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddLayer()
{
    DialogAddLayer diag;
    diag.addLayer();
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditLayer()
{
    DialogEditLayer diag;
    diag.editLayer(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteLayer()
{
    appGui().getControllerGui().deleteLayer(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddBuilding()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditBuilding()
{
    DialogEditBldg diag;
    diag.edit(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteBuilding()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddState()
{
    DialogState diag;
    diag.addState(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddDynState()
{
    DialogDynState diag;
    diag.addDynState(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddTag()
{
    DialogTag diag;
    diag.addTag(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditState()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditDynState()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditTag()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteState()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteDynState()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotDeleteTag()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotCheckAll()
{
    setItemStateRec(getCurrentItem(), true);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotUnCheckAll()
{
    setItemStateRec(getCurrentItem(), false);
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddDoc()
{
    DialogDoc diag;
    diag.addDoc(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotExportJSON()
{
	// function ?
	vcity::abstractLayer* layer = vcity::app().getScene().getLayer(getURI(getCurrentItem()));
	vcity::LayerLas* layerLas = NULL;
    if(layer)
		layerLas = dynamic_cast<vcity::LayerLas*>(layer);

	if (layerLas)
		layerLas->exportJSON();
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotEditShp()
{
	dialogShpTool->Setup(getURI(getCurrentItem()));
	dialogShpTool->show();
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddYearOfConst()
{
	DialogYearOfConst diag;
	diag.editDates(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddYearOfDemol()
{
	DialogYearOfDemol diag;
	diag.editDates(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::slotAddLink()
{
	DialogLink diag;
	diag.addLink(getURI(getCurrentItem()));
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::resetActions()
{
    m_tree->removeAction(m_actionAddTile);
    m_tree->removeAction(m_actionEditTile);
    m_tree->removeAction(m_actionDeleteTile);
    m_tree->removeAction(m_actionEditAssimpNode);
    m_tree->removeAction(m_actionDeleteAssimpNode);
    m_tree->removeAction(m_actionAddLayer);
    m_tree->removeAction(m_actionEditLayer);
    m_tree->removeAction(m_actionDeleteLayer);
    m_tree->removeAction(m_actionAddBuilding);
    m_tree->removeAction(m_actionEditBuilding);
    m_tree->removeAction(m_actionDeleteBuilding);
    m_tree->removeAction(m_actionAddState);
    m_tree->removeAction(m_actionAddDynState);
    m_tree->removeAction(m_actionAddTag);
    m_tree->removeAction(m_actionEditState);
    m_tree->removeAction(m_actionEditTag);
    m_tree->removeAction(m_actionDeleteState);
    m_tree->removeAction(m_actionDeleteTag);
    m_tree->removeAction(m_actionSelectAll);
    m_tree->removeAction(m_actionDeSelectAll);
    m_tree->removeAction(m_actionAddDoc);
	m_tree->removeAction(m_actionExportJSON);
	m_tree->removeAction(m_actionAddYearOfConst);
	m_tree->removeAction(m_actionAddYearOfDemol);
	m_tree->removeAction(m_actionAddLink);
}
////////////////////////////////////////////////////////////////////////////////
