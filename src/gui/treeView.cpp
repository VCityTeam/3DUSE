////////////////////////////////////////////////////////////////////////////////
#include "moc/treeView.hpp"
#include "moc/mainWindow.hpp"
#include "moc/dialogEditLayer.hpp"
#include "moc/dialogEditBldg.hpp"
#include <iostream>
#include <QMenu>
#include <osg/PositionAttitudeTransform>
////////////////////////////////////////////////////////////////////////////////
TreeView::TreeView(QTreeWidget* tree, MainWindow* widget)
    : m_tree(tree), m_mainWindow(widget)
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::init()
{
    // actions
    m_tree->setContextMenuPolicy(Qt::ActionsContextMenu);

    m_actionAddTile = new QAction("Add tile", NULL);
    m_actionEditTile = new QAction("Edit tile", NULL);
    m_actionDeleteTile = new QAction("Delete tile", NULL);
    m_actionAddLayer = new QAction("Add layer", NULL);
    m_actionEditLayer = new QAction("Edit layer", NULL);
    m_actionDeleteLayer = new QAction("Delete layer", NULL);
    m_actionAddBuilding = new QAction("Add building", NULL);
    m_actionEditBuilding = new QAction("Edit building", NULL);
    m_actionDeleteBuilding = new QAction("Delete building", NULL);
    m_actionAddFlag = new QAction("Add Flag", NULL);
    m_actionAddDynFlag = new QAction("Add dynamic Flag", NULL);
    m_actionAddTag = new QAction("Add Tag", NULL);
    m_actionEditFlag = new QAction("Edit Flag", NULL);
    m_actionEditTag = new QAction("Edit Tag", NULL);
    m_actionDeleteFlag = new QAction("Delete Flag", NULL);
    m_actionDeleteTag = new QAction("Delete Tag", NULL);

    //connect(select, SIGNAL(triggered()), this, SLOT(selectNodeHandler()));
    //connect(insertChild, SIGNAL(triggered()), this, SLOT(insertChildHandler()));
    //connect(deleteNode, SIGNAL(triggered()), this, SLOT(deleteNodeHandler()));
    //connect(showInfo, SIGNAL(triggered()), this, SLOT(showInfoHandler()));

    connect(m_actionEditLayer, SIGNAL(triggered()), this, SLOT(editLayer()));
    connect(m_actionEditBuilding, SIGNAL(triggered()), this, SLOT(editBldg()));

    connect(m_actionAddFlag, SIGNAL(triggered()), m_mainWindow, SLOT(optionAddFlag()));
    connect(m_actionAddDynFlag, SIGNAL(triggered()), m_mainWindow, SLOT(optionAddDynFlag()));
    connect(m_actionAddTag, SIGNAL(triggered()), m_mainWindow, SLOT(optionAddTag()));

    // right click handling : to activate actions depending on node type
    connect(m_tree, SIGNAL(itemPressed(QTreeWidgetItem*, int)), this, SLOT(selectNodeCB(QTreeWidgetItem*,int)));

    // show info
    connect(m_tree, SIGNAL(itemClicked(QTreeWidgetItem*,int)), m_mainWindow, SLOT(showInfoHandler()));
    connect(m_tree, SIGNAL(itemClicked(QTreeWidgetItem*,int)), m_mainWindow, SLOT(selectNodeHandler(QTreeWidgetItem*,int)));

    reset();
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::reset()
{
    m_tree->clear();
    m_tree->reset();

    QTreeWidgetItem* root = addItemRoot();
    m_tree->addTopLevelItem(root);

    QTreeWidgetItem* layer = addItemLayer("layer_CityGML");
    root->addChild(layer);
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::addItemGeneric( const QString& name, const QString& type)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(name));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, Qt::Checked);
    item->setText(1, type);

    return item;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::addItemRoot()
{
    QTreeWidgetItem* item = addItemGeneric("root", "Root");
    return item;
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::addItemLayer(const QString& name)
{
    QTreeWidgetItem* item = addItemGeneric(name, "Layer");
    return item;
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::deleteItem(const std::string& /*URI*/)
{
}
////////////////////////////////////////////////////////////////////////////////
QTreeWidgetItem* TreeView::findItem(const vcity::URI& /*URI*/) const
{
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
vcity::URI TreeView::getURI(QTreeWidgetItem* item) const
{
    vcity::URI uri;
    uri.setType(item->text(1).toStdString());
    QTreeWidgetItem* current = item;

    while(current)
    {
        uri.append(current->text(0).toStdString());
        current = current->parent();
    }

    return uri;
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::selectNodeCB(QTreeWidgetItem* item, int /*column*/)
{
    std::cout << "select node : " << item->text(0).toStdString() << "," << item->text(1).toStdString() << std::endl;

    // insane brute force hack :
    // every right click, remove all actions on the QTreeWidget
    // and depending on the type of the node clicked, activate needed actions

    resetActions();

    if(item->text(1).toStdString() == "Root")
    {
        std::cout << "Root" << std::endl;
        m_tree->addAction(m_actionAddLayer);
    }
    else if(item->text(1).toStdString() == "Layer")
    {
        std::cout << "Layer" << std::endl;
        m_tree->addAction(m_actionDeleteLayer);
        m_tree->addAction(m_actionEditLayer);
        m_tree->addAction(m_actionAddTile);
    }
    else if(item->text(1).toStdString() == "Tile")
    {
        std::cout << "Layer" << std::endl;
        m_tree->addAction(m_actionDeleteTile);
        m_tree->addAction(m_actionEditTile);
        m_tree->addAction(m_actionAddBuilding);
    }
    else if(item->text(1).toStdString() == "Building")
    {
        std::cout << "Building" << std::endl;
        m_tree->addAction(m_actionDeleteBuilding);
        m_tree->addAction(m_actionEditBuilding);
        m_tree->addAction(m_actionAddTag);
        m_tree->addAction(m_actionAddFlag);
        m_tree->addAction(m_actionAddDynFlag);
    }
    else if(item->text(1).toStdString() == "Tag")
    {
        std::cout << "Tag" << std::endl;
        m_tree->addAction(m_actionDeleteTag);
        m_tree->addAction(m_actionEditTag);
    }
    else if(item->text(1).toStdString() == "Flag")
    {
        std::cout << "Flag" << std::endl;
        m_tree->addAction(m_actionDeleteFlag);
        m_tree->addAction(m_actionEditFlag);
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::addBldg()
{

}
////////////////////////////////////////////////////////////////////////////////
void TreeView::editLayer()
{
    DialogEditLayer diag;
    diag.setName(m_tree->currentItem()->text(0));

    if(diag.exec())
    {
        diag.setName(m_tree->currentItem()->text(0));
    }
}
////////////////////////////////////////////////////////////////////////////////
void TreeView::editBldg()
{
    DialogEditBldg diag;
    diag.setName(m_tree->currentItem()->text(0));
    //diag.setEnvelope(0, 1, 0, 1);

    osg::ref_ptr<osg::Node> node = m_mainWindow->m_osgScene->findNode(m_tree->currentItem()->text(0).toStdString());

    if(node && node->asGeode())
    {
        //std::cout << "geode" << std::endl;
        node = node->getParent(0);
    }

    if(node)
        if(node->asTransform())
            if(node->asTransform()->asPositionAttitudeTransform())
            {
                osg::ref_ptr<osg::PositionAttitudeTransform> pos = node->asTransform()->asPositionAttitudeTransform();
                osg::Vec3d v = pos->getPosition();
                diag.setOffset(v.x(),v.y());
                //pos->setPosition(osg::Vec3d(x, y, 0));
                //std::cout << "pos : " << pos << std::endl;
            }


    //diag.setOffset(2, 2);

    if(diag.exec())
    {
        //diag.setName(m_tree->currentItem()->text(0));

        osg::ref_ptr<osg::Node> node = m_mainWindow->m_osgScene->findNode(m_tree->currentItem()->text(0).toStdString());

        if(node && node->asGeode())
        {
            //std::cout << "geode" << std::endl;
            node = node->getParent(0);
        }

        if(node)
            if(node->asTransform())
                if(node->asTransform()->asPositionAttitudeTransform())
                {
                    osg::ref_ptr<osg::PositionAttitudeTransform> pos = node->asTransform()->asPositionAttitudeTransform();
                    double x,y;
                    diag.getOffset(x,y);
                    pos->setPosition(osg::Vec3d(x, y, 0));
                    //std::cout << "pos : " << pos << std::endl;
                }


        //std::cout << node << std::endl;
    }
}
////////////////////////////////////////////////////////////////////////////////
 void TreeView::resetActions()
 {
     m_tree->removeAction(m_actionAddTile);
     m_tree->removeAction(m_actionEditTile);
     m_tree->removeAction(m_actionDeleteTile);
     m_tree->removeAction(m_actionAddLayer);
     m_tree->removeAction(m_actionEditLayer);
     m_tree->removeAction(m_actionDeleteLayer);
     m_tree->removeAction(m_actionAddBuilding);
     m_tree->removeAction(m_actionEditBuilding);
     m_tree->removeAction(m_actionDeleteBuilding);
     m_tree->removeAction(m_actionAddFlag);
     m_tree->removeAction(m_actionAddDynFlag);
     m_tree->removeAction(m_actionAddTag);
     m_tree->removeAction(m_actionEditFlag);
     m_tree->removeAction(m_actionEditTag);
     m_tree->removeAction(m_actionDeleteFlag);
     m_tree->removeAction(m_actionDeleteTag);

 }
////////////////////////////////////////////////////////////////////////////////
