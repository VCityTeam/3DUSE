////////////////////////////////////////////////////////////////////////////////
#include "moc/mainWindow.hpp"
#include "ui_mainWindow.h"
#include "ui_dialogLoadBBox.h"
#include "ui_dialogSettings.h"
#include "ui_dialogTag.h"
#include "ui_dialogFlag.h"
#include "ui_dialogDynFlag.h"

#include "controllerGui.hpp"

#include <QFileDialog>
#include <QCheckBox>
#include <QDirIterator>
#include <QSettings>
#include <QListView>
#include <QMessageBox>
#include <QDate>

#include <osg/PositionAttitudeTransform>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
//#include <osgShadow/ParallelSplitShadowMap>
#include <osgDB/Archive>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>

#include "citygml.hpp"
#include "export.hpp"

#include "gui/osg/osgScene.hpp"
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "osg/osgGDAL.hpp"
////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), m_ui(new Ui::MainWindow), m_useTemporal(false)
{
    m_ui->setupUi(this);

    // create Qt treeview
    m_treeView = new TreeView(m_ui->treeWidget, this);

    // create controller
    m_app.setController(new ControllerGui(&m_app));

    // create osgQt view widget
    m_osgView = new osgQtWidget(m_ui->mainGrid);
    //m_osgView = 0;
    m_pickhandler = new PickHandler();
    m_osgView->setPickHandler(m_pickhandler);
    m_ui->mainGridLayout->addWidget(m_osgView->getWidget(), 0, 0);

    // create osg scene
    m_osgScene = new OsgScene();

    // setup osgQt view
    m_osgView->setSceneData(m_osgScene);
    m_pickhandler->setPickHandlerTextBox(m_ui->textBrowser);
    m_pickhandler->setPickHandlerScene(&m_app.getScene());
    m_pickhandler->setPickHandlerTreeView(m_ui->treeWidget);

    // init gdal
    GDALAllRegister();
    OGRRegisterAll();

    // connect slots
    connect(m_ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));
    connect(m_ui->actionLoad, SIGNAL(triggered()), this, SLOT(loadScene()));
    connect(m_ui->actionLoad_recursive, SIGNAL(triggered()), this, SLOT(loadSceneRecursive()));
    //connect(m_ui->actionLoad_bbox, SIGNAL(triggered()), this, SLOT(loadSceneBBox()));
    connect(m_ui->actionExport_citygml, SIGNAL(triggered()), this, SLOT(exportCityGML()));
    connect(m_ui->actionExport_osg, SIGNAL(triggered()), this, SLOT(exportOsg()));
    connect(m_ui->actionExport_tiled_osga, SIGNAL(triggered()), this, SLOT(exportOsga()));
    connect(m_ui->actionExport_JSON, SIGNAL(triggered()), this, SLOT(exportJSON()));
    connect(m_ui->actionDelete_node, SIGNAL(triggered()), this, SLOT(deleteNode()));
    connect(m_ui->actionReset, SIGNAL(triggered()), this, SLOT(resetScene()));
    connect(m_ui->actionClearSelection, SIGNAL(triggered()), this, SLOT(clearSelection()));
    connect(m_ui->actionBuilding, SIGNAL(triggered()), this, SLOT(optionPickBuiling()));
    connect(m_ui->actionFace, SIGNAL(triggered()), this, SLOT(optionPickFace()));
    connect(m_ui->actionInfo_bubbles, SIGNAL(triggered()), this, SLOT(optionInfoBubbles()));
    connect(m_ui->actionShadows, SIGNAL(triggered()), this, SLOT(optionShadow()));
    connect(m_ui->actionSettings, SIGNAL(triggered()), this, SLOT(optionSettings()));
    connect(m_ui->actionAdd_Tag, SIGNAL(triggered()), this, SLOT(optionAddTag()));
    connect(m_ui->actionAdd_Flag, SIGNAL(triggered()), this, SLOT(optionAddFlag()));
    connect(m_ui->actionShow_temporal_tools, SIGNAL(triggered()), this, SLOT(optionShowTemporalTools()));
    connect(m_ui->checkBoxTemporalTools, SIGNAL(clicked()), this, SLOT(toggleUseTemporal()));
    connect(m_ui->actionShow_advanced_tools, SIGNAL(triggered()), this, SLOT(optionShowAdvancedTools()));
    connect(m_ui->treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(handleTreeView(QTreeWidgetItem*, int)));
    connect(m_ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateTemporalParams(int)));
    connect(m_ui->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(updateTemporalParams()));
    //connect(m_ui->buttonBrowserTemporal, SIGNAL(clicked()), this, SLOT(toggleUseTemporal()));
    connect(m_ui->actionDump_osg, SIGNAL(triggered()), this, SLOT(debugDumpOsg()));
    connect(m_ui->actionAbout, SIGNAL(triggered()), this, SLOT(about()));

    // LODs signals
    connect(m_ui->actionAll_LODs, SIGNAL(triggered()), this, SLOT(generateAllLODs()));
    connect(m_ui->actionLOD0, SIGNAL(triggered()), this, SLOT(generateLOD0()));
    connect(m_ui->actionLOD1, SIGNAL(triggered()), this, SLOT(generateLOD1()));
    connect(m_ui->actionLOD2, SIGNAL(triggered()), this, SLOT(generateLOD2()));
    connect(m_ui->actionLOD3, SIGNAL(triggered()), this, SLOT(generateLOD3()));
    connect(m_ui->actionLOD4, SIGNAL(triggered()), this, SLOT(generateLOD4()));

    connect(m_ui->actionTest_1, SIGNAL(triggered()), this, SLOT(test1()));
    connect(m_ui->actionTest_2, SIGNAL(triggered()), this, SLOT(test2()));
    connect(m_ui->actionTest_3, SIGNAL(triggered()), this, SLOT(test3()));
    connect(m_ui->actionTest_4, SIGNAL(triggered()), this, SLOT(test4()));
    connect(m_ui->actionTest_5, SIGNAL(triggered()), this, SLOT(test5()));

    m_ui->menuDebug->setVisible(false);

    reset();

    // set context menu
    //m_ui->treeWidget->setContextMenuPolicy(Qt::ActionsContextMenu);

    //QAction* select = new QAction("Select", NULL);
    //QAction* insertChild = new QAction("Insert child", NULL);
    //QAction* deleteNode = new QAction("Delete node", NULL);
    //QAction* showInfo = new QAction("Show info", NULL);

    //m_ui->treeWidget->addAction(select);
    //m_ui->treeWidget->addAction(insertChild);
    //m_ui->treeWidget->addAction(deleteNode);
    //m_ui->treeWidget->addAction(showInfo);

    //connect(select, SIGNAL(triggered()), this, SLOT(selectNodeHandler()));
    //connect(insertChild, SIGNAL(triggered()), this, SLOT(insertChildHandler()));
    //connect(deleteNode, SIGNAL(triggered()), this, SLOT(deleteNodeHandler()));
    //connect(showInfo, SIGNAL(triggered()), this, SLOT(showInfoHandler()));

    //connect(m_ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(selectNodeHandler(QTreeWidgetItem*, int)));

    updateRecentFiles();

    m_treeView->init();
}
////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow()
{
    delete m_osgView;
    delete m_ui;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::setOsgData(osg::Node* scene)
{
    m_osgScene->addChild(scene);
    //m_osgView->setSceneData(scene);

    // force camera recenter
    //m_osgView->m_osgView->getCamera()->
}
////////////////////////////////////////////////////////////////////////////////
void fillTreeViewRec(QTreeWidgetItem* parent, citygml::CityObject* node)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(parent, QStringList(node->getId().c_str()));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, Qt::Checked);
    item->setText(1, node->getTypeAsString().c_str());

    citygml::CityObjects& cityObjects = node->getChildren();
    //std::cout << "child size : " << cityObject.size() << std::endl;
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        fillTreeViewRec(item, *it);
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::fillTreeView(vcity::Tile* tile)
{
    // need fix !
    //disconnect(m_ui->treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(handleTreeView(QTreeWidgetItem*, int)));
    m_ui->treeWidget->blockSignals(true);

    /*std::vector<Tile*>& tiles = m_scene.getTiles();
    std::vector<Tile*>::iterator it = tiles.begin();
    for(; it != tiles.end(); ++it)
    {
        citygml::CityModel* citymodel = (*it)->getCityModel();

        citygml::CityObjectsMap& cityObjects = citymodel->getCityObjectsMap();
        std::cout << "map size : " << cityObjects.size() << std::endl;
        citygml::CityObjectsMap::iterator it = cityObjects.begin();
        for( ; it != cityObjects.end(); ++it)
        {
            citygml::CityObjects& cityObject = it->second;
            std::cout << "child size : " << cityObject.size() << std::endl;
            citygml::CityObjects::iterator itObj = cityObject.begin();
            for( ; itObj != cityObject.end(); ++itObj)
            {
                QTreeWidgetItem* a = new QTreeWidgetItem(m_ui->treeWidget, QStringList((*itObj)->getId().c_str()));
                a->setFlags(a->flags() | Qt::ItemIsUserCheckable);
                a->setCheckState(0, Qt::Checked);

                ++count;
            }
        }
    }*/

    //std::vector<Tile*>& tiles = m_scene.getTiles();
    //std::vector<Tile*>::iterator it = tiles.begin();
    //for(; it != tiles.end(); ++it)
    //{
        QTreeWidgetItem* root = m_ui->treeWidget->topLevelItem(0);
        //QTreeWidgetItem* root = 0;
        QTreeWidgetItem* layer0 = root->child(0);

        QTreeWidgetItem* item = new QTreeWidgetItem(layer0, QStringList(tile->getName().c_str()));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
        item->setText(1, "Tile");

        citygml::CityModel* citymodel = tile->getCityModel();
        citygml::CityObjects& cityObjects = citymodel->getCityObjectsRoots();
        //std::cout << "root size : " << cityObjects.size() << std::endl;
        citygml::CityObjects::iterator it = cityObjects.begin();
        for( ; it != cityObjects.end(); ++it)
        {
            fillTreeViewRec(item, *it);
        }
    //}

    m_ui->treeWidget->expandToDepth(0);

    // need fix !
    //connect(m_ui->treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(handleTreeView(QTreeWidgetItem*, int)));

    m_ui->treeWidget->blockSignals(false);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::handleTreeView(QTreeWidgetItem* item, int /*column*/)
{
    /*std::cout << "clic : " << item->text(0).toStdString() << "," << item->text(1).toStdString() << std::endl;

    std::vector<vcity::Tile*>& tiles = m_app.getScene().getTiles();
    std::vector<vcity::Tile*>::iterator it = tiles.begin();
    for(; it != tiles.end(); ++it)
    {
        citygml::CityObject* obj = (*it)->findNode(item->text(0).toStdString());
        if(obj != NULL)
        {
            osg::ref_ptr<osg::Node> node = obj->getOsgNode();
            std::cout << "found : " << item->text(0).toStdString() << " : " << node->getNodeMask() << std::endl;

            node->setNodeMask(0xffffffff - node->getNodeMask());
        }*/
        // check root
        /*if((*it)->getOsgRoot()->getName() == item->text(0).toStdString())
        {
            osg::ref_ptr<osg::Node> node = (*it)->getOsgRoot();
            std::cout << "found : " << item->text(0).toStdString() << " : " << node->getNodeMask() << std::endl;

            node->setNodeMask(0xffffffff - node->getNodeMask());
        }*/

        // else continue

        /*citygml::CityModel* citymodel = (*it)->getCityModel();

        citygml::CityObjectsMap& cityObjects = citymodel->getCityObjectsMap();
        citygml::CityObjectsMap::iterator it = cityObjects.begin();
        for( ; it != cityObjects.end(); ++it)
        {
            citygml::CityObjects& cityObject = it->second;
            citygml::CityObjects::iterator itObj = cityObject.begin();
            for( ; itObj != cityObject.end(); ++itObj)
            {
                if((*itObj)->getId() == item->text(0).toStdString())
                {
                    osg::ref_ptr<osg::Node> node = (*itObj)->getOsgNode();
                    std::cout << "found : " << item->text(0).toStdString() << " : " << node->getNodeMask() << std::endl;

                    node->setNodeMask(0xffffffff - node->getNodeMask());
                }
            }
        }*/
    //}

    // test node mask
    //osg::ref_ptr<osg::Node> node = (*it)->getOsgNode();
    //node->setNodeMask(0);*/
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::selectNodeHandler()
{
    std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::selectNodeHandler(QTreeWidgetItem* item, int /*column*/)
{
    /*std::cout << "select node : " << item->text(0).toStdString() << "," << item->text(1).toStdString() << std::endl;
    citygml::CityObject* obj = m_app.getScene().findNode(item->text(0).toStdString());

    if(obj)
    {
        m_pickhandler->setLabel(item->text(0).toStdString());
    }*/


    /*std::string uri = item->text(0).toStdString();
    QTreeWidgetItem* parent = item;
    while((parent = parent->parent()) != NULL)
    {
        uri.insert(0, parent->text(0).toStdString()+'.');
        //URI += parent->text(0).toStdString();
    }
    uri.insert(0, item->text(1).toStdString()+':');*/

    /*std::stringstream ss;

    std::string type = item->text(1).toStdString();
    if(type == "Root")
    {
        ss << "Root" << std::endl;
    }
    else if(type == "Layer")
    {
        ss << "Layer : " << item->text(0).toStdString() << std::endl;
    }
    else if(type == "Tile")
    {
        ss << "Tile : " << item->text(0).toStdString() << std::endl;
    }
    else if(type == "Building")
    {
        ss << "Building : " << item->text(0).toStdString() << std::endl;
    }
    else if(type == "TAG")
    {
        ss << "TAG : " << item->text(0).toStdString() << std::endl;
    }
    else if(type == "FLAG")
    {
        ss << "FLAG : " << item->text(0).toStdString() << std::endl;
    }
    else
    {
    }

    //ss << "URI : " << uri << std::endl;

    vcity::URI uri = m_treeView->getURI(item);
    std::cout << uri.getStringURI() << std::endl;

    m_pickhandler->setLabel(ss.str());*/
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::insertChildHandler()
{
    std::cout << "insert node under : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::deleteNodeHandler()
{
    std::cout << "delete node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::showInfoHandler()
{
    std::cout << "node info : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << "," << m_ui->treeWidget->currentItem()->text(1).toStdString() << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::deleteNode()
{
   /* std::string name = m_osgView->getNodePicked();

    //m_scene

    std::vector<Tile*>& tiles = m_scene.getTiles();
    std::vector<Tile*>::iterator it = tiles.begin();
    for(; it != tiles.end(); ++it)
    {
        // check root
        if((*it)->getOsgRoot()->getName() == name)
        {
            osg::ref_ptr<osg::Node> node = (*it)->getOsgRoot();
            //std::cout << "found : " << item->text(0).toStdString() << " : " << node->getNodeMask() << std::endl;

            node->setNodeMask(0xffffffff - node->getNodeMask());
            //node->clear();
            osg::ref_ptr<osg::Group> grp = node->asGroup();
            std::vector<osg::Node*> nodes;
            for(unsigned int i=0; i<grp->getNumChildren(); ++i)
            {
                nodes.push_back(grp->getChild(i));
            }
            std::vector<osg::Node*>::iterator it = nodes.begin();
            for(;it != nodes.end(); ++it)
            {
                grp->removeChild(*it);
            }
        }

        // else continue

        citygml::CityModel* citymodel = (*it)->getCityModel();

        citygml::CityObjectsMap& cityObjects = citymodel->getCityObjectsMap();
        citygml::CityObjectsMap::iterator it = cityObjects.begin();
        for( ; it != cityObjects.end(); ++it)
        {
            citygml::CityObjects& cityObject = it->second;
            citygml::CityObjects::iterator itObj = cityObject.begin();
            for( ; itObj != cityObject.end(); ++itObj)
            {
                if((*itObj)->getId() == name)
                {
                    osg::ref_ptr<osg::Node> node = (*itObj)->getOsgNode();
                    //std::cout << "found : " << item->text(0).toStdString() << " : " << node->getNodeMask() << std::endl;

                    node->setNodeMask(0xffffffff - node->getNodeMask());

                    osg::ref_ptr<osg::Group> grp = node->asGroup();
                    std::vector<osg::Node*> nodes;
                    for(unsigned int i=0; i<grp->getNumChildren(); ++i)
                    {
                        nodes.push_back(grp->getChild(i));
                    }
                    std::vector<osg::Node*>::iterator it = nodes.begin();
                    for(;it != nodes.end(); ++it)
                    {
                        grp->removeChild(*it);
                    }
                }
            }
        }
    }*/
}
////////////////////////////////////////////////////////////////////////////////
// Recent files
////////////////////////////////////////////////////////////////////////////////
void MainWindow::addRecentFile(const QString& filepath)
{
    QSettings settings("liris", "virtualcity");
    QStringList list = settings.value("recentfiles").toStringList();
    list.insert(0, filepath);

    if(!list.contains(filepath, Qt::CaseInsensitive))
    {
        if(list.size() >= 10)
        {
            list.insert(0, filepath);
            list.removeLast();
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateRecentFiles()
{
    QSettings settings("liris", "virtualcity");
    QStringList list = settings.value("recentfiles").toStringList();
    list = list.toSet().toList();
    settings.setValue("recentfiles", list);

    clearRecentFiles();

    foreach(QString str, list)
    {
        QAction* action = new QAction(str, this);
        m_ui->menuRecent_files->addAction(action);
        connect(action, SIGNAL(triggered()), this, SLOT(openRecentFile()));
        action->setData(str);
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::clearRecentFiles(bool removeAll)
{
    m_ui->menuRecent_files->clear();
    QAction* action = new QAction("Clear", this);
    m_ui->menuRecent_files->addAction(action);
    connect(action, SIGNAL(triggered()), this, SLOT(clearRecentFiles()));

    if(removeAll)
    {
        QSettings settings("liris", "virtualcity");
        settings.setValue("recentfiles", QStringList());
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::openRecentFile()
{
    QAction* action = qobject_cast<QAction*>(sender());
    if(action)
        loadFile(action->data().toString());
}
////////////////////////////////////////////////////////////////////////////////
// Open files
////////////////////////////////////////////////////////////////////////////////
bool MainWindow::loadFile(const QString& filepath)
{
    QSettings settings("liris", "virtualcity");
    QFileInfo file(filepath);
    QString ext = file.suffix().toLower();
    if(ext == "citygml" || ext == "gml")
    {
        std::cout << "load citygml file : " << filepath.toStdString() << std::endl;

        // add tile
        vcity::Tile* tile = new vcity::Tile(filepath.toStdString());
        m_app.getScene().getLayers()[0]->addTile(tile);
        m_osgScene->addTile(*tile);
        m_osgView->centerCamera();


        //setOsgData(buildOsgScene(*tile));
        fillTreeView(tile);

         QStringList list = settings.value("recentfiles").toStringList();
         list.append(filepath);
         settings.setValue("recentfiles", list);
    }
    else if(ext == "shp")
    {
        std::cout << "load shp file : " << filepath.toStdString() << std::endl;
        OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);

        m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));


        /*if(poDS)
        {
            OGRLayer *poLayer;
            int nbLayers = poDS->GetLayerCount();
            printf("%d layer\n", nbLayers);
            if(nbLayers > 0)
            {
                poLayer = poDS->GetLayer(0);
                printf("layer %s\n", poLayer->GetName());
                //poLayer = poDS->GetLayerByName( "point" );

                OGRFeature *poFeature;

                poLayer->ResetReading();
                while( (poFeature = poLayer->GetNextFeature()) != NULL )
                {
                    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
                    int iField;

                    for( iField = 0; iField < poFDefn->GetFieldCount(); iField++ )
                    {
                        OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );

                        if( poFieldDefn->GetType() == OFTInteger )
                            printf( "%d,", poFeature->GetFieldAsInteger( iField ) );
                        else if( poFieldDefn->GetType() == OFTReal )
                            printf( "%.3f,", poFeature->GetFieldAsDouble(iField) );
                        else if( poFieldDefn->GetType() == OFTString )
                            printf( "%s,", poFeature->GetFieldAsString(iField) );
                        else
                            printf( "%s,", poFeature->GetFieldAsString(iField) );
                    }

                    OGRGeometry *poGeometry;

                    poGeometry = poFeature->GetGeometryRef();
                    if(poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbPoint)
                    {
                        OGRPoint *poPoint = (OGRPoint *) poGeometry;
                        printf( "%.3f,%3.f\n", poPoint->getX(), poPoint->getY() );
                    }
                    else
                    {
                        printf( "%ud %s no point geometry\n", poGeometry->getGeometryType(), poGeometry->getGeometryName() );
                    }
                    OGRFeature::DestroyFeature( poFeature );
                }

                OGRDataSource::DestroyDataSource( poDS );

                QStringList list = settings.value("recentfiles").toStringList();
                list.append(filepath);
                settings.setValue("recentfiles", list);
            }
        }*/
    }
    else if(ext == "dxf")
    {
        std::cout << "load dxf file : " << filepath.toStdString() << std::endl;
        OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);

        m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));
    }
    else if(ext == "ecw")
    {
        std::cout << "load ecw file : " << filepath.toStdString() << std::endl;
        GDALDataset* poDataset = (GDALDataset*) GDALOpen( filepath.toStdString().c_str(), GA_ReadOnly );
        if( poDataset == NULL )
        {
            std::cout << "load ecw file : " << filepath.toStdString() << std::endl;
        }
    }
    else
    {
        std::cout << "bad file : " << filepath.toStdString() << std::endl;
        return false;
    }

    return true;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadScene()
{
    std::cout<<"Load Scene"<<std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(0, "Load scene files", lastdir);

    for(int i = 0; i < filenames.count(); ++i)
    {
        QFileInfo file(filenames[i]);
        bool success = loadFile(file.absoluteFilePath());
        if(success)
        {
            // save path
            QFileInfo file(filenames[i]);
            //std::cout << "lastdir : " << file.dir().absolutePath().toStdString() << std::endl;
            settings.setValue("lastdir", file.dir().absolutePath());
        }
    }
    //std::cout << "lastdir set : " << settings.value("lastdir").toString().toStdString() << std::endl;

    updateRecentFiles();
}
////////////////////////////////////////////////////////////////////////////////
void buildRecursiveFileList(const QDir& dir, QStringList& list)
{
    QDirIterator iterator(dir.absolutePath(), QDirIterator::Subdirectories);
    while(iterator.hasNext())
    {
        iterator.next();
        if(!iterator.fileInfo().isDir())
        {
            QString filename = iterator.filePath();
            if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
            {
                list.append(filename);
                qDebug("Found %s matching pattern.", qPrintable(filename));
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadSceneRecursive()
{
    std::cout<<"Load Scene recursive"<<std::endl;
    /*QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();

    QString dirpath = QFileDialog::getExistingDirectory(0, "Load scene files : choose a directory", lastdir, 0);
    //std::cout << "dir : " << dirpath.toStdString() << std::endl;
    if(dirpath != "")
    {
        QDir dir(dirpath);
        QStringList files;
        buildRecursiveFileList(dir, files);

        for(int i = 0; i < files.count(); ++i)
        {
            loadFile(files[i]);
        }

        settings.setValue("lastdir", dirpath);
    }

    //std::cout << "lastdir set : " << settings.value("lastdir").toString().toStdString() << std::endl;

    updateRecentFiles();*/


    QFileDialog w;
    w.setFileMode(QFileDialog::Directory);
    //w.setFileMode(QFileDialog::AnyFile);
    //w.setOption(QFileDialog::DontUseNativeDialog,false);
    QStringList filters;
    filters.append("Any type (*.gml *.citygml *.xml *.shp)");
    filters.append("Citygml files (*.gml *.citygml)");
    filters.append("Xml files (*.xml)");
    filters.append("Shape files (*.shp)");
    filters.append("Any files (*)");
    w.setNameFilters(filters);
    /*QListView *l = w.findChild<QListView*>("listView");
    if(l)
    {
        l->setSelectionMode(QAbstractItemView::MultiSelection);
    }*/
    QTreeView *t = w.findChild<QTreeView*>();
    if(t)
    {
        t->setSelectionMode(QAbstractItemView::MultiSelection);
    }
    w.exec();
    QStringList file = w.selectedFiles();
    foreach (QString s, file)
    {
        QFileInfo file(s);
        // do some useful stuff here
        std::cout << "Working on file " << s.toStdString() << " type : " << file.isDir()<< ", " << file.isFile() << std::endl;
        if(file.isDir())
        {
            QDir dir(s);
            QStringList files;
            buildRecursiveFileList(dir, files);

            for(int i = 0; i < files.count(); ++i)
            {
                loadFile(files[i]);
            }
        }
        else if(file.isFile())
        {
            loadFile(s);
        }
        else
        {
            // fail
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadSceneBBox()
{
    Ui::DialogLoadBBox ui;
    QDialog diag;
    ui.setupUi(&diag);
    diag.exec();
}
////////////////////////////////////////////////////////////////////////////////
// Tools
////////////////////////////////////////////////////////////////////////////////
void MainWindow::reset()
{
    // reset treeview
    /*m_ui->treeWidget->clear();

    // add root element
    QTreeWidgetItem* item = new QTreeWidgetItem(m_ui->treeWidget, QStringList("root"));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, Qt::Checked);
    item->setText(1, "Root");

    //item = 0;
    //item->setText(1, "new");

    QTreeWidgetItem* itemlayer = new QTreeWidgetItem(item, QStringList("layer0"));
    itemlayer->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    itemlayer->setCheckState(0, Qt::Checked);
    itemlayer->setText(1, "Layer");*/

    // reset text box
    m_ui->textBrowser->clear();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::resetScene()
{
    // reset scene
    m_app.getScene().reset();

    // reset osg scene
    m_osgScene->reset();

    // reset ui
    reset();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::clearSelection()
{
    m_pickhandler->resetPicking();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionPickBuiling()
{
    m_ui->actionFace->setChecked(false);
    m_ui->actionBuilding->setChecked(true);

    std::cout << "pick building" << std::endl;
    m_osgView->getPickHandler()->setPickingMode(1);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionPickFace()
{
    m_ui->actionBuilding->setChecked(false);
    m_ui->actionFace->setChecked(true);

    std::cout << "pick face" << std::endl;
    m_osgView->getPickHandler()->setPickingMode(0);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionInfoBubbles()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionShadow()
{
    bool v = m_ui->actionShadows->isChecked();
    m_osgScene->setShadow(v);

    std::cout << "toggle shadow" << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionSettings()
{
    Ui::DialogSettings ui;
    QDialog diag;
    ui.setupUi(&diag);

    ui.lineEditLowerBoundX->setText("0");
    ui.lineEditLowerBoundY->setText("0");
    ui.lineEditUpperBoundX->setText("0");
    ui.lineEditUpperBoundY->setText("0");

    ui.lineTileSizeX->setText("500");
    ui.lineTileSizeY->setText("500");

    diag.exec();

    m_app.getDataProfile().m_bboxLowerBound = TVec3d(ui.lineEditLowerBoundX->text().toDouble(), ui.lineEditLowerBoundY->text().toDouble());
    m_app.getDataProfile().m_bboxUpperBound = TVec3d(ui.lineEditUpperBoundX->text().toDouble(), ui.lineEditUpperBoundY->text().toDouble());

    m_app.getDataProfile().m_xStep = ui.lineTileSizeX->text().toFloat();
    m_app.getDataProfile().m_yStep = ui.lineTileSizeY->text().toFloat();
}
////////////////////////////////////////////////////////////////////////////////
#include "libcitygml/readerOsgCityGML.hpp"
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
void MainWindow::optionAddTag()
{
//    Ui::DialogTag ui;
//    QDialog diag;
//    ui.setupUi(&diag);
//    citygml::CityObject* obj = 0;

//    std::map<std::string, citygml::CityObject*> geoms;

//    if(m_ui->treeWidget->currentItem())
//    {
//        std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
//        obj = m_app.getScene().findNode(m_ui->treeWidget->currentItem()->text(0).toStdString());

//        if(obj)
//        {
//            // add lod
//            ui.comboBox->addItem(m_ui->treeWidget->currentItem()->text(0));
//            geoms[m_ui->treeWidget->currentItem()->text(0).toStdString()] = 0;

//            // add all bldg


//            // add flags
//            std::vector<citygml::BuildingFlag*>::const_iterator it = obj->getFlags().begin();
//            for(; it < obj->getFlags().end(); ++it)
//            {
//                ui.comboBox->addItem(QString(((*it)->getStringId()).c_str()));
//                geoms[(*it)->getStringId()] = (*it)->getGeom();
//            }


//        }
//        ui.comboBox->addItem("NULL");
//        geoms["NULL"] = 0;
//    }

//    int res = diag.exec();

//    //std::cout << "diag res : " << res << std::endl;

//    if(res && obj && m_ui->treeWidget->currentItem())
//    {
//        citygml::CityObject* geom = 0;
//        if(ui.comboBox->currentText().size() > 4 && ui.comboBox->currentText().left(4) == "FLAG")
//        {
//            citygml::BuildingFlag* f = obj->getFlag(ui.comboBox->currentText().toStdString());
//            if(f) geom = f->getGeom();
//            std::cout << "use flag geom : " << ui.comboBox->currentText().toStdString() << " : " << f << " : " << geom << std::endl;
//        }
//        else if(ui.comboBox->currentText() != "NULL")
//        {
//            // use existing
//            geom = m_app.getScene().findNode(ui.comboBox->currentText().toStdString());
//            std::cout << "use existing : " << geom << std::endl;
//        }

//        citygml::BuildingTag* tag = new citygml::BuildingTag(ui.dateTimeEdit->date().year(), geom);
//        tag->m_date = ui.dateTimeEdit->dateTime();
//        tag->m_name = ui.lineEdit->text().toStdString();
//        tag->m_parent = obj;
//        //tag->m_year = ui.dateTimeEdit->date().year();


//        if(geom)
//        {
//            if(obj->getTags().size() == 0)
//            {
//                // mark osg tagged
//                obj->getOsgNode()->getChild(0)->setUserValue("TAGGED", 1);
//                //obj->getOsgNode()->setUserValue("TAGGED", 1);
//                std::cout << "osg parent tagged" << std::endl;
//            }

//            std::string path = ".";
//            /*if(geom->m_path != "")
//            {
//                path = geom->m_path;
//                size_t pos = path.find_last_of("/\\");
//                path = path.substr(0, pos);
//            }*/
//            //size_t pos = filename.toStdString().find_last_of("/\\");
//            //std::string path = filename.toStdString().substr(0, pos);
//            ReaderOsgCityGML readerOsgGml(path);
//            osg::ref_ptr<osg::Group> grp = readerOsgGml.createCityObject(geom);

//            citygml::CityObjects& cityObjects = geom->getChildren();
//            citygml::CityObjects::iterator it = cityObjects.begin();
//            for( ; it != cityObjects.end(); ++it)
//            {
//                loadRecTest(*it, grp, readerOsgGml);
//            }

//            grp->setName(tag->getStringId()+tag->getGeom()->getId());
//            grp->getChild(0)->setName(tag->getStringId()+tag->getGeom()->getId());
//            grp->setUserValue("TAG", 1);

//            std::cout << "insert osg geom" << std::endl;
//            //obj->getOsgNode()->addChild(grp);
//            obj->getOsgNode()->getParent(0)->addChild(grp);
//            geom->setOsgNode(grp);
//            //m_osgScene->addChild(grp);
//            //obj->getOsgNode()->setNodeMask(0);
//        }

//        obj->addTag(tag);

//        QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(tag->getStringId().c_str()));
//        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
//        item->setCheckState(0, Qt::Checked);
//        item->setText(1, "Tag");

//        QTreeWidgetItem* item2 = new QTreeWidgetItem(QStringList(ui.comboBox->currentText()));
//        item->addChild(item2);

//        obj->checkTags();

//        m_ui->treeWidget->currentItem()->addChild(item);
//    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionAddFlag()
{
//    Ui::DialogFlag ui;
//    QDialog diag;
//    ui.setupUi(&diag);
//    citygml::CityObject* obj = 0;

//    if(m_ui->treeWidget->currentItem())
//    {
//        std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
//        obj = m_app.getScene().findNode(m_ui->treeWidget->currentItem()->text(0).toStdString());

//        if(obj)
//        {
//            // add lod
//            ui.comboBox->addItem(m_ui->treeWidget->currentItem()->text(0));
//        }
//        ui.comboBox->addItem("NULL");
//        ui.comboBox->addItem("NEW");
//    }

//    int res = diag.exec();

//    //std::cout << "diag res : " << res << std::endl;

//    if(res && obj && m_ui->treeWidget->currentItem())
//    {
//        citygml::CityObject* geom = 0;
//        std::cout << ui.comboBox->currentText().toStdString() << std::endl;

//        QString item2text;

//        if(ui.comboBox->currentText() == "NEW")
//        {
//            // load
//            std::cout << "load new" << std::endl;

//            QSettings settings("liris", "virtualcity");
//            QString lastdir = settings.value("lastdir").toString();
//            QString filename = QFileDialog::getOpenFileName(0, "Load scene file", lastdir);
//            citygml::ParserParams params;
//            citygml::CityModel* mdl = citygml::load(filename.toStdString(), params);
//            citygml::CityObject* bldg = mdl->getCityObjectsRoots()[0];
//            geom = bldg;
//            geom->m_path = filename.toStdString();
//            std::cout << "nb : " << mdl->getCityObjectsRoots().size()<< std::endl;

//            // create osg geometry
//            /*size_t pos = filename.toStdString().find_last_of("/\\");
//            std::string path = filename.toStdString().substr(0, pos);
//            ReaderOsgCityGML readerOsgGml(path);

//            osg::ref_ptr<osg::Group> grp = readerOsgGml.createCityObject(bldg);

//            if(bldg->getType() == citygml::COT_Building)
//            {
//                int yearOfConstruction;
//                int yearOfDemolition;

//                std::istringstream(bldg->getAttribute("yearOfConstruction")) >> yearOfConstruction;
//                std::istringstream(bldg->getAttribute("yearOfDemolition")) >> yearOfDemolition;

//                grp->setUserValue("yearOfConstruction", yearOfConstruction);
//                grp->setUserValue("yearOfDemolition", yearOfDemolition);
//            }

//            bldg->setOsgNode(grp);*/
//            item2text = bldg->getId().c_str();
//        }
//        else if(ui.comboBox->currentText() == "NULL")
//        {
//            geom = NULL;
//            item2text = "NULL";
//        }
//        else
//        {
//            // use existing
//            geom = m_app.getScene().findNode(ui.comboBox->currentText().toStdString());
//            std::cout << "use existing : " << geom << std::endl;
//            item2text = ui.comboBox->currentText();
//        }

//        citygml::BuildingFlag* flag = new citygml::BuildingFlag(geom);
//        flag->m_name = ui.lineEdit->text().toStdString();
//        flag->m_parent = obj;
//        obj->addFlag(flag);

//        QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(flag->getStringId().c_str()));
//        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
//        item->setCheckState(0, Qt::Checked);
//        item->setText(1, "Flag");

//        QTreeWidgetItem* item2 = new QTreeWidgetItem(QStringList(item2text));
//        item->addChild(item2);

//        m_ui->treeWidget->currentItem()->addChild(item);
//    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionAddDynFlag()
{
//    Ui::DialogDynFlag ui;
//    QDialog diag;
//    ui.setupUi(&diag);
//    citygml::CityObject* obj = 0;

//    if(m_ui->treeWidget->currentItem())
//    {
//        std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
//        obj = m_app.getScene().findNode(m_ui->treeWidget->currentItem()->text(0).toStdString());

//        if(obj)
//        {
//            // add lod
//            ui.comboBox->addItem(m_ui->treeWidget->currentItem()->text(0));
//        }
//        ui.comboBox->addItem("NULL");
//        ui.comboBox->addItem("NEW");
//    }

//    int res = diag.exec();

//    //std::cout << "diag res : " << res << std::endl;

//    if(res && obj && m_ui->treeWidget->currentItem())
//    {
//        citygml::CityObject* geom = 0;
//        std::cout << ui.comboBox->currentText().toStdString() << std::endl;

//        QString item2text;

//        if(ui.comboBox->currentText() == "NEW")
//        {
//            // load
//            std::cout << "load new" << std::endl;

//            QSettings settings("liris", "virtualcity");
//            QString lastdir = settings.value("lastdir").toString();
//            QString filename = QFileDialog::getOpenFileName(0, "Load scene file", lastdir);
//            citygml::ParserParams params;
//            citygml::CityModel* mdl = citygml::load(filename.toStdString(), params);
//            citygml::CityObject* bldg = mdl->getCityObjectsRoots()[0];
//            geom = bldg;
//            geom->m_path = filename.toStdString();
//            std::cout << "nb : " << mdl->getCityObjectsRoots().size()<< std::endl;

//            // create osg geometry
//            /*size_t pos = filename.toStdString().find_last_of("/\\");
//            std::string path = filename.toStdString().substr(0, pos);
//            ReaderOsgCityGML readerOsgGml(path);

//            osg::ref_ptr<osg::Group> grp = readerOsgGml.createCityObject(bldg);

//            if(bldg->getType() == citygml::COT_Building)
//            {
//                int yearOfConstruction;
//                int yearOfDemolition;

//                std::istringstream(bldg->getAttribute("yearOfConstruction")) >> yearOfConstruction;
//                std::istringstream(bldg->getAttribute("yearOfDemolition")) >> yearOfDemolition;

//                grp->setUserValue("yearOfConstruction", yearOfConstruction);
//                grp->setUserValue("yearOfDemolition", yearOfDemolition);
//            }

//            bldg->setOsgNode(grp);*/
//            item2text = bldg->getId().c_str();
//        }
//        else if(ui.comboBox->currentText() == "NULL")
//        {
//            geom = NULL;
//            item2text = "NULL";
//        }
//        else
//        {
//            // use existing
//            geom = m_app.getScene().findNode(ui.comboBox->currentText().toStdString());
//            std::cout << "use existing : " << geom << std::endl;
//            item2text = ui.comboBox->currentText();
//        }

//        citygml::BuildingFlag* flag = new citygml::BuildingFlag(geom);
//        flag->m_name = ui.lineEdit->text().toStdString();
//        flag->m_parent = obj;
//        obj->addFlag(flag);

//        QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(flag->getStringId().c_str()));
//        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
//        item->setCheckState(0, Qt::Checked);
//        item->setText(1, "Flag");

//        QTreeWidgetItem* item2 = new QTreeWidgetItem(QStringList(item2text));
//        item->addChild(item2);

//        m_ui->treeWidget->currentItem()->addChild(item);
//    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionShowTemporalTools()
{
    if(m_ui->actionShow_temporal_tools->isChecked())
    {
        m_ui->hsplitter_bottom->show();
    }
    else
    {
        m_ui->hsplitter_bottom->hide();
    }

    std::cout << "show temporal tools" << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::optionShowAdvancedTools()
{
    bool v = m_ui->actionShow_advanced_tools->isChecked();
    if(v)
    {
        m_ui->menuDebug->show();
    }
    else
    {
        m_ui->menuDebug->hide();
    }
}
////////////////////////////////////////////////////////////////////////////////
/*void MainWindow::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    //m_ui->widget->resize(event->size());
    //m_ui->widget->setGeometry(0, 0, event->size().width(), event->size().height());
    std::cout << "resize main window" << std::endl;
}*/
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateTemporalParams(int value)
{
    if(value == -1) value = m_ui->horizontalSlider->value();
    QDate date(1900, 1, 1);
    date = date.addDays(value);
    //m_ui->buttonBrowserTemporal->setText(date.toString());
    m_ui->dateTimeEdit->setDate(date);

    //std::cout << "set year : " << date.year() << std::endl;

    if(m_useTemporal)   m_osgScene->setYear(date.year());
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::toggleUseTemporal()
{
    m_useTemporal = !m_useTemporal;

    if(m_useTemporal)
    {
        QDate date(1900, 1, 1);
        date = date.addDays(m_ui->horizontalSlider->value());
        m_osgScene->setYear(date.year());
    }
    else
    {
        m_osgScene->setYear(-1); // reset
    }

    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);

    std::cout << "toggle temporal tool" << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportCityGML()
{
    QString filename = QFileDialog::getSaveFileName();

    citygml::Exporter exporter;

    // check temporal params
    if(m_useTemporal)
    {
        exporter.setTemporalExport(true);
        exporter.setDate(m_ui->dateTimeEdit->dateTime());
    }

    // check if something is picked
    const std::set<std::string>& nodes = m_pickhandler->getNodesPicked();
    if(nodes.size() > 0)
    {
        std::cout << "Citygml export cityobject : " << *nodes.begin() << std::endl;
        // use first node picked
        citygml::CityObject* model = m_app.getScene().getDefaultLayer()->getTiles()[0]->findNode(*nodes.begin());
        //citygml::exportCitygml(model, "test.citygml");
        if(model) exporter.exportCityObject(model, filename.toStdString());
    }
    else
    {
        std::cout << "Citygml export citymodel" << std::endl;
        // use first tile
        citygml::CityModel* model = m_app.getScene().getDefaultLayer()->getTiles()[0]->getCityModel();
        //citygml::exportCitygml(model, "test.citygml");
        exporter.exportCityModel(model, filename.toStdString());
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportOsg()
{
    osg::ref_ptr<osg::Node> node = m_osgScene;
    bool res = osgDB::writeNodeFile(*node, "scene.osg");
    std::cout << "export osg : " << res << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportOsga()
{
    osg::ref_ptr<osg::Node> node = m_osgScene;

    //osg::ref_ptr<osgDB::Archive> archive = osgDB::openArchive("scene.osga", osgDB::Archive::CREATE);
    //osg::ref_ptr<osgDB::ReaderWriter> rw =
    //osg::ref_ptr<osgDB::ReaderWriter::ReadResult> res = osgDB::ReaderWriter::openArchive("scene.osga", osgDB::ReaderWriter::CREATE);
    //osg::ref_ptr<osgDB::Archive> archive = res->getArchive();
    osgDB::writeNodeFile(*node, "scene.osga");

    /*if(archive.valid())
    {
        archive->writeNode(*node, "none");
    }
    archive->close();*/
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportJSON()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::debugDumpOsg()
{
    m_osgScene->dump();
    m_app.getScene().dump();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateAllLODs()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD0()
{
    // call real function. In algo.cpp ?
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD1()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD2()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD3()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD4()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::about()
{
    QMessageBox::about(this, "VCity", "VCity is an environment editor");
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test1()
{
    loadFile("/home/maxime/docs/data/dd_gilles/IGN_Data/dpt_75/BDTOPO-75/BDTOPO/1_DONNEES_LIVRAISON_2011-12-00477/BDT_2-1_SHP_LAMB93_D075-ED113/E_BATI/BATI_INDIFFERENCIE.SHP");
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test2()
{
    //loadFile("/home/maxime/docs/data/dd_gilles/IGN_Data/dpt_75/BDTOPO-75/BDTOPO/1_DONNEES_LIVRAISON_2011-12-00477/BDT_2-1_SHP_LAMB93_D075-ED113/E_BATI/BATI_INDIFFERENCIE.SHP");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13725/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13724/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13725/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13724/export-CityGML/ZoneAExporter.gml");
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test3()
{
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13728/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13728/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13727/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1297-13727/export-CityGML/ZoneAExporter.gml");
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test4()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test5()
{

}
////////////////////////////////////////////////////////////////////////////////
