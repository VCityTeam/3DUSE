////////////////////////////////////////////////////////////////////////////////
#include "moc/mainWindow.hpp"
#include "ui_mainWindow.h"
#include "ui_dialogLoadBBox.h"
#include "ui_dialogSettings.h"
#include "ui_dialogTag.h"
#include "ui_dialogFlag.h"
#include "ui_dialogDynFlag.h"
#include "moc/dialogSettings.hpp"

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
    QMainWindow(parent), m_ui(new Ui::MainWindow), m_useTemporal(false), m_temporalAnim(false)
{
    m_ui->setupUi(this);

    m_app.setMainWindow(this);

    // create Qt treeview
    m_treeView = new TreeView(m_ui->treeWidget, this);
    m_app.setTreeView(m_treeView);
    m_app.setTextBowser(m_ui->textBrowser);

    // create controller
    m_app.setControllerGui(new ControllerGui());

    // create osgQt view widget
    m_osgView = new osgQtWidget(m_ui->mainGrid);
    m_pickhandler = new PickHandler();
    m_osgView->setPickHandler(m_pickhandler);
    m_ui->mainGridLayout->addWidget(m_osgView->getWidget(), 0, 0);

    // create osg scene
    m_osgScene = new OsgScene();
    m_app.setOsgScene(m_osgScene);

    // setup osgQt view
    m_osgView->setSceneData(m_osgScene);

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
    //connect(m_ui->actionDelete_node, SIGNAL(triggered()), this, SLOT(deleteNode()));
    connect(m_ui->actionReset, SIGNAL(triggered()), this, SLOT(resetScene()));
    connect(m_ui->actionClearSelection, SIGNAL(triggered()), this, SLOT(clearSelection()));
    connect(m_ui->actionBuilding, SIGNAL(triggered()), this, SLOT(optionPickBuiling()));
    connect(m_ui->actionFace, SIGNAL(triggered()), this, SLOT(optionPickFace()));
    connect(m_ui->actionInfo_bubbles, SIGNAL(triggered()), this, SLOT(optionInfoBubbles()));
    connect(m_ui->actionShadows, SIGNAL(triggered()), this, SLOT(optionShadow()));
    connect(m_ui->actionSettings, SIGNAL(triggered()), this, SLOT(slotSettings()));
    //connect(m_ui->actionAdd_Tag, SIGNAL(triggered()), this, SLOT(optionAddTag()));
    //connect(m_ui->actionAdd_Flag, SIGNAL(triggered()), this, SLOT(optionAddFlag()));
    connect(m_ui->actionShow_temporal_tools, SIGNAL(triggered()), this, SLOT(optionShowTemporalTools()));
    connect(m_ui->checkBoxTemporalTools, SIGNAL(clicked()), this, SLOT(toggleUseTemporal()));
    connect(m_ui->actionShow_advanced_tools, SIGNAL(triggered()), this, SLOT(optionShowAdvancedTools()));
    //connect(m_ui->treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(handleTreeView(QTreeWidgetItem*, int)));
    connect(m_ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateTemporalParams(int)));
    connect(m_ui->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(updateTemporalParams()));
    //connect(m_ui->buttonBrowserTemporal, SIGNAL(clicked()), this, SLOT(toggleUseTemporal()));
    connect(m_ui->actionDump_osg, SIGNAL(triggered()), this, SLOT(debugDumpOsg()));
    connect(m_ui->actionDump_scene, SIGNAL(triggered()), this, SLOT(slotDumpScene()));
    connect(m_ui->actionDump_selected_nodes, SIGNAL(triggered()), this, SLOT(slotDumpSelectedNodes()));
    connect(m_ui->actionAbout, SIGNAL(triggered()), this, SLOT(about()));
    connect(m_ui->actionOptim_osg, SIGNAL(triggered()), this, SLOT(slotOptimOSG()));

    connect(m_ui->toolButton, SIGNAL(clicked()), this, SLOT(slotTemporalAnim()));

    // render lod signals
    connect(m_ui->actionForce_LOD0, SIGNAL(triggered()), this, SLOT(slotRenderLOD0()));
    connect(m_ui->actionForce_LOD1, SIGNAL(triggered()), this, SLOT(slotRenderLOD1()));
    connect(m_ui->actionForce_LOD2, SIGNAL(triggered()), this, SLOT(slotRenderLOD2()));
    connect(m_ui->actionForce_LOD3, SIGNAL(triggered()), this, SLOT(slotRenderLOD3()));
    connect(m_ui->actionForce_LOD4, SIGNAL(triggered()), this, SLOT(slotRenderLOD4()));

    // generate LODs signals
    connect(m_ui->actionAll_LODs, SIGNAL(triggered()), this, SLOT(generateAllLODs()));
    connect(m_ui->actionLOD0, SIGNAL(triggered()), this, SLOT(generateLOD0()));
    connect(m_ui->actionLOD1, SIGNAL(triggered()), this, SLOT(generateLOD1()));
    connect(m_ui->actionLOD2, SIGNAL(triggered()), this, SLOT(generateLOD2()));
    connect(m_ui->actionLOD3, SIGNAL(triggered()), this, SLOT(generateLOD3()));
    connect(m_ui->actionLOD4, SIGNAL(triggered()), this, SLOT(generateLOD4()));

    connect(m_ui->actionFix_building, SIGNAL(triggered()), this, SLOT(slotFixBuilding()));

    connect(m_ui->actionTest_1, SIGNAL(triggered()), this, SLOT(test1()));
    connect(m_ui->actionTest_2, SIGNAL(triggered()), this, SLOT(test2()));
    connect(m_ui->actionTest_3, SIGNAL(triggered()), this, SLOT(test3()));
    connect(m_ui->actionTest_4, SIGNAL(triggered()), this, SLOT(test4()));
    connect(m_ui->actionTest_5, SIGNAL(triggered()), this, SLOT(test5()));

    // filter search
    connect(m_ui->filterButton, SIGNAL(clicked()), m_treeView, SLOT(slotFilter()));

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(slotTemporalAnimUpdate()));

    m_ui->menuDebug->setVisible(false);
    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

    reset();

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

    //clearRecentFiles();

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
        vcity::URI uriLayer = m_app.getScene().getDefaultLayer()->getURI();
        vcity::log() << uriLayer.getStringURI() << "\n";
        appGui().getControllerGui().addTile(uriLayer, *tile);
        //m_app.getScene().getLayers()[0]->addTile(tile);
        //m_osgScene->addTile(*tile);
        //m_osgView->centerCamera();


        //setOsgData(buildOsgScene(*tile));
        //fillTreeView(tile); -> put in controller

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
    m_osgView->setActive(false);

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

    m_osgView->setActive(true);
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
    m_osgView->setActive(false);

    //std::cout<<"Load Scene recursive"<<std::endl;
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
    if(w.exec())
    {
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

    m_osgView->setActive(true);
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
void MainWindow::updateTextBox(const std::stringstream& ss)
{
    m_ui->textBrowser->setText(ss.str().c_str());
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateTextBox(const vcity::URI& uri)
{
    std::stringstream ss;
    ss << uri.getStringURI() << std::endl;

    citygml::CityObject* obj = vcity::app().getScene().getNode(uri);
    if(obj)
    {
        ss << "ID : " << obj->getId() << std::endl;
        ss << "Type : " << obj->getTypeAsString() << std::endl;
        //obj->l
        ss << "Temporal : " << obj->isTemporal() << std::endl;

        ss << "Attributes : " << std::endl;
        citygml::AttributesMap attribs = obj->getAttributes();
        citygml::AttributesMap::const_iterator it = attribs.begin();
        while ( it != attribs.end() )
        {
            ss << " + " << it->first << ": " << it->second << std::endl;
            ++it;
        }

        // get textures
        // parse geometry
        std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
        std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
        for(; itGeom != geoms.end(); ++itGeom)
        {
            // parse polygons
            std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
            std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
            for(; itPoly != polys.end(); ++itPoly)
            {
                citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                std::vector<TVec3d>& vertices = ring->getVertices();
                std::vector<TVec3d>::iterator itVertices = vertices.begin();
                ss << "Linear ring (" << (*itGeom)->getId() << " : " << (*itPoly)->getId() << ") : ";
                for(; itVertices != vertices.end(); ++itVertices)
                {
                    // do stuff with points...
                    TVec3d point = *itVertices;
                    ss << point;
                }
                ss << std::endl;

                const citygml::Texture* tex = (*itPoly)->getTexture();
                if(tex)
                {
                    ss << "Texture (" << (*itGeom)->getId() << " : " << (*itPoly)->getId() << ") : " << tex->getUrl() << std::endl;
                }
            }

        }

        //m_pickhandler->resetPicking();
        //m_pickhandler->addNodePicked(uri);
    }

    updateTextBox(ss);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateTextBoxWithSelectedNodes()
{

}
////////////////////////////////////////////////////////////////////////////////
QLineEdit* MainWindow::getFilter()
{
    return m_ui->filterLineEdit;
}
////////////////////////////////////////////////////////////////////////////////
// Tools
////////////////////////////////////////////////////////////////////////////////
void MainWindow::reset()
{
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
void MainWindow::slotSettings()
{
    DialogSettings diag;
    diag.doSettings();
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
void MainWindow::updateTemporalParams(int value)
{
    if(value == -1) value = m_ui->horizontalSlider->value();
    QDate date(1900, 1, 1);
    date = date.addDays(value);
    //m_ui->buttonBrowserTemporal->setText(date.toString());
    m_ui->dateTimeEdit->setDate(date);

    //std::cout << "set year : " << date.year() << std::endl;

    QDateTime datetime(date);
    if(m_useTemporal)   m_osgScene->setDate(datetime);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::toggleUseTemporal()
{
    m_useTemporal = !m_useTemporal;

    if(m_useTemporal)
    {
        QDate date(1900, 1, 1);
        date = date.addDays(m_ui->horizontalSlider->value());
        QDateTime datetime(date);
        m_osgScene->setDate(datetime);
    }
    else
    {
        QDate date(0, 0, 0);
        QDateTime datetime(date);
        m_osgScene->setDate(datetime); // reset
        m_timer.stop();
    }

    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

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
    //const std::set<std::string>& nodes = m_pickhandler->getNodesPicked(); // TODO : update this with a uri list
    const std::vector<vcity::URI>& nodes = appGui().getSelectedNodes();
    if(nodes.size() > 0)
    {
        //std::cout << "Citygml export cityobject : " << *nodes.begin() << std::endl;
        // use first node picked
        //citygml::CityObject* model = m_app.getScene().getDefaultLayer()->getTiles()[0]->findNode(*nodes.begin()); // use getNode
        //citygml::exportCitygml(model, "test.citygml");
        //if(model) exporter.exportCityObject(model, filename.toStdString());
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
void MainWindow::slotDumpScene()
{
    m_app.getScene().dump();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotDumpSelectedNodes()
{
    vcity::log() << "Selected nodes uri : \n";
    for(std::vector<vcity::URI>::const_iterator it = appGui().getSelectedNodes().begin(); it < appGui().getSelectedNodes().end(); ++it)
    {
        vcity::log() << it->getStringURI() << "\n";
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateAllLODs()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD0()
{
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    if(uris.size() > 0)
    {
        // do all nodes selected
        for(std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
        {
            vcity::app().getAlgo().generateLOD0(*it);
            // TODO
            appGui().getControllerGui().update(*it);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD1()
{
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    if(uris.size() > 0)
    {
        // do all nodes selected
        for(std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
        {
            vcity::app().getAlgo().generateLOD1(*it);
            // TODO
            //appGui().getControllerGui().update(*it);
        }
    }
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
void MainWindow::slotFixBuilding()
{
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    vcity::app().getAlgo2().fixBuilding(uris);

    // TODO
    //appGui().getControllerGui().update(uri);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotOptimOSG()
{
    appGui().getOsgScene()->optim();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD0()
{
    appGui().getOsgScene()->forceLOD(0);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD1()
{
    appGui().getOsgScene()->forceLOD(1);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD2()
{
    appGui().getOsgScene()->forceLOD(2);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD3()
{
    appGui().getOsgScene()->forceLOD(3);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD4()
{
    appGui().getOsgScene()->forceLOD(4);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotTemporalAnim()
{
    m_temporalAnim = !m_temporalAnim;
    if(m_temporalAnim)
    {
        m_timer.start(500);
    }
    else
    {
        m_timer.stop();
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotTemporalAnimUpdate()
{
    m_ui->horizontalSlider->setValue(m_ui->horizontalSlider->value()+365);
    //std::cout << m_ui->horizontalSlider->value() << std::endl;
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

    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13726/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13726/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1294-13726/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1294-13725/export-CityGML/ZoneAExporter.gml");
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
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1305-13719/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1304-13719/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1305-13720/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1304-13720/export-CityGML/ZoneAExporter.gml");
}
////////////////////////////////////////////////////////////////////////////////
citygml::LinearRing* cpyOffsetLinearRing(citygml::LinearRing* ring, float offset)
{
    citygml::LinearRing* ringOffset = new citygml::LinearRing(ring->getId()+"_offset", true);

    std::vector<TVec3d>& vertices = ring->getVertices();
    for(std::vector<TVec3d>::iterator itVertices = vertices.begin(); itVertices != vertices.end(); ++itVertices)
    {
        TVec3d point = *itVertices;
        point.z += offset;
        ringOffset->addVertex(point);
    }

    return ringOffset;
}
///////////////////////////////////////////////////////////////////////////////////
void test5rec(citygml::CityObject* obj)
{
    std::vector<citygml::Polygon*> polyBuf;

    // parse geometry
    std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
    for(std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin(); itGeom != geoms.end(); ++itGeom)
    {
        // parse polygons
        std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
        for(std::vector<citygml::Polygon*>::iterator itPoly = polys.begin(); itPoly != polys.end(); ++itPoly)
        {
            // get linear ring
            citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
            citygml::LinearRing* ringOffset = cpyOffsetLinearRing(ring, 100);

            citygml::Polygon* poly = new citygml::Polygon((*itPoly)->getId()); // ((*itPoly)->getId()+"_"+ringOffset->getId());
            poly->addRing(ringOffset);
            //(*itGeom)->addPolygon(poly);
            polyBuf.push_back(poly);
        }

        for(std::vector<citygml::Polygon*>::iterator it = polyBuf.begin(); it < polyBuf.end(); ++it)
        {
            (*itGeom)->addPolygon(*it);
        }
    }

    citygml::CityObjects& cityObjects = obj->getChildren();
    for(citygml::CityObjects::iterator itObj = cityObjects.begin(); itObj != cityObjects.end(); ++itObj)
    {
        test5rec(*itObj);
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test5()
{
    if(appGui().getSelectedNodes().size() > 0)
    {
        citygml::CityObject* obj = appGui().getScene().getNode(appGui().getSelectedNodes()[0]);
        test5rec(obj);
        appGui().getControllerGui().update(appGui().getSelectedNodes()[0]);
    }
}
////////////////////////////////////////////////////////////////////////////////
