// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include <QFileDialog>
#include <QCheckBox>
#include <QDirIterator>
#include <QSettings>
#include <QListView>
#include <QMessageBox>
#include <QPluginLoader>
#include <ctime>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <time.h>

#include "ui_mainWindow.h"

#include "moc/mainWindow.hpp"
#include "moc/dialogLoadBBox.hpp"
#include "moc/dialogSettings.hpp"
#include "moc/dialogAbout.hpp"
#include "moc/dialogTilingCityGML.hpp"
#include "moc/dialogConvertObjToCityGML.hpp"
#include "moc/dialogBuildBuildingAABBs.hpp"

#include "controllerGui.hpp"

#include "libcitygml/citygml.hpp"
#include "libcitygml/export/exportCityGML.hpp"
#include "libcitygml/utils/exportJSON.hpp"
#include "libcitygml/utils/exportOBJ.hpp"

#include "importerAssimp.hpp"

#include "gui/osg/osgScene.hpp"
#include "gdal_priv.h"
#include "osg/osgGDAL.hpp"
#include "osg/osgAssimp.hpp"
#include "osg/osgLas.hpp"

#include "DataStructures/DEM/osgMnt.hpp"

#include "utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"
#include "utils/CityGMLFusion.h"

#include "libcitygml/utils/CityGMLtools.hpp"

#include "filters/ChangeDetection/ChangeDetection.hpp"
#include "filters/EnhanceCityGML/LinkCityGMLShape.hpp"
#include "filters/Tiling/TilingCityGML.hpp"
#include "filters/EnhanceCityGML/EnhanceMNT.hpp"

#include "pluginInterface.h"
#include "moc/plugindialog.hpp"
#include "TiledFilesLayout.hpp"

//FIXME: the following block (CLEMENT LIB ADDING) is used by one of the
// test functions and can be made away together with those tests.

#include "filters/raytracing/RayTracing.hpp"
#include "Triangle.hpp"
#include "filters/raytracing/Hit.hpp"

////////////////////////////////////////////////////////////////////////////////

std::vector<std::pair<double, double>> Hauteurs;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), m_ui(new Ui::MainWindow), m_useTemporal(false), m_temporalAnim(false), m_unlockLevel(0)
{
    m_ui->setupUi(this);

    m_app.setMainWindow(this);

    // create Qt treeview
    m_treeView = new TreeView(m_ui->treeWidget, this);
    m_app.setTreeView(m_treeView);
    m_app.setTextBowser(m_ui->textBrowser);

    // create controller
    m_app.setControllerGui(new ControllerGui());

    reset();

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
    connect(m_ui->actionLoad_CSV, SIGNAL(triggered()), this, SLOT(loadDocuments()));
	connect(m_ui->actionExport_citygml, SIGNAL(triggered()), this, SLOT(exportCityGML()));
	connect(m_ui->actionExport_osg, SIGNAL(triggered()), this, SLOT(exportOsg()));
	connect(m_ui->actionExport_tiled_osga, SIGNAL(triggered()), this, SLOT(exportOsga()));
	connect(m_ui->actionExport_JSON, SIGNAL(triggered()), this, SLOT(exportJSON()));
	connect(m_ui->actionExport_OBJ, SIGNAL(triggered()), this, SLOT(exportOBJ()));
	connect(m_ui->actionExport_OBJ_split, SIGNAL(triggered()), this, SLOT(exportOBJsplit()));
	//connect(m_ui->actionDelete_node, SIGNAL(triggered()), this, SLOT(deleteNode()));
	connect(m_ui->actionReset, SIGNAL(triggered()), this, SLOT(resetScene()));
	connect(m_ui->actionClearSelection, SIGNAL(triggered()), this, SLOT(clearSelection()));
	connect(m_ui->actionBuilding, SIGNAL(triggered()), this, SLOT(optionPickBuiling()));
	connect(m_ui->actionFace, SIGNAL(triggered()), this, SLOT(optionPickFace()));
	connect(m_ui->actionInfo_bubbles, SIGNAL(triggered()), this, SLOT(optionInfoBubbles()));
	connect(m_ui->actionShadows, SIGNAL(triggered()), this, SLOT(optionShadow()));
    connect(m_ui->actionSkybox, SIGNAL(triggered()), this, SLOT(optionSkybox()));
	connect(m_ui->actionSettings, SIGNAL(triggered()), this, SLOT(slotSettings()));
	//connect(m_ui->actionAdd_Tag, SIGNAL(triggered()), this, SLOT(optionAddTag()));
	//connect(m_ui->actionAdd_Flag, SIGNAL(triggered()), this, SLOT(optionAddFlag()));
	connect(m_ui->actionShow_temporal_tools, SIGNAL(triggered()), this, SLOT(optionShowTemporalTools()));
    connect(m_ui->checkBoxTemporalTools, SIGNAL(stateChanged(int)), this, SLOT(toggleUseTemporal()));
	connect(m_ui->actionShow_advanced_tools, SIGNAL(triggered()), this, SLOT(optionShowAdvancedTools()));
	//connect(m_ui->treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(handleTreeView(QTreeWidgetItem*, int)));
	connect(m_ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateTemporalParams(int)));
	connect(m_ui->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(updateTemporalParams()));
	connect(m_ui->dateTimeEdit, SIGNAL(editingFinished()), this, SLOT(updateTemporalSlider()));
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
    connect(m_ui->actionScene_GenerateLOD0, SIGNAL(triggered()), this, SLOT(generateLOD0())); //Generate LOD0 on objects from scene
    connect(m_ui->actionScene_GenerateLOD1, SIGNAL(triggered()), this, SLOT(generateLOD1()));
    connect(m_ui->actionLOD0, SIGNAL(triggered()), this, SLOT(generateLOD0OnFile()));
    connect(m_ui->actionLOD1, SIGNAL(triggered()), this, SLOT(generateLOD1OnFile())); //Generate LOD1 on objects from folder
    connect(m_ui->actionLOD2, SIGNAL(triggered()), this, SLOT(generateLOD2()));
    connect(m_ui->actionLOD3, SIGNAL(triggered()), this, SLOT(generateLOD3()));
    connect(m_ui->actionLOD4, SIGNAL(triggered()), this, SLOT(generateLOD4()));

    connect(m_ui->actionFix_building, SIGNAL(triggered()), this, SLOT(slotFixBuilding()));

    connect(m_ui->actionChange_Detection, SIGNAL(triggered()), this, SLOT(slotChangeDetection()));
    connect(m_ui->actionOBJ_to_CityGML, SIGNAL(triggered()), this, SLOT(slotObjToCityGML()));
    connect(m_ui->actionSplit_CityGML_Buildings, SIGNAL(triggered()), this, SLOT(slotSplitCityGMLBuildings()));
    connect(m_ui->actionCut_CityGML_with_Shapefile, SIGNAL(triggered()), this, SLOT(slotCutCityGMLwithShapefile()));

    connect(m_ui->actionTiling_CityGML, SIGNAL(triggered()), this, SLOT(slotTilingCityGML()));
    connect(m_ui->actionBuild_Building_AABBs,SIGNAL(triggered()), this, SLOT(slotBuildBuildingAABBs()));
    connect(m_ui->actionCut_MNT_with_Shapefile, SIGNAL(triggered()), this, SLOT(slotCutMNTwithShapefile()));
    connect(m_ui->actionCreate_Roads_on_MNT, SIGNAL(triggered()), this, SLOT(slotCreateRoadOnMNT()));
    connect(m_ui->actionCreate_Vegetation_on_MNT, SIGNAL(triggered()), this, SLOT(slotCreateVegetationOnMNT()));

    connect(m_ui->actionTest_1, SIGNAL(triggered()), this, SLOT(test1()));
    connect(m_ui->actionTest_2, SIGNAL(triggered()), this, SLOT(test2()));
    connect(m_ui->actionTest_3, SIGNAL(triggered()), this, SLOT(test3()));
    connect(m_ui->actionTest_4, SIGNAL(triggered()), this, SLOT(test4()));
    connect(m_ui->actionTest_5, SIGNAL(triggered()), this, SLOT(test5()));

    // filter search
    connect(m_ui->filterButton, SIGNAL(clicked()), m_treeView, SLOT(slotFilter()));

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(slotTemporalAnimUpdate()));

    initTemporalTools();
    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

    updateRecentFiles();

    m_treeView->init();

    // plugins
    aboutPluginsAct = new QAction(tr("Plugins information"), this);
    connect(aboutPluginsAct, SIGNAL(triggered()), this, SLOT(aboutPlugins()));

    pluginMenu = m_ui->menuPlugins;
    pluginMenu->clear();
    pluginMenu->addAction(aboutPluginsAct);

    loadPlugins();
    // plugins

    //m_ui->statusBar->showMessage("none");

    setlocale(LC_ALL, "C"); // MT : important for Linux
}
////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow()
{
    delete aboutPluginsAct;

    delete m_treeView;
    delete m_osgView;
    delete m_ui;
}
////////////////////////////////////////////////////////////////////////////////
// Plugins
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadPlugins()
{
    foreach(QObject *plugin, QPluginLoader::staticInstances())
        populateMenus(plugin);

    //std::cout << "-> loadPlugins from " << qApp->applicationDirPath().toStdString() << "\n";
    pluginsDir = QDir(qApp->applicationDirPath());

#if defined(Q_OS_WIN)
    /*if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
    pluginsDir.cdUp();*/
#elif defined(Q_OS_MAC)
    if (pluginsDir.dirName() == "MacOS") {
        pluginsDir.cdUp();
        pluginsDir.cdUp();
        pluginsDir.cdUp();
    }
#endif
    //pluginsDir.cd("plugins");

    foreach(QString fileName, pluginsDir.entryList(QDir::Files))
    {
        if (/*fileName.contains("Filter") && */QLibrary::isLibrary(fileName))
        {
            QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
            QObject *plugin = loader.instance();
            if (plugin)
            {
                pluginMenu->addSeparator();
                populateMenus(plugin);
                pluginFileNames += fileName;

                // call plugin's init() method
                Generic_PluginInterface* vcity_plugin = qobject_cast<Generic_PluginInterface*>(plugin);
                vcity_plugin->init(this);
            }
        }
    }

    pluginMenu->setEnabled(!pluginMenu->actions().isEmpty());
}

void MainWindow::populateMenus(QObject *plugin)
{
    Generic_PluginInterface *iGeneric_Filter = qobject_cast<Generic_PluginInterface *>(plugin);
    if (iGeneric_Filter)
        addToMenu(plugin, iGeneric_Filter->Generic_plugins(), pluginMenu, SLOT(applyPlugin()));
}

void MainWindow::addToMenu(QObject *plugin, const QStringList &texts,
    QMenu *menu, const char *member,
    QActionGroup *actionGroup)
{
    foreach(QString text, texts) {
        QAction *action = new QAction(text, plugin);
        connect(action, SIGNAL(triggered()), this, member);
        menu->addAction(action);

        if (actionGroup) {
            action->setCheckable(true);
            actionGroup->addAction(action);
        }
    }
}

void MainWindow::applyPlugin()
{
    QAction *action = qobject_cast<QAction *>(sender());

    Generic_PluginInterface *iGeneric_Filter
        = qobject_cast<Generic_PluginInterface *>(action->parent());
    if (iGeneric_Filter)
        iGeneric_Filter->Generic_plugin(action->text());
}

void MainWindow::aboutPlugins()
{
    PluginDialog dialog(pluginsDir.path(), pluginFileNames, this);
    dialog.exec();
}
////////////////////////////////////////////////////////////////////////////////
// Recent files
////////////////////////////////////////////////////////////////////////////////
void MainWindow::addRecentFile(const QString& filepath)
{
    QSettings settings("liris", "virtualcity");
    QStringList list = settings.value("recentfiles").toStringList();

    list.removeAll(filepath);
    list.prepend(filepath);
    if (list.size() >= 10)
    {
        list.removeLast();
    }

    settings.setValue("recentfiles", list);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::removeRecentFile(const QString& filepath)
{
    QSettings settings("liris", "virtualcity");
    QStringList list = settings.value("recentfiles").toStringList();
    list.removeAll(filepath);
    settings.setValue("recentfiles", list);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateRecentFiles()
{
    QSettings settings("liris", "virtualcity");
    QStringList list = settings.value("recentfiles").toStringList();
    list.removeDuplicates();
    settings.setValue("recentfiles", list);

    clearRecentFiles(false);

    foreach(QString str, list)
    {
        QAction* action = new QAction(str, this);
        m_ui->menuRecent_files->addAction(action);
        connect(action, SIGNAL(triggered()), this, SLOT(openRecentFile()));
        action->setData(str);
    }

    // add reset item last
    QAction* action = new QAction("Clear", this);
    m_ui->menuRecent_files->addAction(action);
    connect(action, SIGNAL(triggered()), this, SLOT(clearRecentFiles()));
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::clearRecentFiles(bool removeAll)
{
    m_ui->menuRecent_files->clear();

    if (removeAll)
    {
        QSettings settings("liris", "virtualcity");
        settings.setValue("recentfiles", QStringList());
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::openRecentFile()
{
    QAction* action = qobject_cast<QAction*>(sender());
    if (action)
    {
        loadFile(action->data().toString());
        updateRecentFiles();
    }
}
////////////////////////////////////////////////////////////////////////////////
// Open files
////////////////////////////////////////////////////////////////////////////////
bool MainWindow::loadFile(const QString& filepath)
{
    // date check
    if (QDate::currentDate() > QDate(2016, 12, 31))
    {
        QMessageBox(QMessageBox::Critical, "Error", "Expired").exec();
        return false;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    QSettings settings("liris", "virtualcity");
    QFileInfo file(filepath);

    if (!file.exists())
    {
        removeRecentFile(filepath);
        QApplication::restoreOverrideCursor();
        return false;
    }

    QString ext = file.suffix().toLower();
    if (ext == "citygml" || ext == "gml")
    {
        std::cout << "load citygml file : " << filepath.toStdString() << std::endl;

        // add tile
        vcity::Tile* tile = new vcity::Tile(filepath.toStdString());
        vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerCityGML")->getURI();
        vcity::log() << uriLayer.getStringURI() << "\n";
        appGui().getControllerGui().addTile(uriLayer, *tile);

        addRecentFile(filepath);

        /* QStringList list = settings.value("recentfiles").toStringList();
        list.append(filepath);
        settings.setValue("recentfiles", list);*/
    }
    // Assimp importer
    else if (ext == "assimp" || ext == "dae" || ext == "blend" || ext == "3ds" || ext == "ase" || ext == "obj" || ext == "xgl" || ext == "ply" || ext == "dxf" || ext == "lwo" || ext == "lws" ||
        ext == "lxo" || ext == "stl" || ext == "x" || ext == "ac" || ext == "ms3d" || ext == "scn" || ext == "xml" || ext == "irrmesh" || ext == "irr" ||
        ext == "mdl" || ext == "md2" || ext == "md3" || ext == "pk3" || ext == "md5" || ext == "smd" || ext == "m3" || ext == "3d" || ext == "q3d" || ext == "off" || ext == "ter")
    {
        /*Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(filepath.toStdString(), aiProcessPreset_TargetRealtime_Fast); // aiProcessPreset_TargetRealtime_Quality

        aiMesh *mesh = scene->mMeshes[0]; // assuming you only want the first mesh*/

        // ---

        std::string readOptionString = "";
        osg::ref_ptr<osgDB::Options> readOptions = new osgDB::Options(readOptionString.c_str());
        ReadResult readResult = readNode(filepath.toStdString(), readOptions);
        if (readResult.success())
        {
            osg::ref_ptr<osg::Node> node = readResult.getNode();

            // set assimpNode name
            static int id = 0;
            std::stringstream ss;
            ss << "assimpNode" << id++;
            node->setName(ss.str());

            vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerAssimp")->getURI();
            vcity::log() << uriLayer.getStringURI() << "\n";
            appGui().getControllerGui().addAssimpNode(uriLayer, node);

            addRecentFile(filepath);
        }
    }
    // MntAsc importer
    else if (ext == "asc")
    {
        MNT mnt;

        if (mnt.charge(filepath.toStdString().c_str(), "ASC"))
        {
            osg::ref_ptr<osg::Node> node = mnt.buildAltitudesGrid(-m_app.getSettings().getDataProfile().m_offset.x, -m_app.getSettings().getDataProfile().m_offset.y);

            // set mntAscNode name
            static int id = 0;
            std::stringstream ss;
            ss << "mntAscNode" << id++;
            node->setName(ss.str());

            vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerMnt")->getURI();
            vcity::log() << uriLayer.getStringURI() << "\n";
            appGui().getControllerGui().addMntAscNode(uriLayer, node);

            addRecentFile(filepath);

            //mnt.sauve_log(std::string("mntAsc.txt").c_str(), std::string("mntAsc.tga").c_str()); // mntAsc.tga bidon
            //mnt.sauve_partie(std::string("mntAsc_partie.txt").c_str(), 0, 0, mnt.get_dim_x(), mnt.get_dim_y());
            //mnt.sauve_partie_XML(std::string("mntAsc_partie_xml.txt").c_str(), 0, 0, mnt.get_dim_x(), mnt.get_dim_y());
        }
    }
    // las importer
    else if (ext == "las" || ext == "laz")
    {
        LAS las;

        if (las.open(filepath.toStdString().c_str()))
        {
            vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerLas")->getURI();
            vcity::log() << uriLayer.getStringURI() << "\n";

            osg::ref_ptr<osg::Node> node = las.buildLasPoints(uriLayer, -m_app.getSettings().getDataProfile().m_offset.x, -m_app.getSettings().getDataProfile().m_offset.y);

            // set lasNode name
            static int id = 0;
            std::stringstream ss;
            ss << "lasNode" << id++;
            node->setName(ss.str());

            appGui().getControllerGui().addLasNode(uriLayer, node);

            las.close();

            addRecentFile(filepath);
        }
    }
    else if (ext == "shp")
    {
        loadShpFile(filepath);
    }
    else if (ext == "dxf")
    {
        std::cout << "load dxf file : " << filepath.toStdString() << std::endl;
        OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);

        m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));
    }
    else if (ext == "ecw")
    {
        std::cout << "load ecw file : " << filepath.toStdString() << std::endl;
        GDALDataset* poDataset = (GDALDataset*)GDALOpen(filepath.toStdString().c_str(), GA_ReadOnly);
        if (poDataset == NULL)
        {
            std::cout << "load ecw file : " << filepath.toStdString() << std::endl;
        }
    }
    else
    {
        std::cout << "bad file : " << filepath.toStdString() << std::endl;
        QApplication::restoreOverrideCursor();
        return false;
    }

    QApplication::restoreOverrideCursor();

    return true;
}
////////////////////////////////////////////////////////////////////////////////
bool MainWindow::loadCSV(const QString& CSVfilepath, const QString& DIRfilepath)
{
    std::string sourcepath = "/home/pers/clement.chagnaud/Documents/Data/spreadsheet_testnolod.csv";

    // date check
    if (QDate::currentDate() > QDate(2016, 12, 31))
    {
        QMessageBox(QMessageBox::Critical, "Error", "Expired").exec();
        return false;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    QFileInfo file(CSVfilepath);

    if (!file.exists())
    {
        removeRecentFile(CSVfilepath);
        QApplication::restoreOverrideCursor();
        return false;
    }

    QString ext = file.suffix().toLower();
    if (ext == "csv")
    {
        std::cout << "load csv file : " << CSVfilepath.toStdString() << std::endl;


        double offsetx=m_app.getSettings().getDataProfile().m_offset.x;
        double offsety=m_app.getSettings().getDataProfile().m_offset.y;

        std::string sourcepath = CSVfilepath.toStdString();
        std::string tilesdirectory = DIRfilepath.toStdString();

        std::vector<osgInfo*> v_info;
        std::vector<float> v_height;
        std::vector<float> v_width;
        std::vector<double> v_angle;
        std::vector<float> v_anchoring;

        std::vector<int> v_priority;

        std::vector<osg::Vec3> v_position;
        std::vector<osg::Vec3> v_axis;


        std::vector<std::string> v_filepath;
        std::vector<std::string> v_name;
        std::vector<std::string> v_filetype;
        std::vector<std::string> v_sourcetype;
        std::vector<std::string> v_LOD;
        std::vector<std::string> v_publicationdate;

    // *** CSV Load
        std::ifstream file(sourcepath);
        std::string line;
        std::getline(file,line); //get the first line
        int cpt = 0 ;
        float x=0;
        float y=0;
        float z=0;
        while(std::getline(file,line)) // For all lines of csv file
        {
            std::stringstream  lineStream(line);
            std::string        cell;
            cpt=0;
            x=0;
            y=0;
            z=0;
            while(std::getline(lineStream,cell,','))
            {
                if (cpt==0)
                        v_height.push_back(std::stod(cell));
                if (cpt==1)
                        v_width.push_back(std::stod(cell));
                if (cpt==2)
                        x=std::stod(cell);

                if (cpt==3)
                        y=std::stod(cell);

                if (cpt==4)
                        z=std::stod(cell);

                if (cpt==5)
                        v_angle.push_back(std::stod(cell));
                if (cpt==6)
                {
                    if(cell=="z")
                        v_axis.push_back(osg::Vec3(0,0,1));
                }
                if (cpt==7)
                        v_filepath.push_back(cell);

                if (cpt==8)
                        v_name.push_back(cell);

                if (cpt==9)
                        v_filetype.push_back(cell);

                if (cpt==10)
                        v_sourcetype.push_back(cell);

                if (cpt==11)
                    v_LOD.push_back(cell);
                if (cpt==12)
                    v_anchoring.push_back(std::stod(cell));

                if(cpt==13)
                    v_priority.push_back(std::stod(cell));
                if(cpt==14)
                    v_publicationdate.push_back(cell);

                cpt++;
             }
            v_position.push_back(osg::Vec3(x,y,z));
        }

        for (std::size_t i=0; i<v_filepath.size(); ++i)
        {
            v_info.push_back(new osgInfo(v_height[i],v_width[i], v_position[i],v_angle[i], v_axis[i], v_filepath[i], v_name[i], v_filetype[i],
                                         v_sourcetype[i], v_LOD[i], v_anchoring[i], v_priority[i],v_publicationdate[i]));
        }


        std::vector<Ray*> v_ray;
        int cpt2=0;
        int id = 0; //Position of current osgInfo in v_info. Will be used as id for raytracing
        bool raytracing = false;

        std::vector<std::string> vecTile;
        bool tilefound ;

        std::vector< std::vector<Ray*> > vecRay;

        for(osgInfo* i : v_info)
        {
            if(i->m_anchoring==0)
            {
                osg::Vec3 pos = i->getPosition();
                TVec3d ori = TVec3d(pos.x()+offsetx,pos.y()+offsety,pos.z());
                Ray* tmp_ray = new Ray(ori,TVec3d(0.0,0.0,-1.0), id);

                int tileX = floor((pos.x()+offsetx)/500);
                int tileY = floor((pos.y()+offsety)/500);
                std::string tilename = std::to_string(tileX)+"_"+std::to_string(tileY)+".gml";
                tilefound = false;
                for(std::size_t j=0; j<vecTile.size(); ++j)
                {
                    if(vecTile[j]==tilename)
                    {
                        vecRay[j].push_back(tmp_ray);
                        tilefound = true;
                    }
                }
                if(!tilefound)
                {
                    vecTile.push_back(tilename);
                    std::vector<Ray*> newtile;
                    newtile.push_back(tmp_ray);
                    vecRay.push_back(newtile);
                }
                raytracing = true;
                cpt2++;
            }
            ++id;
        }

        std::cout<<"Anchoring points to update : "<<cpt2<<std::endl;

        if(raytracing)
        {
            for(std::size_t j=0; j<vecTile.size(); ++j)
            {
                std::cout<<"In file : "<<tilesdirectory<<std::endl;
                std::cout<<"    ... open tile : "<<vecTile[j]<<std::endl;
                std::string batipath=tilesdirectory+"/_BATI/"+vecTile[j];
                std::string mntpath=tilesdirectory+"/_MNT/"+vecTile[j];

                TriangleList* triangles_bati = BuildTriangleList(batipath, citygml::CityObjectsType::COT_Building);
                TriangleList* triangles_mnt = BuildTriangleList(mntpath, citygml::CityObjectsType::COT_TINRelief);

                TriangleList* triangles = new TriangleList();
                triangles->triangles.reserve(triangles_bati->triangles.size() + triangles_mnt->triangles.size());

                triangles->triangles.insert(triangles->triangles.end(), triangles_bati->triangles.begin(), triangles_bati->triangles.end());
                triangles->triangles.insert(triangles->triangles.end(), triangles_mnt->triangles.begin(), triangles_mnt->triangles.end());

                std::vector<Hit*>* v_hit = RayTracing(triangles, vecRay[j]);

                for(Hit* h : *(v_hit))
                {
                    v_info.at(h->ray.id)->setAnchoringPoint(h->point.z);
                }

            }

        }


                std::ofstream ofs;
        ofs.open (sourcepath, std::ofstream::in | std::ofstream::out);
        ofs<<"height,width,position x,position y,position z,angle,axe,filepath,name,filetype,sourcetype,LOD,ancrage,priority,publicationdate"<<std::endl;
        for(osgInfo* i : v_info)
        {
            ofs<<std::to_string(i->m_height)<<","<<std::to_string(i->m_width)<<","<<i->m_initposition.x()<<","<<i->m_initposition.y()<<","<<i->m_initposition.z()
                                 <<","<<std::to_string(i->m_angle)<<",";
            ofs<<"z"<<","<<i->m_filepath<<","<<i->m_name<<","<<i->m_filetype<<","<<i->m_sourcetype<<","<<i->m_LOD<<","
                                <<i->m_anchoring<<","<<i->m_priority<<","<<i->m_publicationDate<<std::endl;
        }
        ofs.close();






        vcity::URI uriInfoLayer = m_app.getScene().getDefaultLayer("LayerInfo")->getURI();
        appGui().getControllerGui().addInfo(uriInfoLayer, v_info);

        for (std::size_t i=0; i<v_info.size() ; ++i)
        {
            v_info[i]->setBillboarding(true);
        }

        addRecentFile(CSVfilepath);
    }

}



////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadScene()
{
    m_osgView->setActive(false); // reduce osg framerate to have better response in Qt ui (it would be better if ui was threaded)

    std::cout << "Load Scene" << std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(this, "Load scene files", lastdir);

    for (int i = 0; i < filenames.count(); ++i)
    {
        QFileInfo file(filenames[i]);
        bool success = loadFile(file.absoluteFilePath());
        if (success)
        {
            // save path
            QFileInfo file(filenames[i]);
            //std::cout << "lastdir : " << file.dir().absolutePath().toStdString() << std::endl;
            settings.setValue("lastdir", file.dir().absolutePath());
        }
    }
    //std::cout << "lastdir set : " << settings.value("lastdir").toString().toStdString() << std::endl;

    updateRecentFiles();

    m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadDocuments()
{
    m_osgView->setActive(false); // reduce osg framerate to have better response in Qt ui (it would be better if ui was threaded)

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList CSVfilenames = QFileDialog::getOpenFileNames(this, "Selectionner le fichier CSV", lastdir);

    QString GMLdirectory = QFileDialog::getExistingDirectory(this, tr("Selectionner le dossier Tuiles"),lastdir,QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    std::cout << "directory : " << GMLdirectory.toStdString() << std::endl;

    if(!GMLdirectory.isNull())
    {
        for (int i = 0; i < CSVfilenames.count(); ++i)
        {
            QFileInfo file(CSVfilenames[i]);
            bool success = loadCSV(file.absoluteFilePath(), GMLdirectory);
            if (success)
            {
                // save path
                QFileInfo file(CSVfilenames[i]);
                //std::cout << "lastdir : " << file.dir().absolutePath().toStdString() << std::endl;
                settings.setValue("lastdir", file.dir().absolutePath());
            }
        }
    }

    updateRecentFiles();

    m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)
}
////////////////////////////////////////////////////////////////////////////////
void buildRecursiveFileList(const QDir& dir, QStringList& list)
{
    QDirIterator iterator(dir.absolutePath(), QDirIterator::Subdirectories);
    while (iterator.hasNext())
    {
        iterator.next();
        if (!iterator.fileInfo().isDir())
        {
            QString filename = iterator.filePath();
            if (filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive) || filename.endsWith(".shp", Qt::CaseInsensitive) || filename.endsWith(".obj", Qt::CaseInsensitive))
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
    w.setWindowTitle("Selectionner le dossier contenant les fichiers a ouvrir");
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
    if (t)
    {
        t->setSelectionMode(QAbstractItemView::MultiSelection);
    }
    if (w.exec())
    {
        QStringList file = w.selectedFiles();
        foreach(QString s, file)
        {
            QFileInfo file(s);
            // do some useful stuff here
            std::cout << "Working on file " << s.toStdString() << " type : " << file.isDir() << ", " << file.isFile() << std::endl;
            if (file.isDir())
            {
                QDir dir(s);
                QStringList files;
                buildRecursiveFileList(dir, files);

                for (int i = 0; i < files.count(); ++i)
                {
                    loadFile(files[i]);
                }
            }
            else if (file.isFile())
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
    DialogLoadBBox diag;
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

    // not really good here but, no choice...
    bool bHack = true;
    if (uri.getType() == "Workspace" || uri.getType() == "Version") bHack = false;
    // not really good here but, no choice...

    citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(uri, bHack);
    if (obj)
    {
        ss << "ID : " << obj->getId() << std::endl;
        ss << "Type : " << obj->getTypeAsString() << std::endl;

        ss << "Attributes : " << std::endl;
        citygml::AttributesMap attribs = obj->getAttributes();
        citygml::AttributesMap::const_iterator it = attribs.begin();
        while (it != attribs.end())
        {
            ss << " + " << it->first << ": " << it->second << std::endl;
            ++it;
        }

        // get textures
        // parse geometry
        std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
        std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
        for (; itGeom != geoms.end(); ++itGeom)
        {
            // parse polygons
            std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
            std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
            for (; itPoly != polys.end(); ++itPoly)
            {
                citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                std::vector<TVec3d>& vertices = ring->getVertices();
                std::vector<TVec3d>::iterator itVertices = vertices.begin();
                ss << "Linear ring (" << (*itGeom)->getId() << " : " << (*itPoly)->getId() << ") : ";
                for (; itVertices != vertices.end(); ++itVertices)
                {
                    // do stuff with points...
                    TVec3d point = *itVertices;
                    ss << point;
                }
                ss << std::endl;

                ss << "Texcoords : ";
                citygml::TexCoords texCoords = (*itPoly)->getTexCoords();
                for (citygml::TexCoords::const_iterator itTC = texCoords.begin(); itTC < texCoords.end(); ++itTC)
                {
                    ss << *itTC;
                }
                ss << std::endl;

                const citygml::Texture* tex = (*itPoly)->getTexture();
                if (tex)
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
void MainWindow::unlockFeatures(const QString& pass)
{
    // choose admin level depending on password
    if (pass == "pass1")
    {
        m_unlockLevel = 1;
    }
    else if (pass == "pass2")
    {
        m_unlockLevel = 2;
    }
    else
    {
        m_unlockLevel = 0;
    }

    switch (m_unlockLevel)
    {
    case 2:
        m_ui->menuDebug->menuAction()->setVisible(true);
        m_ui->menuTest->menuAction()->setVisible(true);
        m_ui->menuPlugins->menuAction()->setVisible(true);
        m_ui->actionExport_osg->setVisible(true);
        m_ui->actionExport_tiled_osga->setVisible(true);
        m_ui->actionExport_JSON->setVisible(true);
        m_ui->actionLoad_bbox->setVisible(true);
        m_ui->actionShow_advanced_tools->setVisible(true);
        m_ui->actionHelp->setVisible(true);
        m_ui->actionLOD0->setVisible(true);
        m_ui->actionLOD2->setVisible(true);
        m_ui->actionLOD3->setVisible(true);
        m_ui->actionLOD4->setVisible(true);
        m_ui->actionAll_LODs->setVisible(true);
        //m_ui->tab_16->setVisible(true);
        //break; // missing break on purpose
    case 1:
        m_ui->hsplitter_bottom->setVisible(true);
        m_ui->widgetTemporal->setVisible(true);
        m_ui->actionShow_temporal_tools->setVisible(true);
        break;
    case 0:
        m_ui->menuDebug->menuAction()->setVisible(false);
        m_ui->menuTest->menuAction()->setVisible(false); //A cacher
        m_ui->menuPlugins->menuAction()->setVisible(false); //A cacher
        m_ui->actionFix_building->setVisible(false);
        m_ui->actionShadows->setVisible(false);
        m_ui->actionExport_osg->setVisible(false);
        m_ui->actionExport_tiled_osga->setVisible(false);
        m_ui->actionExport_JSON->setVisible(false);
        m_ui->actionLoad_bbox->setVisible(false);
        m_ui->actionShow_advanced_tools->setVisible(false);
        m_ui->actionHelp->setVisible(false);
        m_ui->tab_16->setVisible(false);
        m_ui->tabWidget->removeTab(1);
        m_ui->widgetTemporal->setVisible(false); //A cacher
        m_ui->hsplitter_bottom->setVisible(false); //A cacher
        m_ui->actionShow_temporal_tools->setVisible(false); //A cacher
        m_ui->actionLOD0->setVisible(false);
        m_ui->actionLOD2->setVisible(false);
        m_ui->actionLOD3->setVisible(false);
        m_ui->actionLOD4->setVisible(false);
        m_ui->actionAll_LODs->setVisible(false);
        break;
    default:
        break;
    }
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
    unlockFeatures("pass2");
    //unlockFeatures("");
    m_ui->mainToolBar->hide();
    //m_ui->statusBar->hide();

    // TODO : need to be adjusted manually if we had other dataprofiles, should do something better

    // set dataprofile
    QSettings settings("liris", "virtualcity");
    QString dpName = settings.value("dataprofile").toString();
    if (dpName == "None")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileNone();
    }
    else if (dpName == "Paris")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileParis();
    }
    else if (dpName == "Lyon")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileLyon();
    }
    else if (dpName == "Sablons")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileSablons();
    }
    else
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileDefault();
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::resetScene()
{
    // reset scene
    m_app.getScene().reset();

    // reset osg scene
    m_osgScene->reset();

    // reset ui
    //reset();

    m_treeView->reset();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::clearSelection()
{
    appGui().getControllerGui().resetSelection();
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
void MainWindow::optionSkybox()
{
    bool isChecked = m_ui->actionSkybox->isChecked();
    m_osgScene->toggleSkybox(isChecked);

    std::cout << "Toggle Skybox" << std::endl;
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
    if (m_ui->actionShow_temporal_tools->isChecked())
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
    /*bool v = m_ui->actionShow_advanced_tools->isChecked();
    if(v)
    {
    m_ui->menuDebug->show();
    }
    else
    {
    m_ui->menuDebug->hide();
    }*/
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::initTemporalTools()
{
    QDateTime startDate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate), Qt::ISODate);
    QDateTime endDate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_endDate), Qt::ISODate);

    int max = appGui().getSettings().m_incIsDay ? startDate.daysTo(endDate) : startDate.secsTo(endDate);
    m_ui->horizontalSlider->setMaximum(max);

    m_ui->dateTimeEdit->setDisplayFormat("dd/MM/yyyy hh:mm:ss");
    m_ui->dateTimeEdit->setDateTime(startDate);
    m_ui->dateTimeEdit->setMinimumDateTime(startDate);
    m_ui->dateTimeEdit->setMaximumDateTime(endDate);

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateTemporalParams(int value)
{
    // min and max dates are controlled in the Settings.
    // default size for the temporal slider is in mainWindow.ui, in the temporal slider params
    // QAbractSlider::maximum = 109574 -> number of days in 300 years

    if (value == -1) value = m_ui->horizontalSlider->value();
    QDateTime date = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate), Qt::ISODate);
    date = appGui().getSettings().m_incIsDay ? date.addDays(value) : date.addSecs(value);
    //m_ui->buttonBrowserTemporal->setText(date.toString());
    m_ui->dateTimeEdit->setDateTime(date);

    //std::cout << "set year : " << date.year() << std::endl;

	QDateTime datetime(date);
	m_currentDate = datetime;
    if(m_useTemporal)
    {
        m_osgScene->setDate(datetime);

        //Send signal to Sunlight Plugin which will trap it if visu is activated
        emit activateVisuSunlightPlugin(m_currentDate);
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateTemporalSlider()
{
    QDateTime newdate = m_ui->dateTimeEdit->dateTime();
    int value = m_ui->horizontalSlider->value();
    QDateTime olddate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate), Qt::ISODate);
    if (appGui().getSettings().m_incIsDay)
    {
        olddate = olddate.addDays(value);
        m_ui->horizontalSlider->setValue(value + olddate.daysTo(newdate));
    }
    else
    {
        olddate = olddate.addSecs(value);
        m_ui->horizontalSlider->setValue(value + olddate.secsTo(newdate));
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::toggleUseTemporal()
{
    // min and max dates are controlled in the Settings.
    // default size for the temporal slider is in mainWindow.ui, in the temporal slider params
    // QAbractSlider::maximum = 109574 -> number of days in 300 years

    m_useTemporal = !m_useTemporal;

    if (m_useTemporal)
    {
        bool isDays = appGui().getSettings().m_incIsDay;
        QDateTime startDate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate), Qt::ISODate);
        QDateTime date(startDate);
        date = isDays ? date.addDays(m_ui->horizontalSlider->value()) : date.addSecs(m_ui->horizontalSlider->value());
        m_currentDate = date;
        m_osgScene->setDate(date);
        m_ui->dateTimeEdit->setDateTime(date);
    }
    else
    {
        // -4000 is used as a special value to disable time
        QDate date(-4000, 1, 1);
        QDateTime datetime(date);
        m_osgScene->setDate(datetime); // reset
        m_currentDate = datetime;
        m_timer.stop();
    }

    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

	//std::cout << "toggle temporal tool" << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::ChangecheckBoxTemporalToolsState()
{
    m_ui->checkBoxTemporalTools->setChecked(!m_useTemporal); //This will trigger a signal and call toggleUseTemporal function
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportCityGML()
{
    m_osgView->setActive(false);

    QString filename = QFileDialog::getSaveFileName();

    citygml::ExporterCityGML exporter(filename.toStdString());

    // check temporal params
    if (m_useTemporal)
    {
       exporter.setTemporalExport(true);
       exporter.setDate(boost::posix_time::time_from_string(
                             m_ui->dateTimeEdit->dateTime().toString().toStdString()));
    }

    // check if something is picked
    const std::vector<vcity::URI>& uris = appGui().getSelectedNodes();
    if (uris.size() > 0)
    {
        //std::cout << "Citygml export cityobject : " << uris[0].getStringURI() << std::endl;
        std::vector<const citygml::CityObject*> objs;
        std::vector<TextureCityGML*> TexturesList;

        for (const vcity::URI& uri : uris)
        {
            uri.resetCursor();
            std::cout << "export cityobject : " << uri.getStringURI() << std::endl;

            if (uri.getType() == "Building")
            {
                const citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri); // use getNode
                if (obj) objs.push_back(obj);

                for (citygml::CityObject* object : obj->getChildren())
                {
                    for (citygml::Geometry* Geometry : object->getGeometries())
                    {
                        for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                        {
                            if (PolygonCityGML->getTexture() == nullptr)
                            {
                                continue;
                            }

                            //Remplissage de ListTextures
                            std::string Url = PolygonCityGML->getTexture()->getUrl();
                            citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

                            TexturePolygonCityGML Poly;
                            Poly.Id = PolygonCityGML->getId();
                            Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                            Poly.TexUV = PolygonCityGML->getTexCoords();

                            bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                            for (TextureCityGML* Tex : TexturesList)
                            {
                                if (Tex->Url == Url)
                                {
                                    URLTest = true;
                                    Tex->ListPolygons.push_back(Poly);
                                    break;
                                }
                            }
                            if (!URLTest)
                            {
                                TextureCityGML* Texture = new TextureCityGML;
                                Texture->Wrap = WrapMode;
                                Texture->Url = Url;
                                Texture->ListPolygons.push_back(Poly);
                                TexturesList.push_back(Texture);
                            }
                        }
                    }
                }
            }
            else if (uri.getType() == "File")
            {
                citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
                for (const citygml::CityObject* obj : model->getCityObjectsRoots())
                {
                    objs.push_back(obj);

                    for (citygml::CityObject* object : obj->getChildren())
                    {
                        for (citygml::Geometry* Geometry : object->getGeometries())
                        {
                            for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                            {
                                if (PolygonCityGML->getTexture() == nullptr)
                                {
                                    continue;
                                }

                                //Remplissage de ListTextures
                                std::string Url = PolygonCityGML->getTexture()->getUrl();
                                citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

                                TexturePolygonCityGML Poly;
                                Poly.Id = PolygonCityGML->getId();
                                Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                                Poly.TexUV = PolygonCityGML->getTexCoords();

                                bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                for (TextureCityGML* Tex : TexturesList)
                                {
                                    if (Tex->Url == Url)
                                    {
                                        URLTest = true;
                                        Tex->ListPolygons.push_back(Poly);
                                        break;
                                    }
                                }
                                if (!URLTest)
                                {
                                    TextureCityGML* Texture = new TextureCityGML;
                                    Texture->Wrap = WrapMode;
                                    Texture->Url = Url;
                                    Texture->ListPolygons.push_back(Poly);
                                    TexturesList.push_back(Texture);
                                }
                            }
                        }
                    }
                }
            }
            else if (uri.getType() == "LayerCityGML")
            {
                vcity::LayerCityGML* layer = static_cast<vcity::LayerCityGML*>(m_app.getScene().getLayer(uri));

                for (vcity::Tile* tile : layer->getTiles())
                {
                    for (const citygml::CityObject* obj : tile->getCityModel()->getCityObjectsRoots())
                    {
                        objs.push_back(obj);
                        for (citygml::CityObject* object : obj->getChildren())
                        {
                            for (citygml::Geometry* Geometry : object->getGeometries())
                            {
                                for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                                {
                                    if (PolygonCityGML->getTexture() == nullptr)
                                        continue;

                                    //Remplissage de ListTextures
                                    std::string Url = PolygonCityGML->getTexture()->getUrl();
                                    citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

                                    TexturePolygonCityGML Poly;
                                    Poly.Id = PolygonCityGML->getId();
                                    Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                                    Poly.TexUV = PolygonCityGML->getTexCoords();

                                    bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                    for (TextureCityGML* Tex : TexturesList)
                                    {
                                        if (Tex->Url == Url)
                                        {
                                            URLTest = true;
                                            Tex->ListPolygons.push_back(Poly);
                                            break;
                                        }
                                    }
                                    if (!URLTest)
                                    {
                                        TextureCityGML* Texture = new TextureCityGML;
                                        Texture->Wrap = WrapMode;
                                        Texture->Url = Url;
                                        Texture->ListPolygons.push_back(Poly);
                                        TexturesList.push_back(Texture);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            uri.resetCursor();
        }
        //exporter.exportCityObject(objs);
        exporter.exportCityObjectWithListTextures(objs, &TexturesList);
    }
    else
    {
        std::cout << "Citygml export citymodel" << std::endl;
        // use first tile
        vcity::LayerCityGML* layer = dynamic_cast<vcity::LayerCityGML*>(m_app.getScene().getDefaultLayer("LayerCityGML"));
        citygml::CityModel* model = layer->getTiles()[0]->getCityModel();

        std::vector<TextureCityGML*> TexturesList;

        for (citygml::CityObject* obj : model->getCityObjectsRoots())
        {
            for (citygml::CityObject* object : obj->getChildren())
            {
                for (citygml::Geometry* Geometry : object->getGeometries())
                {
                    for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                    {
                        if (PolygonCityGML->getTexture() == nullptr)
                            continue;

                        //Remplissage de ListTextures
                        std::string Url = PolygonCityGML->getTexture()->getUrl();
                        citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

                        TexturePolygonCityGML Poly;
                        Poly.Id = PolygonCityGML->getId();
                        Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                        Poly.TexUV = PolygonCityGML->getTexCoords();

                        bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                        for (TextureCityGML* Tex : TexturesList)
                        {
                            if (Tex->Url == Url)
                            {
                                URLTest = true;
                                Tex->ListPolygons.push_back(Poly);
                                break;
                            }
                        }
                        if (!URLTest)
                        {
                            TextureCityGML* Texture = new TextureCityGML;
                            Texture->Wrap = WrapMode;
                            Texture->Url = Url;
                            Texture->ListPolygons.push_back(Poly);
                            TexturesList.push_back(Texture);
                        }
                    }
                }
            }
        }

        //exporter.exportCityModel(*model);
        exporter.exportCityModelWithListTextures(*model, &TexturesList);
    }

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportOsg()
{
    m_osgView->setActive(false);

    osg::ref_ptr<osg::Node> node = m_osgScene->m_layers;
    bool res = osgDB::writeNodeFile(*node, "scene.osg");
    std::cout << "export osg : " << res << std::endl;

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportOsga()
{
    m_osgView->setActive(false);

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

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportJSON()
{
    m_osgView->setActive(false);

    QString filename = QFileDialog::getSaveFileName();
    QFileInfo fileInfo(filename);
    filename = fileInfo.path() + "/" + fileInfo.baseName();
    citygml::ExporterJSON exporter;

    const std::vector<vcity::URI>& uris = appGui().getSelectedNodes();
    if (uris.size() > 0)
    {
        if (uris[0].getType() == "File")
        {
            citygml::CityModel* model = m_app.getScene().getTile(uris[0])->getCityModel();
            if (model) exporter.exportCityModel(*model, filename.toStdString(), "test");
        }
        else
        {
            // only tiles
        }
    }

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportOBJ()
{
    m_osgView->setActive(false);

    QString filename = QFileDialog::getSaveFileName();
    QFileInfo fileInfo(filename);
    filename = fileInfo.path() + "/" + fileInfo.baseName();
    citygml::ExporterOBJ exporter;
    exporter.setOffset(m_app.getSettings().getDataProfile().m_offset.x, m_app.getSettings().getDataProfile().m_offset.y);

    const std::vector<vcity::URI>& uris = appGui().getSelectedNodes();
    if (uris.size() > 0)
    {
        std::vector<citygml::CityObject*> objs;
        for (const vcity::URI& uri : uris)
        {
            uri.resetCursor();
            if (uri.getType() == "File")
            {
                citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
                for (citygml::CityObject* obj : model->getCityObjectsRoots())
                {
                    objs.push_back(obj);
                }
            }
            else
            {
                citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri);
                if (obj)
                {
                    objs.push_back(obj);
                }
            }
            uri.resetCursor();
        }
        exporter.exportCityObjects(objs, filename.toStdString());
    }

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportOBJsplit()
{
    m_osgView->setActive(false);

    QString filename = QFileDialog::getSaveFileName();
    QFileInfo fileInfo(filename);
    filename = fileInfo.path() + "/" + fileInfo.baseName();
    citygml::ExporterOBJ exporter;
    exporter.setOffset(m_app.getSettings().getDataProfile().m_offset.x, m_app.getSettings().getDataProfile().m_offset.y);
    exporter.addFilter(citygml::COT_All, "");
    exporter.addFilter(citygml::COT_WallSurface, "Wall");
    exporter.addFilter(citygml::COT_RoofSurface, "Roof");
    exporter.addFilter(citygml::COT_TINRelief, "Terrain");
    exporter.addFilter(citygml::COT_LandUse, "LandUse");
    exporter.addFilter(citygml::COT_Road, "Road");
    exporter.addFilter(citygml::COT_Door, "Door");
    exporter.addFilter(citygml::COT_Window, "Window");

    const std::vector<vcity::URI>& uris = appGui().getSelectedNodes();
    if (uris.size() > 0)
    {
        std::vector<citygml::CityObject*> objs;
        for (const vcity::URI& uri : uris)
        {
            uri.resetCursor();
            if (uri.getType() == "File")
            {
                citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
                for (citygml::CityObject* obj : model->getCityObjectsRoots())
                {
                    objs.push_back(obj);
                }
            }
            else
            {
                citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri);
                if (obj)
                {
                    objs.push_back(obj);
                }
            }
            uri.resetCursor();
        }
        exporter.exportCityObjects(objs, filename.toStdString());
    }

    m_osgView->setActive(true);
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
    for (std::vector<vcity::URI>::const_iterator it = appGui().getSelectedNodes().begin(); it < appGui().getSelectedNodes().end(); ++it)
    {
        vcity::log() << it->getStringURI() << "\n";
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateAllLODs()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);

    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD0OnFile()
{

}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD0()
{
    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    //OGRGeometry* LOD0 = new OGRMultiPolygon;
    OGRGeometryCollection* LOD0 = new OGRGeometryCollection;

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    if (uris.size() > 0)//Si des batiments ont ete selectionnes
    {
        // do all nodes selected
        for (std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
        {
            citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(*it);
            if (obj)
            {
                //std::cout<< "GenerateLOD0 on "<< obj->getId() << std::endl;
                OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
                double * heightmax = new double;
                double * heightmin = new double;
                generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                //SaveGeometrytoShape(Folder + "/" + obj->getId()+"_Footprint.shp", Enveloppe);
                //OGRGeometry* tmp = LOD0;
                //LOD0 = tmp->Union(Enveloppe);
                //delete tmp;

                LOD0->addGeometry(Enveloppe);

                citygml::Geometry* geom = ConvertLOD0ToCityGML(obj->getId(), Enveloppe, heightmin);
                citygml::CityObject* obj2 = new citygml::GroundSurface("Footprint");
                obj2->addGeometry(geom);
                obj->insertNode(obj2);

                appGui().getControllerGui().update(*it);

                delete Enveloppe;
                delete heightmax;
                delete heightmin;
            }
        }
    }
    else//Sinon, on genere les LOD0 de tous les batiments de la scene
    {
        int cpt = 0;
        for (vcity::Tile * tile : dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles())
        {
            for (citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
            {
                std::cout << "Avancement : " << cpt << " / " << tile->getCityModel()->getCityObjectsRoots().size() << std::endl;

                vcity::URI uri;
                uri.append(appGui().getScene().getDefaultLayer("LayerCityGML")->getName(), "LayerCityGML");
                uri.append(tile->getName(), "File");
                uri.append(obj->getId(), "Building");
                uri.setType("Building");

                if (obj)
                {
                    //std::cout<< "GenerateLOD0 on "<< obj->getId() << std::endl;
                    OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
                    double * heightmax = new double;
                    double * heightmin = new double;
                    generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                    if (!Enveloppe->IsEmpty())
                    {
                        //OGRGeometry* tmp = LOD0;
                        //LOD0 = tmp->Union(Enveloppe);
                        //delete tmp;

                        LOD0->addGeometry(Enveloppe);

                        /* citygml::Geometry* geom = ConvertLOD0ToCityGML(obj->getId(), Enveloppe, heightmin);

                        citygml::CityObject* obj2 = new citygml::GroundSurface("Footprint");
                        obj2->addGeometry(geom);
                        obj->insertNode(obj2);

                        appGui().getControllerGui().update(uri);*/

                        delete Enveloppe;
                        delete heightmax;
                        delete heightmin;
                    }
                }

                ++cpt;
            }
        }
    }
    SaveGeometrytoShape(Folder + "/LOD0.shp", LOD0);
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD1OnFile()
{
    //Generate LOD1 on files and export results in Folder

    m_osgView->setActive(false); // reduce osg framerate to have better response in Qt ui (it would be better if ui was threaded)

    std::cout << "Load Scene" << std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(this, "Selectionner les fichiers a traiter", lastdir);

    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);

    for (int i = 0; i < filenames.count(); ++i)
    {
        QFileInfo file(filenames[i]);
        QString filepath = file.absoluteFilePath();
        QFileInfo file2(filepath);

        if (!file2.exists())
        {
            std::cout << "Erreur : Le fichier " << filepath.toStdString() << " n'existe plus." << std::endl;
            continue;
        }
        settings.setValue("lastdir", file.dir().absolutePath());

        QString ext = file2.suffix().toLower();
        if (ext == "citygml" || ext == "gml")
        {
            citygml::CityModel* ModelOut = new citygml::CityModel;

            std::cout << "load citygml file : " << filepath.toStdString() << std::endl;
            vcity::Tile* tile = new vcity::Tile(filepath.toStdString());

            //Generate LOD1 on tile and save in CityGML File

            citygml::ExporterCityGML exporter(Folder + "/" + file.baseName().toStdString() + "_LOD1.gml");

            for (citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
            {
                if (obj)
                {
                    std::cout << "Generate LOD1 on " << obj->getId() << std::endl;
                    OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
                    double * heightmax = new double;
                    double * heightmin = new double;
                    generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                    citygml::CityObject* LOD1 = ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

                    ModelOut->addCityObject(LOD1);
                    ModelOut->addCityObjectAsRoot(LOD1);

                    delete Enveloppe;
                    delete heightmax;
                    delete heightmin;
                }
            }

            delete tile;
            ModelOut->computeEnvelope(),
                exporter.exportCityModel(*ModelOut);

            delete ModelOut;
            std::cout << "Fichier " << file.baseName().toStdString() + "_LOD1.gml cree dans " << Folder << std::endl;
        }
    }

    QApplication::restoreOverrideCursor();
    m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)

    //Generate LOD1 on files and export LOD1+LOD2 in Folder

    /* std::cout<<"Load Scene"<<std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(this, "Selectionner les fichiers a traiter", lastdir);

    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
    std::cout << "Annulation : Dossier non valide." << std::endl;
    return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    for(int i = 0; i < filenames.count(); ++i)
    {
    QFileInfo file(filenames[i]);
    QString filepath = file.absoluteFilePath();
    QFileInfo file2(filepath); //Ces deux dernieres lignes servent a transformer les \ en / dans le chemin de file.

    if(!file2.exists())
    {
    std::cout << "Erreur : Le fichier " << filepath.toStdString() <<" n'existe plus." << std::endl;
    continue;
    }
    settings.setValue("lastdir", file.dir().absolutePath());

    QString ext = file2.suffix().toLower();
    if(ext == "citygml" || ext == "gml")
    {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    std::cout << "load citygml file : " << filepath.toStdString() << std::endl;
    vcity::Tile* tile = new vcity::Tile(filepath.toStdString());

    //Generate LOD1 on tile and save in CityGML File

    citygml::ExporterCityGML exporter(Folder + "/" + file.baseName().toStdString() +"_LOD1_LOD2.gml");
    exporter.initExport();
    citygml::Envelope Envelope;

    for(citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
    {
    if(obj)
    {
    std::cout<< "Generate LOD1 on "<< obj->getId() << std::endl;
    OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
    double * heightmax = new double;
    double * heightmin = new double;
    generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

    citygml::CityObject* LOD1 = ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

    exporter.appendCityObject(*LOD1);
    LOD1->computeEnvelope();
    Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et a mesure pour l'exporter a la fin dans le fichier CityGML.

    delete Enveloppe;
    delete heightmax;
    delete heightmin;
    }
    }
    exporter.addEnvelope(Envelope);
    exporter.endExport();
    std::cout << "Fichier " << file.baseName().toStdString() + "_LOD1_LOD2.gml cree dans " << Folder << std::endl;
    QApplication::restoreOverrideCursor();
    }
    }*/
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD1()
{
    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);

    citygml::ExporterCityGML exporter(Folder + "/" + appGui().getScene().getDefaultLayer("LayerCityGML")->getName() + ".gml");
    exporter.initExport();

    citygml::Envelope Envelope;
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    if (uris.size() > 0)//Si des batiments ont ete selectionnes
    {
        // do all nodes selected
        for (std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
        {
            citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(*it);
            if (obj)
            {
                std::cout << "GenerateLOD1 on " << obj->getId() << std::endl;
                OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
                double * heightmax = new double;
                double * heightmin = new double;
                generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                citygml::CityObject* LOD1 = ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

                exporter.appendCityObject(*LOD1);
                LOD1->computeEnvelope();
                Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et a mesure pour l'exporter a la fin dans le fichier CityGML.
                //appGui().getControllerGui().update(*it);

                delete Enveloppe;
                delete heightmax;
                delete heightmin;
            }
        }
    }
    else//Sinon, on genere les LOD1 de tous les batiments de la scene
    {
        int i = 0;
        for (vcity::Tile * tile : dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles())
        {
            for (citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
            {
                vcity::URI uri;
                uri.append(appGui().getScene().getDefaultLayer("LayerCityGML")->getName(), "LayerCityGML");
                uri.append(tile->getName(), "File");
                uri.append(obj->getId(), "Building");
                uri.setType("Building");

                //std::cout << uri.getStringURI() << std::endl;

                if (obj)
                {
                    std::cout << "GenerateLOD1 on " << obj->getId() << std::endl;
                    OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
                    double * heightmax = new double;
                    double * heightmin = new double;
                    generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                    citygml::CityObject* LOD1 = ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

                    exporter.appendCityObject(*LOD1);
                    LOD1->computeEnvelope();
                    Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et a mesure pour l'exporter a la fin dans le fichier CityGML.
                    //appGui().getControllerGui().update(uri);
                    ++i;

                    delete Enveloppe;
                    delete heightmax;
                    delete heightmin;
                }
            }
        }
        if (i == 0)
        {
            std::cout << "Erreur : Aucun batiment dans la scene." << std::endl;
            QApplication::restoreOverrideCursor();
            exporter.endExport();
            return;
        }
    }
    exporter.addEnvelope(Envelope);
    exporter.endExport();
    std::cout << "Fichier " << appGui().getScene().getDefaultLayer("LayerCityGML")->getName() + ".gml cree dans " + Folder << std::endl;

    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD2()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD3()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::generateLOD4()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotFixBuilding()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    vcity::app().getAlgo2().fixBuilding(uris);

    // TODO
    //appGui().getControllerGui().update(uri);
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotSplitCityGMLBuildings()
{
    m_osgView->setActive(false); // reduce osg framerate to have better response in Qt ui (it would be better if ui was threaded)

    std::cout << "Load Scene" << std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(this, "Selectionner les fichiers a traiter", lastdir);

    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);


    QTime time;
    time.start();

    for (int i = 0; i < filenames.count(); ++i)
    {
        QFileInfo file(filenames[i]);
        QString filepath = file.absoluteFilePath();
        QFileInfo file2(filepath);

        if (!file2.exists())
        {
            std::cout << "Erreur : Le fichier " << filepath.toStdString() << " n'existe plus." << std::endl;
            continue;
        }
        settings.setValue("lastdir", file.dir().absolutePath());

        QString ext = file2.suffix().toLower();
        if (ext == "citygml" || ext == "gml")
        {
            std::cout << "Debut du traitement sur : " << file.baseName().toStdString() << std::endl;
            vcity::Tile* BatiLOD2CityGML = new vcity::Tile(filepath.toStdString());

            std::vector<TextureCityGML*> ListTextures;

            citygml::CityModel* ModelOut = SplitBuildingsFromCityGML(BatiLOD2CityGML, &ListTextures);

            delete BatiLOD2CityGML;

            ModelOut->computeEnvelope();
            citygml::ExporterCityGML exporter(Folder + "/" + file.baseName().toStdString() + "_SplitBuildings.gml");

            exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

            std::cout << Folder + "/" + file.baseName().toStdString() + "_Split.gml a ete cree." << std::endl;

            delete ModelOut;

            for (TextureCityGML* Tex : ListTextures)
                delete Tex;
        }
    }

    int millisecondes = time.elapsed();
    std::cout << "Execution time : " << millisecondes / 1000.0 << std::endl;

    QApplication::restoreOverrideCursor();
    m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)

    return;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotCutCityGMLwithShapefile()
{
    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML a traiter.", lastdir);
    QFileInfo file1(filename1);
    QString filepath1 = file1.absoluteFilePath();
    QString ext1 = file1.suffix().toLower();
    if (ext1 != "citygml" && ext1 != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file1.dir().absolutePath());

    lastdir = settings.value("lastdir").toString();
    QString filename2 = QFileDialog::getOpenFileName(this, "Selectionner le fichier Shapefile.", lastdir);
    QFileInfo file2(filename2);
    QString filepath2 = file2.absoluteFilePath();
    QString ext2 = file2.suffix().toLower();
    if (ext2 != "shp")
    {
        std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file2.dir().absolutePath());


    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    vcity::Tile* BatiLOD2CityGML = new vcity::Tile(filepath1.toStdString());

    OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open(filepath2.toStdString().c_str(), FALSE);

    QApplication::setOverrideCursor(Qt::WaitCursor);

    QTime time;
    time.start();

    std::vector<TextureCityGML*> ListTextures;
    citygml::CityModel* ModelOut = CutCityGMLwithShapefile(BatiLOD2CityGML, BatiShapeFile, &ListTextures);

    delete BatiShapeFile;

    ModelOut->computeEnvelope();

    citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString() + "_CutBuildings.gml");

    exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

    for (TextureCityGML* Tex : ListTextures)
        delete Tex;

    delete BatiLOD2CityGML;
    //delete ModelOut; // !!!!!!!!!!!! On ne peut pas delete BatiLOD2CityGML et ModelOut car on a recupere des batiments tels quels du premier pour les mettre dans le second (ceux qui n'ont pas d'equivalents dans le Shapefile). Du coup ce n'est pas propre (fuite memoire) mais il n'y a a pas de clone() sur les Cityobject...
    //////////
    int millisecondes = time.elapsed();
    std::cout << "Execution time : " << millisecondes / 1000.0 << std::endl;

    std::cout << Folder + "/" + file1.baseName().toStdString() + "_CutBuildings.gml a ete cree." << std::endl;

    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotObjToCityGML()
{
    m_osgView->setActive(false);
    DialogConvertObjToCityGML diag(m_app.getSettings().getDataProfile().m_offset.x, m_app.getSettings().getDataProfile().m_offset.y);
    diag.exec();

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotChangeDetection()
{
    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);

    /*const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

    if(tiles.size() != 2)
    {
    std::cout << "Erreur : Il faut ouvrir deux fichiers CityGML de la meme zone, en commencant par le plus ancien." << std::endl;
    QApplication::restoreOverrideCursor();
    return;
    }

    CompareTiles(Folder, tiles[0]->getCityModel(),tiles[1]->getCityModel());*/ //COMPARE DEUX FICHIERS OUVERTS DANS 3DUSE

    // Ouvre deux fichiers juste pour ce traitement

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML de la premiere date.", lastdir);
    QFileInfo file1(filename1);
    QString filepath1 = file1.absoluteFilePath();
    QString ext1 = file1.suffix().toLower();
    if (ext1 != "citygml" && ext1 != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file1.dir().absolutePath());

    lastdir = settings.value("lastdir").toString();
    QString filename2 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML de la seconde date.", lastdir);
    QFileInfo file2(filename2);
    QString filepath2 = file2.absoluteFilePath();
    QString ext2 = file2.suffix().toLower();
    if (ext2 != "citygml" && ext2 != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file2.dir().absolutePath());

    vcity::Tile* tile1 = new vcity::Tile(filepath1.toStdString());
    std::cout << "Le fichier " << filepath1.toStdString() << " a ete charge." << std::endl;
    vcity::Tile* tile2 = new vcity::Tile(filepath2.toStdString());
    std::cout << "Le fichier " << filepath2.toStdString() << " a ete charge." << std::endl;

    ChangeDetectionRes Res = CompareTiles(Folder, tile1->getCityModel(), tile2->getCityModel());

    SaveGeometrytoShape(Folder + "/BatisOld.shp", Res.EnveloppeCityU1);
    SaveGeometrytoShape(Folder + "/BatisNew.shp", Res.EnveloppeCityU2);
    SaveGeometrytoShape(Folder + "/BatisCrees.shp", Res.BatiCrees);
    SaveGeometrytoShape(Folder + "/BatisDetruits.shp", Res.BatiDetruits);
    SaveGeometrytoShape(Folder + "/BatisModifiesOld.shp", Res.BatiModifies1);
    SaveGeometrytoShape(Folder + "/BatisModifiesNew.shp", Res.BatiModifies2);
    SaveGeometrytoShape(Folder + "/BatisInchanges.shp", Res.BatiInchanges);

    delete Res.BatiInchanges;
    delete Res.BatiModifies2;
    delete Res.BatiModifies1;
    delete Res.BatiCrees;
    delete Res.BatiDetruits;

    delete Res.EnveloppeCityU1;
    delete Res.EnveloppeCityU2;

    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::TilingCityGML(QString CityGMLPath, std::string OutputPath, int TileX, int TileY) //BIEN PENSER A METTRE LES DOSSIER DE TEXTURE AVEC LES CITYGML POUR LES MNT AVEC DES TEXTURE WORLD
{
    //CityGMLPath = "D:/Donnees/Data/CityGML Grand Lyon/2012_DonneesVisibilite/Test";
    //OutputPath = "D:/Donnees/Data/CityGML Grand Lyon/2012_DonneesVisibilite/Test2";

    CPLPushErrorHandler(CPLQuietErrorHandler); //POUR CACHER LES WARNING DE GDAL

    QTime time;
    time.start();

    QDir dir(CityGMLPath);
    QStringList list;

    QDirIterator iterator(dir.absolutePath(), QDirIterator::Subdirectories);
    while (iterator.hasNext())
    {
        iterator.next();
        if (!iterator.fileInfo().isDir())
        {
            QString filename = iterator.filePath();
            if (filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
                list.append(filename);
        }
    }
    std::cout << list.size() << " fichier(s) CityGML trouve(s)." << std::endl;

    int cpt = 0;
    for (QString Path : list)
    {
        vcity::Tile* Tile = new vcity::Tile(Path.toStdString());
        TVec3d Lower = Tile->getEnvelope().getLowerBound();
        TVec3d Upper = Tile->getEnvelope().getUpperBound();

        std::cout << "Fichier " << Path.toStdString() << std::endl;

        TVec2d MinTile((int)(Lower.x / TileX) * TileX, (int)(Lower.y / TileY) * TileY);
        TVec2d MaxTile((int)(Upper.x / TileX) * TileX, (int)(Upper.y / TileY) * TileY);

        std::cout << Lower << std::endl;
        std::cout << Upper << std::endl;

        std::cout << MinTile << std::endl;
        std::cout << MaxTile << std::endl;

        for (int x = (int)MinTile.x; x <= (int)MaxTile.x; x += TileX)
        {
            for (int y = (int)MinTile.y; y <= (int)MaxTile.y; y += TileY)
            {
                std::vector<TextureCityGML*> TexturesList;

                std::cout << "Tuile : " << x / TileX << "_" << y / TileY << std::endl;
                citygml::CityModel* Tuile = TileCityGML(Tile, &TexturesList, TVec2d(x, y), TVec2d(x + TileX, y + TileY), CityGMLPath.toStdString().substr(0, CityGMLPath.toStdString().find_last_of("/")));

                std::string FileName = OutputPath + "/" + std::to_string((int)(x / TileX)) + "_" + std::to_string((int)(y / TileY)) + ".gml";

                FILE * fp = fopen(FileName.c_str(), "rb");
                if (fp == nullptr) //Le fichier correspondant a la tuile courante n'existe pas, on peut donc le creer
                {
                    citygml::ExporterCityGML exporter(FileName);
                    Tuile->computeEnvelope();
                    exporter.exportCityModelWithListTextures(*Tuile, &TexturesList);
                }
                else // Cette tuile existe deja, il faut donc la fusionner avec la nouvelle decoupe
                {
                    fclose(fp);
                    //std::cout << "Le fichier existe deja" << std::endl;
                    vcity::Tile* OldTile = new vcity::Tile(FileName);
                    MergingTile(OldTile, Tuile, &TexturesList);

                    Tuile->computeEnvelope();
                    citygml::ExporterCityGML exporter(FileName);
                    exporter.exportCityModelWithListTextures(*Tuile, &TexturesList);
                }
                delete Tuile;
                for (TextureCityGML* Tex : TexturesList)
                    delete Tex;
            }
        }
        delete Tile;

        ++cpt;
        std::cout << cpt << " fichier(s) traite(s)." << std::endl;
    }

    int millisecondes = time.elapsed();
    std::cout << "Execution time : " << millisecondes / 1000.0 << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotOptimOSG()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    appGui().getOsgScene()->optim();
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD0()
{
    appGui().getOsgScene()->forceLOD(0);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotRenderLOD1()
{
    //QTime time;
    //time.start();
    appGui().getOsgScene()->forceLOD(1);
    //int millisecondes = time.elapsed();
    //std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;
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
    if (m_temporalAnim)
    {
        m_timer.start(1000); // anim each 500ms
        m_ui->toolButton->setIcon(QIcon::fromTheme("media-playback-pause"));
        m_ui->toolButton->setToolTip("Pause temporal animation");
    }
    else
    {
        m_timer.stop();
        m_ui->toolButton->setIcon(QIcon::fromTheme("media-playback-start"));
        m_ui->toolButton->setToolTip("Start temporal animation");
    }
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotTemporalAnimUpdate()
{
    // increase by a year
    int incr = appGui().getSettings().m_incSize;
    m_ui->horizontalSlider->setValue(m_ui->horizontalSlider->value() + incr);
    //std::cout << m_ui->horizontalSlider->value() << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::about()
{
    DialogAbout diag;
    diag.exec();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotTilingCityGML()
{
    DialogTilingCityGML diag;
    diag.exec();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotBuildBuildingAABBs()
{
    DialogBuildBuildingAABBs diag;
    diag.exec();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotCutMNTwithShapefile()
{
    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML a traiter.", lastdir);
    QFileInfo file1(filename1);
    QString filepath1 = file1.absoluteFilePath();
    QString ext1 = file1.suffix().toLower();
    if (ext1 != "citygml" && ext1 != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file1.dir().absolutePath());

    lastdir = settings.value("lastdir").toString();
    QString filename2 = QFileDialog::getOpenFileName(this, "Selectionner le fichier Shapefile contenant les polygones de decoupe.", lastdir);
    QFileInfo file2(filename2);
    QString filepath2 = file2.absoluteFilePath();
    QString ext2 = file2.suffix().toLower();
    if (ext2 != "shp")
    {
        std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file2.dir().absolutePath());


    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    vcity::Tile* MNT = new vcity::Tile(filepath1.toStdString());

    OGRDataSource* CutPolygons = OGRSFDriverRegistrar::Open(filepath2.toStdString().c_str(), FALSE);

    QApplication::setOverrideCursor(Qt::WaitCursor);

    QTime time;
    time.start();

    std::vector<TextureCityGML*> ListTextures;
    citygml::CityModel* ModelOut = CutMNTwithShapefile(MNT, CutPolygons, &ListTextures);

    ModelOut->computeEnvelope();

    citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString() + "_" + file2.baseName().toStdString() + ".gml");

    exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

    int millisecondes = time.elapsed();
    std::cout << "Traitement termine, fichier MNT decoupe cree. Execution time : " << millisecondes / 1000.0 << std::endl;

    QApplication::restoreOverrideCursor();

    delete MNT;
    delete CutPolygons;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotCreateRoadOnMNT()
{
    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML a traiter.", lastdir);
    QFileInfo file1(filename1);
    QString filepath1 = file1.absoluteFilePath();
    QString ext1 = file1.suffix().toLower();
    if (ext1 != "citygml" && ext1 != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file1.dir().absolutePath());

    lastdir = settings.value("lastdir").toString();
    QString filename2 = QFileDialog::getOpenFileName(this, "Selectionner le fichier Shapefile contenant le reseau routier.", lastdir);
    QFileInfo file2(filename2);
    QString filepath2 = file2.absoluteFilePath();
    QString ext2 = file2.suffix().toLower();
    if (ext2 != "shp")
    {
        std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file2.dir().absolutePath());


    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    vcity::Tile* MNT = new vcity::Tile(filepath1.toStdString());

    OGRDataSource* Roads = OGRSFDriverRegistrar::Open(filepath2.toStdString().c_str(), FALSE);

    QApplication::setOverrideCursor(Qt::WaitCursor);

    QTime time;
    time.start();

    std::vector<TextureCityGML*> ListTextures_Roads;
    std::vector<TextureCityGML*> ListTextures_Ground;
    citygml::CityModel* MNT_roads = new citygml::CityModel;
    citygml::CityModel* MNT_grounds = new citygml::CityModel;

    CreateRoadsOnMNT(MNT, Roads, MNT_roads, &ListTextures_Roads, MNT_grounds, &ListTextures_Ground);

    MNT_roads->computeEnvelope();
    MNT_grounds->computeEnvelope();

    citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString() + "_MNT_Roads.gml");

    exporter.exportCityModelWithListTextures(*MNT_roads, &ListTextures_Roads);

    citygml::ExporterCityGML exporter2(Folder + "/" + file1.baseName().toStdString() + "_MNT_Ground.gml");

    exporter2.exportCityModelWithListTextures(*MNT_grounds, &ListTextures_Ground);

    int millisecondes = time.elapsed();
    std::cout << "Traitement termine, fichier MNT road cree. Execution time : " << millisecondes / 1000.0 << std::endl;

    QApplication::restoreOverrideCursor();

    delete MNT;
    delete Roads;
}

////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotCreateVegetationOnMNT()
{
    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML a traiter.", lastdir);
    QFileInfo file1(filename1);
    QString filepath1 = file1.absoluteFilePath();
    QString ext1 = file1.suffix().toLower();
    if (ext1 != "citygml" && ext1 != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file1.dir().absolutePath());

    lastdir = settings.value("lastdir").toString();
    QString filename2 = QFileDialog::getOpenFileName(this, "Selectionner le fichier Shapefile contenant les polygones de vegetation.", lastdir);
    QFileInfo file2(filename2);
    QString filepath2 = file2.absoluteFilePath();
    QString ext2 = file2.suffix().toLower();
    if (ext2 != "shp")
    {
        std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }
    settings.setValue("lastdir", file2.dir().absolutePath());


    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    vcity::Tile* MNT = new vcity::Tile(filepath1.toStdString());

    OGRDataSource* Vegetation = OGRSFDriverRegistrar::Open(filepath2.toStdString().c_str(), FALSE);

    QApplication::setOverrideCursor(Qt::WaitCursor);

    //////////////////
    //vcity::Tile* MNT = new vcity::Tile("D:/Donnees/Data/Lyon01/LYON01_MNT.gml");

    //OGRDataSource* Roads = OGRSFDriverRegistrar::Open("D:/Donnees/Data/Lyon01/Routes_Lyon01.shp", TRUE);
    //////////////////

    QTime time;
    time.start();

    std::vector<TextureCityGML*> ListTextures_Vegetation;
    std::vector<TextureCityGML*> ListTextures_Ground;
    citygml::CityModel* MNT_vegetation = new citygml::CityModel;
    citygml::CityModel* MNT_grounds = new citygml::CityModel;

    CreateVegetationOnMNT(MNT, Vegetation, MNT_vegetation, &ListTextures_Vegetation, MNT_grounds, &ListTextures_Ground);

    MNT_vegetation->computeEnvelope();
    MNT_grounds->computeEnvelope();

    citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString() + "_MNT_Vegetation.gml");

    exporter.exportCityModelWithListTextures(*MNT_vegetation, &ListTextures_Vegetation);

    citygml::ExporterCityGML exporter2(Folder + "/" + file1.baseName().toStdString() + "_MNT_Ground.gml");

    exporter2.exportCityModelWithListTextures(*MNT_grounds, &ListTextures_Ground);

    int millisecondes = time.elapsed();
    std::cout << "Traitement termine, fichier MNT vegetation cree. Execution time : " << millisecondes / 1000.0 << std::endl;

    QApplication::restoreOverrideCursor();

    delete MNT;
    delete Vegetation;
}

////////////////////////////////////////////////////////////////////////////////
/*void buildJson()//Paris
{
QString dataPath("/mnt/docs/data/dd_backup/GIS_Data/Donnees_IGN");
//std::string basePath("/tmp/json/");
std::string basePath("/home/frederic/Documents/JSON/Paris/");
int idOffsetX = 1286;
int idOffsetY = 13714;
double offsetX = 643000.0;
double offsetY = 6857000.0;
double stepX = 500.0;
double stepY = 500.0;

QDirIterator iterator(dataPath, QDirIterator::Subdirectories);
while(iterator.hasNext())
{
iterator.next();
if(!iterator.fileInfo().isDir())
{
QString filename = iterator.filePath();
if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
{
citygml::ParserParams params;
citygml::CityModel* citygmlmodel = citygml::load(filename.toStdString(), params);
if(citygmlmodel)
{
QFileInfo fileInfo(filename);
std::string id = filename.toStdString();
id = id.substr(id.find("EXPORT_")+7);
id = id.substr(0, id.find_first_of("/\\"));
int idX = std::stoi(id.substr(0,id.find('-')));
int idY = std::stoi(id.substr(id.find('-')+1));
std::string f = "tile_" + std::to_string(idX) + '-' + std::to_string(idY);
std::cout << filename.toStdString() << " -> " << basePath+f << "\n";

std::cout << "id : " << idX << ", " << idY << std::endl;

citygml::ExporterJSON exporter;
exporter.setBasePath(basePath);
exporter.setOffset(offsetX+stepX*(idX-idOffsetX), offsetY+stepY*(idY-idOffsetY));
exporter.setTileSize(stepX, stepY);
exporter.exportCityModel(*citygmlmodel, f, id);
delete citygmlmodel;
}
}
}
}
std::cout << std::endl;
}*/
/*void buildJson()//Lyon03
{
//Error ! Mismatch type: 5TVec3IdE expected. Ring/Polygon discarded!

//QString dataPath("/home/frederic/Telechargements/Data/Lyon03/LYON03_BATI/cut"); //Decoupe Bati (attention au #if dans exportJSON.cpp)
QString dataPath("/home/frederic/Telechargements/Data/Lyon03/LYON03_MNT/cut"); //Decoupe Terrain (attention au #if dans exportJSON.cpp)

std::string basePath("/home/frederic/Documents/JSON/Lyon03/"); //Dossier de sortie
double offsetX = 1843000.0;
double offsetY = 5172500.0;
double stepX = 500.0;
double stepY = 500.0;

QDirIterator iterator(dataPath, QDirIterator::Subdirectories);
while(iterator.hasNext())
{
iterator.next();
if(!iterator.fileInfo().isDir())
{
QString filename = iterator.filePath();

if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
{
citygml::ParserParams params;
citygml::CityModel* citygmlmodel = citygml::load(filename.toStdString(), params);
if(citygmlmodel)
{
std::string id = filename.toStdString();
id = id.substr(id.find("Lyon03_")+7);
id = id.substr(0, id.find_first_of("."));
int idX = std::stoi(id.substr(0,id.find('_')));
int idY = std::stoi(id.substr(id.find('_')+1));
std::string f = "tile_" + std::to_string(idX) + '-' + std::to_string(idY);
std::cout << filename.toStdString() << " -> " << basePath+f << "\n";

id = std::to_string(idX) + "_" + std::to_string(idY);

std::cout << "id : " << idX << ", " << idY << std::endl;

citygml::ExporterJSON exporter;
exporter.setBasePath(basePath);
exporter.setPath(filename.toStdString());
exporter.setOffset(offsetX+stepX*idX, offsetY+stepY*idY);
exporter.setTileSize(stepX, stepY);
exporter.exportCityModel(*citygmlmodel, f, id);
delete citygmlmodel;
}
}
}
}
std::cout << std::endl;
}*/
/*void buildJson()//Villeurbanne
{
//Error ! Mismatch type: 5TVec3IdE expected. Ring/Polygon discarded!

//QString dataPath("/home/frederic/Telechargements/Data/VILLEURBANNE_BATIS_CITYGML/cut1000"); //Decoupe Bati (attention au #if dans exportJSON.cpp)
QString dataPath("/home/frederic/Telechargements/Data/VILLEURBANNE_MNT_CITYGML/cut1000"); //Decoupe Terrain (attention au #if dans exportJSON.cpp)

std::string basePath("/home/frederic/Documents/JSON/Villeurbanne1000/"); //Dossier de sortie
double offsetX = 1844500.0;
double offsetY = 5173500.0;
double stepX = 1000.0;//500.0;
double stepY = 1000.0;//500.0;

QDirIterator iterator(dataPath, QDirIterator::Subdirectories);
while(iterator.hasNext())
{
iterator.next();
if(!iterator.fileInfo().isDir())
{
QString filename = iterator.filePath();

if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
{
citygml::ParserParams params;
citygml::CityModel* citygmlmodel = citygml::load(filename.toStdString(), params);
if(citygmlmodel)
{
std::string id = filename.toStdString();
id = id.substr(id.find("Villeurbanne_")+13);
id = id.substr(0, id.find_first_of("."));
int idX = std::stoi(id.substr(0,id.find('_')));
int idY = std::stoi(id.substr(id.find('_')+1));
std::string f = "tile_" + std::to_string(idX) + '-' + std::to_string(idY);
std::cout << filename.toStdString() << " -> " << basePath+f << "\n";

id = std::to_string(idX) + "_" + std::to_string(idY);

std::cout << "id : " << idX << ", " << idY << std::endl;

citygml::ExporterJSON exporter;
exporter.setBasePath(basePath);
exporter.setPath(filename.toStdString());
exporter.setOffset(offsetX+stepX*idX, offsetY+stepY*idY);
exporter.setTileSize(stepX, stepY);
exporter.exportCityModel(*citygmlmodel, f, id);
delete citygmlmodel;
}
}
}
}
std::cout << std::endl;
}*/

void buildJson()//GrandLyon
{
    //QString dataPath("/home/frederic/Telechargements/Data/GrandLyon/cut_500/GrandLyon_BATI"); //Decoupe Bati (attention au #if dans exportJSON.cpp)
    QString dataPath("/home/frederic/Telechargements/Data/GrandLyon/cut_500/GrandLyon_MNT"); //Decoupe Terrain (attention au #if dans exportJSON.cpp)
    //QString dataPath("/home/frederic/Telechargements/Data/GrandLyon/cut_500/GrandLyon_BatiRemarquables"); //Decoupe BatiRemarquables (attention au #if dans exportJSON.cpp)

    std::string basePath("/home/frederic/Documents/JSON/GrandLyon/cut_500/"); //Dossier de sortie

    double stepX = 500.0;
    double stepY = 500.0;

    //double stepX = 2000.0;
    //double stepY = 2000.0;

    QDirIterator iterator(dataPath, QDirIterator::Subdirectories);
    while (iterator.hasNext())
    {
        iterator.next();
        if (!iterator.fileInfo().isDir())
        {
            QString filename = iterator.filePath();

            if (filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
            {
                citygml::ParserParams params;
                citygml::CityModel* citygmlmodel = citygml::load(filename.toStdString(), params);
                if (citygmlmodel)
                {
                    std::string id = filename.toStdString();
                    id = id.substr(id.find_last_of("/") + 1);
                    id = id.substr(id.find_first_of("_") + 1, id.find_first_of("."));
                    //std::cout << "id" << std::endl;
                    //std::cout << id.substr(0,id.find('_')) << std::endl;
                    //std::cout << id.substr(id.find('_')+1) << std::endl;
                    int idX = std::stoi(id.substr(0, id.find('_')));
                    int idY = std::stoi(id.substr(id.find('_') + 1));
                    std::string f = "tile_" + std::to_string(idX) + '-' + std::to_string(idY);
                    std::cout << filename.toStdString() << " -> " << basePath + f << "\n";

                    id = std::to_string(idX) + "_" + std::to_string(idY);

                    std::cout << "id : " << idX << ", " << idY << std::endl;

                    citygml::ExporterJSON exporter;
                    exporter.setBasePath(basePath);
                    exporter.setPath(filename.toStdString());
                    exporter.setOffset(stepX*idX, stepY*idY);
                    exporter.setTileSize(stepX, stepY);
                    exporter.exportCityModel(*citygmlmodel, f, id);
                    delete citygmlmodel;
                }
            }
        }
    }
    std::cout << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test1()
{
    QTime time;
    time.start();

    vcity::app().getAlgo().MainLidar();

    int millisecondes = time.elapsed();
    std::cout << "Execution time : " << millisecondes / 1000.0 << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test2()
{
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test3()
{
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
void MainWindow::loadShpFile(const QString& filepath)
{
    std::cout << "load shp file : " << filepath.toStdString() << std::endl;
    OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), TRUE/*FALSE*/); //False pour read only et TRUE pour pouvoir modifier

    //m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));

    if (poDS)
    {
        vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerShp")->getURI();
        appGui().getControllerGui().addShpNode(uriLayer, poDS);

        addRecentFile(filepath);

        //m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));
    }

    //OGRSFDriverRegistrar::GetRegistrar()->ReleaseDataSource(poDS);
}
