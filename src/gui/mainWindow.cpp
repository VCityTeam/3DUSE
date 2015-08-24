// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/mainWindow.hpp"
#include "ui_mainWindow.h"
#include "moc/dialogLoadBBox.hpp"
#include "moc/dialogSettings.hpp"
#include "moc/dialogAbout.hpp"
#include "moc/dialogShpLoad.hpp"

#include "controllerGui.hpp"

#include <QFileDialog>
#include <QCheckBox>
#include <QDirIterator>
#include <QSettings>
#include <QListView>
#include <QMessageBox>
#include <QDate>

#include "citygml.hpp"
#include "export/exportCityGML.hpp"
#include "export/exportJSON.hpp"
#include "export/exportOBJ.hpp"

#include "import/importerAssimp.hpp"

#include "gui/osg/osgScene.hpp"
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "ogrsf_frmts.h"
#include "osg/osgGDAL.hpp"

#include "core/BatimentShape.hpp"
#include <geos/geom/GeometryFactory.h>

/*#include "assimp/Importer.hpp"
#include "assimp/PostProcess.h"
#include "assimp/Scene.h"*/

#include "osg/osgAssimp.hpp"
#include "osg/osgMnt.hpp"

#include "utils/CityGMLFusion.h"
#include "osg/osgLas.hpp"

#include "src/processes/lodsmanagement.hpp"
#include "src/processes/ExportToShape.hpp"
#include "src/processes/ChangeDetection.hpp"
#include "src/processes/LinkCityGMLShape.hpp"
#include "src/Visibilite/Visibilite.hpp"
#include "src/Visibilite/BuildOSGNode.h"
#include "src/Visibilite/Export.hpp"
#include "src/Visibilite/data/AABB.hpp"
#include "src/processes/FlatRoof.hpp"
#include "src/visibilite/ShpExtrusion.h"
#include "src/visibilite/AlignementTree.hpp"
#include "src/visibilite/VegetTool.hpp"
#include "src/visibilite/ProcessCloudPoint.h"

#include <QPluginLoader>
#include "pluginInterface.h"
#include "moc/plugindialog.hpp"

////////////////////////////////////////////////////////////////////////////////

geos::geom::Geometry* ShapeGeo = nullptr;
std::vector<std::pair<double, double>> Hauteurs;
std::vector<BatimentShape> InfoBatiments;

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

	dialVisibilite = new DialogVisibilite(this,this);

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
    connect(m_ui->actionExport_OBJ, SIGNAL(triggered()), this, SLOT(exportOBJ()));
    connect(m_ui->actionExport_OBJ_split, SIGNAL(triggered()), this, SLOT(exportOBJsplit()));
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
	connect(m_ui->actionShp_Extrusion, SIGNAL(triggered()), this, SLOT(ShpExtrusionSlot()));

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
    connect(m_ui->actionCityGML_cut, SIGNAL(triggered()), this, SLOT(slotCityGML_cut()));
    connect(m_ui->actionOBJ_to_CityGML, SIGNAL(triggered()), this, SLOT(slotObjToCityGML()));
    connect(m_ui->actionCut_CityGML_with_Shapefile, SIGNAL(triggered()), this, SLOT(slotCutCityGMLwithShapefile()));

    connect(m_ui->actionTest_1, SIGNAL(triggered()), this, SLOT(test1()));
    connect(m_ui->actionTest_2, SIGNAL(triggered()), this, SLOT(test2()));
    connect(m_ui->actionTest_3, SIGNAL(triggered()), this, SLOT(test3()));
    connect(m_ui->actionTest_4, SIGNAL(triggered()), this, SLOT(test4()));
    connect(m_ui->actionTest_5, SIGNAL(triggered()), this, SLOT(test5()));

	connect(m_ui->actionVisibilite, SIGNAL(triggered()), dialVisibilite, SLOT(show()));

    // filter search
    connect(m_ui->filterButton, SIGNAL(clicked()), m_treeView, SLOT(slotFilter()));

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(slotTemporalAnimUpdate()));

    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

    updateRecentFiles();

    m_treeView->init();

	// plugins
    aboutPluginsAct = new QAction(tr("About &Plugins"), this);
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
    delete ShapeGeo;

	delete dialVisibilite;
    delete m_treeView;
    delete m_osgView;
    delete m_ui;
}
////////////////////////////////////////////////////////////////////////////////
// Plugins
////////////////////////////////////////////////////////////////////////////////
  void MainWindow::loadPlugins()
  {
    foreach (QObject *plugin, QPluginLoader::staticInstances())
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
    
    foreach (QString fileName, pluginsDir.entryList(QDir::Files))
    {
      if (/*fileName.contains("Filter") && */QLibrary::isLibrary(fileName))
      {
        QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
        QObject *plugin = loader.instance();
        if (plugin) {
          pluginMenu->addSeparator();
          populateMenus(plugin);
          pluginFileNames += fileName;
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
    foreach (QString text, texts) {
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
    if(list.size() >= 10)
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
    if(QDate::currentDate() > QDate(2015, 12, 31))
    {
        QMessageBox(QMessageBox::Critical,  "Error", "Expired").exec();
        return false;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    QSettings settings("liris", "virtualcity");
    QFileInfo file(filepath);

    if(!file.exists())
    {
        removeRecentFile(filepath);
        QApplication::restoreOverrideCursor();
        return false;
    }

    QString ext = file.suffix().toLower();
    if(ext == "citygml" || ext == "gml")
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
	else if(ext == "assimp" || ext == "dae" || ext == "blend" || ext == "3ds" || ext == "ase" || ext == "obj" || ext == "xgl" || ext == "ply" || ext == "dxf" || ext == "lwo" || ext == "lws" ||
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
	else if(ext == "asc")
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
	else if(ext == "las" || ext == "laz")
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
    else if(ext == "shp")
    {
        loadShpFile(filepath);
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
        QApplication::restoreOverrideCursor();
        return false;
    }

    QApplication::restoreOverrideCursor();

    return true;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadScene()
{
    m_osgView->setActive(false); // reduce osg framerate to have better response in Qt ui (it would be better if ui was threaded)

    std::cout<<"Load Scene"<<std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(this, "Load scene files", lastdir);

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

    m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)
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
            if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive) || filename.endsWith(".shp", Qt::CaseInsensitive)  || filename.endsWith(".obj", Qt::CaseInsensitive))
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

    citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(uri);
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

                ss << "Texcoords : ";
                citygml::TexCoords texCoords = (*itPoly)->getTexCoords();
                for(citygml::TexCoords::const_iterator itTC = texCoords.begin(); itTC < texCoords.end(); ++itTC)
                {
                    ss << *itTC;
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
void MainWindow::unlockFeatures(const QString& pass)
{
    // choose admin level depending on password
    if(pass == "pass1")
    {
        m_unlockLevel = 1;
    }
    else if(pass == "pass2")
    {
        m_unlockLevel = 2;
    }
    else
    {
        m_unlockLevel = 0;
    }

    switch(m_unlockLevel)
    {
    case 2:
        m_ui->menuDebug->menuAction()->setVisible(true);
        m_ui->menuTest->menuAction()->setVisible(true);
        m_ui->actionExport_osg->setVisible(true);
        m_ui->actionExport_tiled_osga->setVisible(true);
        m_ui->actionExport_JSON->setVisible(true);
        m_ui->actionLoad_bbox->setVisible(true);
        m_ui->actionShow_advanced_tools->setVisible(true);
        m_ui->actionHelp->setVisible(true);
        m_ui->actionCityGML_cut->setVisible(true);
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
        m_ui->menuTest->menuAction()->setVisible(true); //A cacher
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
        m_ui->widgetTemporal->setVisible(false);
        m_ui->hsplitter_bottom->setVisible(false);
        m_ui->actionShow_temporal_tools->setVisible(false);
        m_ui->actionCityGML_cut->setVisible(false);
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
    //unlockFeatures("pass2");
    unlockFeatures("");
    m_ui->mainToolBar->hide();
    //m_ui->statusBar->hide();

    // TODO : need to be adjusted manually if we had other dataprofiles, should do something better

    // set dataprofile
    QSettings settings("liris", "virtualcity");
    QString dpName = settings.value("dataprofile").toString();
    if(dpName == "None")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileNone();
    }
    else if(dpName == "Paris")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileParis();
    }
    else if(dpName == "Lyon")
    {
        m_app.getSettings().getDataProfile() = vcity::createDataProfileLyon();
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
void MainWindow::updateTemporalParams(int value)
{
    // date is starting at year 1900 and ending at 2100
    // this is controlled in mainWindow.ui, in the temporal slider params
    // QAbractSlider::maximum = 73049 -> number of days in 200 years

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
    // date is starting at year 1900 and ending at 2100
    // this is controlled in mainWindow.ui, in the temporal slider params
    // QAbractSlider::maximum = 73049 -> number of days in 200 years

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
        // -4000 is used as a special value to disable time
        QDate date(-4000, 1, 1);
        QDateTime datetime(date);
        m_osgScene->setDate(datetime); // reset
        m_timer.stop();
    }

    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

    //std::cout << "toggle temporal tool" << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::exportCityGML()
{
    m_osgView->setActive(false);

    QString filename = QFileDialog::getSaveFileName();

    citygml::ExporterCityGML exporter(filename.toStdString());

    // check temporal params
    if(m_useTemporal)
    {
        exporter.setTemporalExport(true);
        exporter.setDate(m_ui->dateTimeEdit->dateTime());
    }

    // check if something is picked
    const std::vector<vcity::URI>& uris = appGui().getSelectedNodes();
    if(uris.size() > 0)
    {
        //std::cout << "Citygml export cityobject : " << uris[0].getStringURI() << std::endl;
        std::vector<const citygml::CityObject*> objs;
        for(const vcity::URI& uri : uris)
        {
            uri.resetCursor();
            std::cout << "export cityobject : " << uri.getStringURI() << std::endl;
            const citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri); // use getNode

            if(obj) objs.push_back(obj);
            if(uri.getType() == "Tile")
            {
                citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
                for(const citygml::CityObject* o : model->getCityObjectsRoots())
                {
                    objs.push_back(o);
                }
            }
            if(uri.getType() == "LayerCityGML")
            {
                vcity::LayerCityGML* layer = static_cast<vcity::LayerCityGML*>(m_app.getScene().getLayer(uri));
                for(vcity::Tile* tile : layer->getTiles())
                {
                    for(const citygml::CityObject* o : tile->getCityModel()->getCityObjectsRoots())
                    {
                        objs.push_back(o);
                    }
                }
            }
            uri.resetCursor();
        }
        exporter.exportCityObject(objs);
    }
    else
    {
        std::cout << "Citygml export citymodel" << std::endl;
        // use first tile
		vcity::LayerCityGML* layer = dynamic_cast<vcity::LayerCityGML*>(m_app.getScene().getDefaultLayer("LayerCityGML"));
        citygml::CityModel* model = layer->getTiles()[0]->getCityModel();
        exporter.exportCityModel(*model);
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
    if(uris.size() > 0)
    {
        if(uris[0].getType() == "Tile")
        {
            citygml::CityModel* model = m_app.getScene().getTile(uris[0])->getCityModel();
            if(model) exporter.exportCityModel(*model, filename.toStdString(), "test");
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
    if(uris.size() > 0)
    {
        std::vector<citygml::CityObject*> objs;
        for(const vcity::URI& uri : uris)
        {
            uri.resetCursor();
            if(uri.getType() == "Tile")
            {
                citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
                for(citygml::CityObject* obj : model->getCityObjectsRoots())
                {
                    objs.push_back(obj);
                }
            }
            else
            {
                citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri);
                if(obj)
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
    if(uris.size() > 0)
    {
        std::vector<citygml::CityObject*> objs;
        for(const vcity::URI& uri : uris)
        {
            uri.resetCursor();
            if(uri.getType() == "Tile")
            {
                citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
                for(citygml::CityObject* obj : model->getCityObjectsRoots())
                {
                    objs.push_back(obj);
                }
            }
            else
            {
                citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri);
                if(obj)
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
    for(std::vector<vcity::URI>::const_iterator it = appGui().getSelectedNodes().begin(); it < appGui().getSelectedNodes().end(); ++it)
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

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

	OGRGeometry* LOD0 = new OGRMultiPolygon;

    std::string Folder = w.selectedFiles().at(0).toStdString();

	QApplication::setOverrideCursor(Qt::WaitCursor);
	// get all selected nodes (with a uri)
	const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
	if(uris.size() > 0)//Si des bâtiments ont été selectionnés
	{
		// do all nodes selected
		for(std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
		{
			citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(*it);
			if(obj)
			{
				//std::cout<< "GenerateLOD0 on "<< obj->getId() << std::endl;
				OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
				double * heightmax = new double;
				double * heightmin = new double;
                generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                SaveGeometrytoShape(Folder + "/" + obj->getId()+"_Footprint.shp", Enveloppe);
				//OGRGeometry* tmp = LOD0;
				//LOD0 = tmp->Union(Enveloppe);
				//delete tmp;

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
    else//Sinon, on genere les LOD0 de tous les bâtiments de la scène
    {
		for(vcity::Tile * tile : dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles())
		{
			for(citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
			{
				vcity::URI uri;
				uri.append(appGui().getScene().getDefaultLayer("LayerCityGML")->getName(), "LayerCityGML");
				uri.append(tile->getName(), "Tile");
				uri.append(obj->getId(), "Building");
				uri.setType("Building");

				if(obj)
				{
					//std::cout<< "GenerateLOD0 on "<< obj->getId() << std::endl;
					OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
					double * heightmax = new double;
					double * heightmin = new double;

                    generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                    //SaveGeometrytoShape(Folder + "/" + obj->getId()+"_Footprint.shp", Enveloppe);
					OGRGeometry* tmp = LOD0;
					LOD0 = tmp->Union(Enveloppe);
					delete tmp;

                   /* citygml::Geometry* geom = ConvertLOD0ToCityGML(obj->getId(), Enveloppe, heightmin);

					citygml::CityObject* obj2 = new citygml::GroundSurface("Footprint");
					obj2->addGeometry(geom);
					obj->insertNode(obj2);

					appGui().getControllerGui().update(uri);*/

					delete Enveloppe;
					delete heightmax;
					delete heightmin;

                    //std::cout<< "LOD0 genere sur "<< obj->getId() << std::endl;
				}
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

    std::cout<<"Load Scene"<<std::endl;

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
        QFileInfo file2(filepath);

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

            citygml::ExporterCityGML exporter(Folder + "/" + file.baseName().toStdString() +"_LOD1.gml");
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
                    Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et à mesure pour l'exporter à la fin dans le fichier CityGML.

                    delete Enveloppe;
                    delete heightmax;
                    delete heightmin;
                }
            }
            exporter.addEnvelope(Envelope);
            exporter.endExport();
            std::cout << "Fichier " << file.baseName().toStdString() + "_LOD1.gml cree dans " << Folder << std::endl;
            QApplication::restoreOverrideCursor();
        }
    }

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
        QFileInfo file2(filepath); //Ces deux dernières lignes servent à transformer les \ en / dans le chemin de file.

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
                    Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et à mesure pour l'exporter à la fin dans le fichier CityGML.

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

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);

	citygml::ExporterCityGML exporter(Folder + "/" + appGui().getScene().getDefaultLayer("LayerCityGML")->getName() +".gml");
	exporter.initExport();

    citygml::Envelope Envelope;
	// get all selected nodes (with a uri)
	const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
	if(uris.size() > 0)//Si des bâtiments ont été selectionnés
	{
		// do all nodes selected
		for(std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
		{
			citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(*it);
			if(obj)
			{
				std::cout<< "GenerateLOD1 on "<< obj->getId() << std::endl;
                OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
				double * heightmax = new double;
				double * heightmin = new double;
                generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                citygml::CityObject* LOD1 = ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

				exporter.appendCityObject(*LOD1);
                LOD1->computeEnvelope();
                Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et à mesure pour l'exporter à la fin dans le fichier CityGML.
				//appGui().getControllerGui().update(*it);

				delete Enveloppe;
				delete heightmax;
				delete heightmin;
			}
		}
	}
	else//Sinon, on génère les LOD1 de tous les bâtiments de la scène
	{
        int i = 0;
		for(vcity::Tile * tile : dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles())
		{
			for(citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
			{                    
				vcity::URI uri;
				uri.append(appGui().getScene().getDefaultLayer("LayerCityGML")->getName(), "LayerCityGML");
				uri.append(tile->getName(), "Tile");
				uri.append(obj->getId(), "Building");
				uri.setType("Building");
		
                //std::cout << uri.getStringURI() << std::endl;
		
				if(obj)
				{
					std::cout<< "GenerateLOD1 on "<< obj->getId() << std::endl;
					OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
					double * heightmax = new double;
					double * heightmin = new double;
                    generateLOD0fromLOD2(obj, &Enveloppe, heightmax, heightmin);

                    citygml::CityObject* LOD1 = ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

					exporter.appendCityObject(*LOD1);
                    LOD1->computeEnvelope();
                    Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et à mesure pour l'exporter à la fin dans le fichier CityGML.
					//appGui().getControllerGui().update(uri);
                    ++i;

					delete Enveloppe;
					delete heightmax;
					delete heightmin;
				}
			}
		}
        if(i == 0)
        {
            std::cout << "Erreur : Aucun batiment dans la scene." << std::endl;
            QApplication::restoreOverrideCursor();
			exporter.endExport();
            return;
        }
    }
    exporter.addEnvelope(Envelope);
    exporter.endExport();
    std::cout << "Fichier " << appGui().getScene().getDefaultLayer("LayerCityGML")->getName() +".gml cree dans " + Folder << std::endl;

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
void MainWindow::slotCityGML_cut()
{
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotCutCityGMLwithShapefile()
{
	////SPLIT BUILDINGS FROM CITYGML => AJOUTER UN BOUTON + CHOIX DES FICHIERS ET DOSSIER DE SORTIE
	/*city::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/LYON01_BATIS_WithoutTextures.gml"); //Doit ouvrir un fichier CityGML contenant des bâtiments LOD2
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Jeux de test/LYON_1ER_00136.gml");
	citygml::CityModel* ModelOut = SplitBuildingsFromCityGML(BatiLOD2CityGML);
	ModelOut->computeEnvelope();

	citygml::ExporterCityGML exporter("BatimentsDecoupes.gml");
	exporter.exportCityModel(*ModelOut);

	return;*/
    /*QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();*/

    QApplication::setOverrideCursor(Qt::WaitCursor);
	////////// Ancienne version utilisant GEOS : 
    //DecoupeCityGML(Folder, ShapeGeo, InfoBatiments);
	////////// Nouvelle version de découpe : 

    //vcity::Tile* BatiLOD2CityGML = new vcity::Tile("/home/frederic/Telechargements/Data/GrandLyon_old/Lyon01/Jeux de test/LYON_1ER_00136_BatimentsDecoupes.gml");
    //vcity::Tile* BatiLOD2CityGML = new vcity::Tile("/home/frederic/Telechargements/Data/Lyon01/Lyon01_BatimentsDecoupes.gml");

	vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Lyon01_BatimentsDecoupes.gml"); //Doit ouvrir un fichier CityGML contenant des bâtiments LOD2
    //vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Jeux de test/LYON_1ER_00136_BatimentsDecoupes.gml");

    //OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("/home/frederic/Telechargements/Data/GrandLyon_old/Lyon01/CADASTRE_SHP/BatiTest.shp", TRUE);
    //OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("/home/frederic/Telechargements/Data/Lyon01/CADASTRE_SHP/BATIS_LYON01.shp", TRUE);
	
	OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/PARCELLES_LYON01.shp", FALSE);
    //OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/BATIS_LYON01.shp", FALSE); //Doit ouvrir un fichier CityGML contenant des bâtiments LOD2
	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/BatiTest.shp", FALSE);

	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/VCity.git/VCity-build/BatiTest - Copie.shp", TRUE);

    QTime time;
    time.start();
	
	citygml::CityModel* ModelOut = CutCityGMLwithShapefile(BatiLOD2CityGML, BatiShapeFile);

	//ModelOut->computeEnvelope();
	//citygml::ExporterCityGML exporter("BatimentsDecoupes.gml");
	//exporter.exportCityModel(*ModelOut);

    // millisecondes contient le nombre de millisecondes entre l'appel à la fonction start()
    // et l'appel 0 la fonction elapsed()
    int millisecondes = time.elapsed();
    std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;

    //std::cout << "Fichier BatimentsDecoupes.gml cree" << std::endl;
	//////////
    QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotObjToCityGML()
{
    m_osgView->setActive(false);

    QStringList filenames = QFileDialog::getOpenFileNames(this, "Convert OBJ to CityGML");

    for(int i = 0; i < filenames.count(); ++i)
    {
        QFileInfo file(filenames[i]);
        QString ext = file.suffix().toLower();
        if(ext == "obj")
        {
            citygml::ImporterAssimp importer;
            importer.setOffset(m_app.getSettings().getDataProfile().m_offset.x, m_app.getSettings().getDataProfile().m_offset.y);
            citygml::CityModel* model = importer.import(file.absoluteFilePath().toStdString());

            citygml::ExporterCityGML exporter((file.path()+'/'+file.baseName()+".gml").toStdString());
            exporter.exportCityModel(*model);
        }
    }

    m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::slotChangeDetection()
{
    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
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
    if(ext1 != "citygml" && ext1 != "gml")
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
    if(ext2 != "citygml" && ext2 != "gml")
    {
		std::cout << "Erreur : Le fichier n'est pas un CityGML." << std::endl;
		QApplication::restoreOverrideCursor();
		return;
	}
    settings.setValue("lastdir", file2.dir().absolutePath());

	vcity::Tile* tile1 = new vcity::Tile(filepath1.toStdString());
	std::cout << "Le fichier " << filepath1.toStdString() <<" a ete charge." << std::endl;
	vcity::Tile* tile2 = new vcity::Tile(filepath2.toStdString());
	std::cout << "Le fichier " << filepath2.toStdString() <<" a ete charge." << std::endl;

	CompareTiles(Folder, tile1->getCityModel(), tile2->getCityModel());

	QApplication::restoreOverrideCursor();
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
        m_timer.start(500); // anim each 500ms
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
    m_ui->horizontalSlider->setValue(m_ui->horizontalSlider->value()+365);
    //std::cout << m_ui->horizontalSlider->value() << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::about()
{
    // TODO : add Liris image and text
    //QMessageBox::about(this, "VCity", "VCity is an environment editor");
    DialogAbout diag;
    diag.exec();
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

    //QString dataPath("/home/frederic/Telechargements/Data/Lyon03/LYON03_BATI/cut"); //Découpe Bati (attention au #if dans exportJSON.cpp)
    QString dataPath("/home/frederic/Telechargements/Data/Lyon03/LYON03_MNT/cut"); //Découpe Terrain (attention au #if dans exportJSON.cpp)

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

    //QString dataPath("/home/frederic/Telechargements/Data/VILLEURBANNE_BATIS_CITYGML/cut1000"); //Découpe Bati (attention au #if dans exportJSON.cpp)
    QString dataPath("/home/frederic/Telechargements/Data/VILLEURBANNE_MNT_CITYGML/cut1000"); //Découpe Terrain (attention au #if dans exportJSON.cpp)

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
    //QString dataPath("/home/frederic/Telechargements/Data/GrandLyon/cut_500/GrandLyon_BATI"); //Découpe Bati (attention au #if dans exportJSON.cpp)
    QString dataPath("/home/frederic/Telechargements/Data/GrandLyon/cut_500/GrandLyon_MNT"); //Découpe Terrain (attention au #if dans exportJSON.cpp)
    //QString dataPath("/home/frederic/Telechargements/Data/GrandLyon/cut_500/GrandLyon_BatiRemarquables"); //Découpe BatiRemarquables (attention au #if dans exportJSON.cpp)

    std::string basePath("/home/frederic/Documents/JSON/GrandLyon/cut_500/"); //Dossier de sortie

    double stepX = 500.0;
    double stepY = 500.0;

    //double stepX = 2000.0;
    //double stepY = 2000.0;

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
                    id = id.substr(id.find_last_of("/") + 1);
                    id = id.substr(id.find_first_of("_") + 1, id.find_first_of("."));
                    //std::cout << "id" << std::endl;
                    //std::cout << id.substr(0,id.find('_')) << std::endl;
                    //std::cout << id.substr(id.find('_')+1) << std::endl;
                    int idX = std::stoi(id.substr(0,id.find('_')));
                    int idY = std::stoi(id.substr(id.find('_')+1));
                    std::string f = "tile_" + std::to_string(idX) + '-' + std::to_string(idY);
                    std::cout << filename.toStdString() << " -> " << basePath+f << "\n";

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
	////// Récupère les ilots partages en ilot Bati et ilot non bati (terrain) découpés par les routes et les extrude en 3D grâce aux informations de hauteurs
	QApplication::setOverrideCursor(Qt::WaitCursor);

	OGRDataSource* Bati = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/batiout.shp", TRUE);
	OGRDataSource* Terrain = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/terrainout.shp", TRUE);

	OGRLayer *LayerBati = Bati->GetLayer(0);
	OGRLayer *LayerTerrain = Terrain->GetLayer(0);

	citygml::ExporterCityGML exporter("C:/Users/Game Trap/Downloads/Ilots.gml");
    exporter.initExport();
	citygml::Envelope Envelope;

	OGRFeature *FeatureBati;
	LayerBati->ResetReading();

	int cpt = 0;

	while((FeatureBati = LayerBati->GetNextFeature()) != NULL)
	{
		OGRMultiPolygon* GeometryBati = new OGRMultiPolygon;
		GeometryBati->addGeometry(FeatureBati->GetGeometryRef());
		double H = 20;
		double Zmin = 0;
		if(FeatureBati->GetFieldIndex("HAUTEUR") != -1)
			H = FeatureBati->GetFieldAsDouble("HAUTEUR");
		if(FeatureBati->GetFieldIndex("Z_MIN") != -1)
			Zmin = FeatureBati->GetFieldAsDouble("Z_MIN");
		double Zmax = Zmin + H;

		Zmax = Zmin;
		Zmin = Zmax-H;

		citygml::CityObject* Ilot = ConvertLOD1ToCityGML("IlotBati_" + std::to_string(cpt), GeometryBati, &Zmax, &Zmin);
		++cpt;

		exporter.appendCityObject(*Ilot);
        Ilot->computeEnvelope();
        Envelope.merge(Ilot->getEnvelope());

		delete GeometryBati;
	}

	OGRFeature *FeatureTerrain;
	LayerTerrain->ResetReading();

	cpt = 0;

	while((FeatureTerrain = LayerTerrain->GetNextFeature()) != NULL)
	{
		OGRMultiPolygon* GeometryTerrain = new OGRMultiPolygon;
		GeometryTerrain->addGeometry(FeatureTerrain->GetGeometryRef());
		double H = 1;
		double Zmin = 0;
		if(FeatureTerrain->GetFieldIndex("HAUTEUR") != -1)
			H = FeatureTerrain->GetFieldAsDouble("HAUTEUR");
		if(FeatureTerrain->GetFieldIndex("Z_MIN") != -1)
			Zmin = FeatureTerrain->GetFieldAsDouble("Z_MIN");
		double Zmax = Zmin + H;

		citygml::CityObject* Ilot = ConvertLOD1ToCityGML("IlotTerrain_" + std::to_string(cpt), GeometryTerrain, &Zmax, &Zmin);
		++cpt;

		exporter.appendCityObject(*Ilot);
        Ilot->computeEnvelope();
        Envelope.merge(Ilot->getEnvelope());

		delete GeometryTerrain;
	}

	exporter.addEnvelope(Envelope);
    exporter.endExport();

	QApplication::restoreOverrideCursor();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test2()
{
	//Création d'ilots à partir de Shapefile contenant des routes
	//OGRDataSource* Routes = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/Routes_Lyon01.shp", TRUE);
	OGRDataSource* Routes = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/BD_TOPO_DPT69/A_RESEAU_ROUTIER/ROUTE_Test.SHP", TRUE);
	OGRLayer *LayerRoutes = Routes->GetLayer(0);
	OGRFeature *FeatureRoutes;
	LayerRoutes->ResetReading();

	OGRMultiLineString* ReseauRoutier = new OGRMultiLineString;
	while((FeatureRoutes = LayerRoutes->GetNextFeature()) != NULL)
	{
		OGRGeometry* Route = FeatureRoutes->GetGeometryRef();
		
		if(Route->getGeometryType() == wkbLineString || Route->getGeometryType() == wkbLineString25D)
		{
			ReseauRoutier->addGeometry(Route);
		}
	}

	OGRGeometryCollection * ReseauPolygonize = (OGRGeometryCollection*) ReseauRoutier->Polygonize();

	OGRMultiPolygon * ReseauMP = new OGRMultiPolygon;

	for(int i = 0; i < ReseauPolygonize->getNumGeometries(); ++i)
	{
		OGRGeometry* temp = ReseauPolygonize->getGeometryRef(i);
		if(temp->getGeometryType() == wkbPolygon || temp->getGeometryType() == wkbPolygon25D)
			ReseauMP->addGeometry(temp);
	}

	SaveGeometrytoShape("ReseauRoutier.shp", ReseauMP);

	delete ReseauRoutier;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test3()
{
    //FusionTiles(); //Fusion des fichiers CityGML contenus dans deux dossiers : sert à fusionner les tiles donc deux fichiers du même nom seront fusionnés en un fichier contenant tous leurs objets à la suite.

	//// FusionLODs : prend deux fichiers modélisant les bâtiments avec deux lods différents et les fusionne en un seul
	QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList File1 = QFileDialog::getOpenFileNames(this, "Selectionner le premier fichier.", lastdir);

	QFileInfo file1temp(File1[0]);
	QString file1path = file1temp.absoluteFilePath();
	QFileInfo file1(file1path);

	QString ext = file1.suffix().toLower();
	if(ext != "citygml" && ext != "gml")
	{
		std::cout << "Erreur : Le fichier n'est pas un CityGML" << std::endl;
		return;
	}

	QStringList File2 = QFileDialog::getOpenFileNames(this, "Selectionner le second fichier.", lastdir);

	QFileInfo file2temp(File2[0]);
	QString file2path = file2temp.absoluteFilePath();
	QFileInfo file2(file2path);

	ext = file2.suffix().toLower();
	if(ext != "citygml" && ext != "gml")
	{
		std::cout << "Erreur : Le fichier n'est pas un CityGML" << std::endl;
		return;
	}

	QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }
	std::string Folder = w.selectedFiles().at(0).toStdString() + "/" + file2.baseName().toStdString() + "_Fusion.gml";

	QApplication::setOverrideCursor(Qt::WaitCursor);
	std::cout << "load citygml file : " << file1path.toStdString() << std::endl;
	vcity::Tile* tile1 = new vcity::Tile(file1path.toStdString());
	std::cout << "load citygml file : " << file2path.toStdString() << std::endl;
	vcity::Tile* tile2 = new vcity::Tile(file2path.toStdString());

	citygml::CityModel * City1 = tile1->getCityModel();
	citygml::CityModel * City2 = tile2->getCityModel();

	FusionLODs(City1, City2);

	citygml::ExporterCityGML exporter(Folder);

	exporter.exportCityModel(*City2);

	QApplication::restoreOverrideCursor();
}

#define addTree(message) appGui().getControllerGui().addAssimpNode(m_app.getScene().getDefaultLayer("LayerAssimp")->getURI(), message);

////////////////////////////////////////////////////////////////////////////////
void MainWindow::test4()
{
    //buildJson();

	/*std::vector<std::string> building;
	building.push_back("C:/VCityData/Jeux de test/LYON_1ER_00136.gml");
	
	std::vector<AnalysisResult> res = Analyse(building,m_app.getSettings().getDataProfile().m_offset,cam);

	int cpt = 0;
	for(AnalysisResult ar : res)
	{
		addTree(BuildViewshedOSGNode(ar,std::to_string(cpt)+"_"));
		addTree(BuildSkylineOSGNode(ar.skyline,std::to_string(cpt)+"_"));
		cpt++;
	}*/

	//if(p.getX() >= 1841000 && p.getX() <= 1843000 && p.getY() >= 5175000 && p.getY() <= 5177000)

	//ProcessLasShpVeget();

	//ProcessCL();

	//BuildAABB("C:/VCityData/Tile/",m_app.getSettings().getDataProfile().m_offset);

	/*LASreadOpener lasreadopener;
	lasreadopener.set_file_name("C:\VCityData\Veget\1841_5175.las");
	LASreader* lasreader = lasreadopener.open();

	OGRMultiPoint* mp = new OGRMultiPoint;

	while (lasreader->read_point())
	{
		OGRPoint* point = new OGRPoint;
		mp->addGeometry(new OGRPoint((lasreader->point).get_x(),(lasreader->point).get_y(),(lasreader->point).get_z()));
	}*/
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
        citygml::CityObject* obj = appGui().getScene().getCityObjectNode(appGui().getSelectedNodes()[0]);
        test5rec(obj);
        appGui().getControllerGui().update(appGui().getSelectedNodes()[0]);
    }
}
////////////////////////////////////////////////////////////////////////////////

void MainWindow::loadShpBbox()
{
	DialogShpLoad dial(this,this);
	dial.exec();
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadShpFile(const QString& filepath, bool useBBox, TVec2d boxMin, TVec2d boxMax)
{
	std::cout << "load shp file : " << filepath.toStdString() << std::endl;
    OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), TRUE/*FALSE*/); //False pour read only et TRUE pour pouvoir modifier

	//Pour sauvegarder un shapefile
	/*const char *pszDriverName = "ESRI Shapefile";
	OGRSFDriver *poDriver;
	OGRRegisterAll();
	poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(pszDriverName);

	if( poDriver == NULL )
	{
		printf( "%s driver not available.\n", pszDriverName );
		return false;
	}
	OGRDataSource *poDS2;
	remove("Polygon.shp");
	poDS2 = poDriver->CreateDataSource("Polygon.shp", NULL);
		
	poDS2->CopyLayer(poDS->GetLayer(0), "test");
	OGRDataSource::DestroyDataSource(poDS2);

	return false;*/

    //m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));
		
    // clean previous shapeGeo
    delete ShapeGeo;
	buildGeosShape(poDS, &ShapeGeo, &Hauteurs, &InfoBatiments);
    if(poDS)
    {
        vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerShp")->getURI();
        appGui().getControllerGui().addShpNode(uriLayer, poDS);

        addRecentFile(filepath);

        //m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));
    }

		//OGRSFDriverRegistrar::GetRegistrar()->ReleaseDataSource(poDS);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::ShpExtrusionSlot()
{
	ShpExtruction();
}