// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/mainWindow.hpp"
#include "ui_mainWindow.h"
#include "moc/dialogLoadBBox.hpp"
#include "moc/dialogSettings.hpp"
#include "moc/dialogAbout.hpp"
#include "moc/dialogTilingCityGML.hpp"

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
#include "osg/osgGDAL.hpp"

/*#include "assimp/Importer.hpp"
#include "assimp/PostProcess.h"
#include "assimp/Scene.h"*/

#include "osg/osgAssimp.hpp"
#include "src/DataStructures/DEM/osgMnt.hpp"

#include "utils/CityGMLFusion.h"
#include "osg/osgLas.hpp"

#include "src/processes/lodsmanagement.hpp"
#include "src/processes/ExportToShape.hpp"
#include "src/processes/ChangeDetection.hpp"
#include "src/processes/LinkCityGMLShape.hpp"
#include "src/processes/TilingCityGML.hpp"
#include "src/processes/EnhanceMNT.hpp"

#include <QPluginLoader>
#include "pluginInterface.h"
#include "moc/plugindialog.hpp"
#include "TiledFilesLayout.hpp"

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
	if(QDate::currentDate() > QDate(2016, 12, 31))
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

			osg::ref_ptr<osg::Node> node = las.buildLasPoints(uriLayer, - m_app.getSettings().getDataProfile().m_offset.x, - m_app.getSettings().getDataProfile().m_offset.y);

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

	// not really good here but, no choice...
	bool bHack=true;
	if (uri.getType() == "Workspace" || uri.getType() == "Version") bHack=false;
	// not really good here but, no choice...

	citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(uri, bHack);
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
		m_ui->actionLOD3->setVisible(false );
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
void MainWindow::initTemporalTools()
{
	QDateTime startDate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate),Qt::ISODate);
	QDateTime endDate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_endDate),Qt::ISODate);

	int max = appGui().getSettings().m_incIsDay?startDate.daysTo(endDate):startDate.secsTo(endDate);
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

	if(value == -1) value = m_ui->horizontalSlider->value();
	QDateTime date = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate),Qt::ISODate);
	date = appGui().getSettings().m_incIsDay?date.addDays(value):date.addSecs(value);
	//m_ui->buttonBrowserTemporal->setText(date.toString());
	m_ui->dateTimeEdit->setDateTime(date);

	//std::cout << "set year : " << date.year() << std::endl;

	QDateTime datetime(date);
	m_currentDate = datetime;
	if(m_useTemporal)   m_osgScene->setDate(datetime);
	
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::updateTemporalSlider()
{
	QDateTime newdate = m_ui->dateTimeEdit->dateTime();
	int value = m_ui->horizontalSlider->value();
	QDateTime olddate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate),Qt::ISODate);
	if (appGui().getSettings().m_incIsDay)
	{
		olddate = olddate.addDays(value);
		m_ui->horizontalSlider->setValue(value+olddate.daysTo(newdate));
	}
	else
	{
		olddate = olddate.addSecs(value);
		m_ui->horizontalSlider->setValue(value+olddate.secsTo(newdate));
	}	
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::toggleUseTemporal()
{
	// min and max dates are controlled in the Settings.
	// default size for the temporal slider is in mainWindow.ui, in the temporal slider params
	// QAbractSlider::maximum = 109574 -> number of days in 300 years

	m_useTemporal = !m_useTemporal;

	if(m_useTemporal)
	{
		bool isDays = appGui().getSettings().m_incIsDay;
		QDateTime startDate = QDateTime::fromString(QString::fromStdString(appGui().getSettings().m_startDate),Qt::ISODate);
		QDateTime date(startDate);
		date = isDays?date.addDays(m_ui->horizontalSlider->value()):date.addSecs(m_ui->horizontalSlider->value());
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
		std::vector<TextureCityGML*> TexturesList;

		for(const vcity::URI& uri : uris)
		{
			uri.resetCursor();
			std::cout << "export cityobject : " << uri.getStringURI() << std::endl;

			if(uri.getType() == "Building")
			{
				const citygml::CityObject* obj = m_app.getScene().getCityObjectNode(uri); // use getNode
				if(obj) objs.push_back(obj);

				for(citygml::CityObject* object : obj->getChildren())
				{
					for(citygml::Geometry* Geometry : object->getGeometries())
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
						{
							if(PolygonCityGML->getTexture() == nullptr)
							{
								continue;
							}

							//Remplissage de ListTextures
							std::string Url = PolygonCityGML->getTexture()->getUrl();
							citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

							TexturePolygonCityGML Poly;
							Poly.Id = PolygonCityGML->getId();
							Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
							Poly.TexUV = PolygonCityGML->getTexCoords();

							bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
							for(TextureCityGML* Tex: TexturesList)
							{
								if(Tex->Url == Url)
								{
									URLTest = true;
									Tex->ListPolygons.push_back(Poly);
									break;
								}
							}
							if(!URLTest)
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
			else if(uri.getType() == "Tile")
			{
				citygml::CityModel* model = m_app.getScene().getTile(uri)->getCityModel();
				for(const citygml::CityObject* obj : model->getCityObjectsRoots())
				{
					objs.push_back(obj);

					for(citygml::CityObject* object : obj->getChildren())
					{
						for(citygml::Geometry* Geometry : object->getGeometries())
						{
							for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
							{
								if(PolygonCityGML->getTexture() == nullptr)
								{
									continue;
								}

								//Remplissage de ListTextures
								std::string Url = PolygonCityGML->getTexture()->getUrl();
								citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

								TexturePolygonCityGML Poly;
								Poly.Id = PolygonCityGML->getId();
								Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
								Poly.TexUV = PolygonCityGML->getTexCoords();

								bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
								for(TextureCityGML* Tex: TexturesList)
								{
									if(Tex->Url == Url)
									{
										URLTest = true;
										Tex->ListPolygons.push_back(Poly);
										break;
									}
								}
								if(!URLTest)
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
			else if(uri.getType() == "LayerCityGML")
			{
				vcity::LayerCityGML* layer = static_cast<vcity::LayerCityGML*>(m_app.getScene().getLayer(uri));

				for(vcity::Tile* tile : layer->getTiles())
				{
					for(const citygml::CityObject* obj : tile->getCityModel()->getCityObjectsRoots())
					{
						objs.push_back(obj);
						for(citygml::CityObject* object : obj->getChildren())
						{
							for(citygml::Geometry* Geometry : object->getGeometries())
							{
								for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
								{
									if(PolygonCityGML->getTexture() == nullptr)
										continue;

									//Remplissage de ListTextures
									std::string Url = PolygonCityGML->getTexture()->getUrl();
									citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

									TexturePolygonCityGML Poly;
									Poly.Id = PolygonCityGML->getId();
									Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
									Poly.TexUV = PolygonCityGML->getTexCoords();

									bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
									for(TextureCityGML* Tex: TexturesList)
									{
										if(Tex->Url == Url)
										{
											URLTest = true;
											Tex->ListPolygons.push_back(Poly);
											break;
										}
									}
									if(!URLTest)
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

		for(citygml::CityObject* obj : model->getCityObjectsRoots())
		{
			for(citygml::CityObject* object : obj->getChildren())
			{
				for(citygml::Geometry* Geometry : object->getGeometries())
				{
					for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
					{
						if(PolygonCityGML->getTexture() == nullptr)
							continue;

						//Remplissage de ListTextures
						std::string Url = PolygonCityGML->getTexture()->getUrl();
						citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

						TexturePolygonCityGML Poly;
						Poly.Id = PolygonCityGML->getId();
						Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
						Poly.TexUV = PolygonCityGML->getTexCoords();

						bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
						for(TextureCityGML* Tex: TexturesList)
						{
							if(Tex->Url == Url)
							{
								URLTest = true;
								Tex->ListPolygons.push_back(Poly);
								break;
							}
						}
						if(!URLTest)
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

	//OGRGeometry* LOD0 = new OGRMultiPolygon;
	OGRGeometryCollection* LOD0 = new OGRGeometryCollection;

	std::string Folder = w.selectedFiles().at(0).toStdString();

	QApplication::setOverrideCursor(Qt::WaitCursor);
	// get all selected nodes (with a uri)
	const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
	if(uris.size() > 0)//Si des batiments ont ete selectionnes
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
		for(vcity::Tile * tile : dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles())
		{
			for(citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
			{
				std::cout << "Avancement : " << cpt << " / " << tile->getCityModel()->getCityObjectsRoots().size() << std::endl;

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

					if(!Enveloppe->IsEmpty())
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

	QApplication::setOverrideCursor(Qt::WaitCursor);

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
			citygml::CityModel* ModelOut = new citygml::CityModel;

			std::cout << "load citygml file : " << filepath.toStdString() << std::endl;
			vcity::Tile* tile = new vcity::Tile(filepath.toStdString());

			//Generate LOD1 on tile and save in CityGML File

			citygml::ExporterCityGML exporter(Folder + "/" + file.baseName().toStdString() +"_LOD1.gml");

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
	if(uris.size() > 0)//Si des batiments ont ete selectionnes
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
					Envelope.merge(LOD1->getEnvelope()); //On remplit l'envelope au fur et a mesure pour l'exporter a la fin dans le fichier CityGML.
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
void MainWindow::slotSplitCityGMLBuildings()
{
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

	QApplication::setOverrideCursor(Qt::WaitCursor);


	QTime time;
	time.start();

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
			std::cout << "Debut du traitement sur : " << file.baseName().toStdString() << std::endl;
			vcity::Tile* BatiLOD2CityGML = new vcity::Tile(filepath.toStdString());

			std::vector<TextureCityGML*> ListTextures;

			citygml::CityModel* ModelOut = SplitBuildingsFromCityGML(BatiLOD2CityGML, &ListTextures);

			delete BatiLOD2CityGML;

			ModelOut->computeEnvelope();
			citygml::ExporterCityGML exporter(Folder + "/" + file.baseName().toStdString()  + "_SplitBuildings.gml");

			exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

			std::cout << Folder + "/" + file.baseName().toStdString()  + "_Split.gml a ete cree." << std::endl;

			delete ModelOut;

			for(TextureCityGML* Tex:ListTextures)
				delete Tex;
		}
	}

	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;

	QApplication::restoreOverrideCursor();
	m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)


	/*QSettings settings("liris", "virtualcity");
	QString lastdir = settings.value("lastdir").toString();
	QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML a traiter.", lastdir);
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


	QFileDialog w;
	w.setWindowTitle("Selectionner le dossier de sortie");
	w.setFileMode(QFileDialog::Directory);

	if(w.exec() == 0)
	{
	std::cout << "Annulation : Dossier non valide." << std::endl;
	return;
	}

	std::string Folder = w.selectedFiles().at(0).toStdString();

	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/LYON01_BATIS.gml");
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/LYON01_BATIS_WithoutTextures.gml"); //Doit ouvrir un fichier CityGML contenant des batiments LOD2
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Jeux de test/LYON_1ER_00136.gml");

	QTime time;
	time.start();

	vcity::Tile* BatiLOD2CityGML = new vcity::Tile(filepath1.toStdString());

	std::vector<TextureCityGML*> ListTextures;

	citygml::CityModel* ModelOut = SplitBuildingsFromCityGML(BatiLOD2CityGML, &ListTextures);

	delete BatiLOD2CityGML;

	ModelOut->computeEnvelope();
	citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString()  + "_SplitBuildings.gml");
	//exporter.exportCityModel(*ModelOut);

	exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

	delete ModelOut;

	for(TextureCityGML* Tex:ListTextures)
	delete Tex;

	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;

	std::cout << Folder + "/" + file1.baseName().toStdString()  + "_Split.gml a ete cree." << std::endl;*/

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
	if(ext1 != "citygml" && ext1 != "gml")
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
	if(ext2 != "shp")
	{
		std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
		QApplication::restoreOverrideCursor();
		return;
	}
	settings.setValue("lastdir", file2.dir().absolutePath());


	QFileDialog w;
	w.setWindowTitle("Selectionner le dossier de sortie");
	w.setFileMode(QFileDialog::Directory);

	if(w.exec() == 0)
	{
		std::cout << "Annulation : Dossier non valide." << std::endl;
		return;
	}

	std::string Folder = w.selectedFiles().at(0).toStdString();

	vcity::Tile* BatiLOD2CityGML = new vcity::Tile(filepath1.toStdString());

	OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open(filepath2.toStdString().c_str(), FALSE);

	QApplication::setOverrideCursor(Qt::WaitCursor);

	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("/home/frederic/Telechargements/Data/GrandLyon_old/Lyon01/Jeux de test/LYON_1ER_00136_BatimentsDecoupes.gml");
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("/home/frederic/Telechargements/Data/GrandLyon_old/Lyon01/Jeux de test/LYON_1ER_00136_BatimentsDecoupes_Textures.gml");
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("/home/frederic/Telechargements/Data/Lyon01/Lyon01_BatimentsDecoupes.gml");

	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Lyon01_BatimentsDecoupes.gml"); //Doit ouvrir un fichier CityGML contenant des batiments LOD2
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Lyon01_BatimentsDecoupes_Textures.gml"); //Doit ouvrir un fichier CityGML contenant des batiments LOD2
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Jeux de test/LYON_1ER_00136_BatimentsDecoupes.gml");
	//vcity::Tile* BatiLOD2CityGML = new vcity::Tile("C:/Users/Game Trap/Downloads/Data/Lyon01/Jeux de test/LYON_1ER_00136_BatimentsDecoupes_Textures.gml");

	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("/home/frederic/Telechargements/Data/GrandLyon_old/Lyon01/CADASTRE_SHP/BatiTest.shp", TRUE);
	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("/home/frederic/Telechargements/Data/Lyon01/CADASTRE_SHP/BATIS_LYON01.shp", TRUE);

	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/PARCELLES_LYON01.shp", FALSE);
	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/BATIS_LYON01.shp", FALSE); //Doit ouvrir un fichier CityGML contenant des batiments LOD2
	//OGRDataSource* BatiShapeFile = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/BatiTest.shp", FALSE);

	QTime time;
	time.start();

	std::vector<TextureCityGML*> ListTextures;
	citygml::CityModel* ModelOut = CutCityGMLwithShapefile(BatiLOD2CityGML, BatiShapeFile, &ListTextures);

	delete BatiShapeFile;

	ModelOut->computeEnvelope();

	citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString()  + "_CutBuildings.gml");
	//citygml::ExporterCityGML exporter("A_CutBuildings.gml");

	exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

	for(TextureCityGML* Tex:ListTextures)
		delete Tex;

	delete BatiLOD2CityGML;
	//delete ModelOut; // !!!!!!!!!!!! On ne peut pas delete BatiLOD2CityGML et ModelOut car on a recupere des batiments tels quels du premier pour les mettre dans le second (ceux qui n'ont pas d'equivalents dans le Shapefile). Du coup ce n'est pas propre (fuite memoire) mais il n'y a a pas de clone() sur les Cityobject...
	//////////
	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;

	std::cout << Folder + "/" + file1.baseName().toStdString()  + "_CutBuildings.gml a ete cree." << std::endl;

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
void MainWindow::TilingCityGML(QString CityGMLPath, std::string OutputPath, int TileX, int TileY) //BIEN PENSER A METTRE LES DOSSIER DE TEXTURE AVEC LES CITYGML POUR LES MNT AVEC DES TEXTURE WORLD
{
	//CityGMLPath = "D:/Donnees/Data/CityGML Grand Lyon/2012_DonneesVisibilite/Test";
	//OutputPath = "D:/Donnees/Data/CityGML Grand Lyon/2012_DonneesVisibilite/Test2";

	CPLPushErrorHandler( CPLQuietErrorHandler ); //POUR CACHER LES WARNING DE GDAL

	QTime time;
	time.start();

	QDir dir(CityGMLPath);
	QStringList list;

	QDirIterator iterator(dir.absolutePath(), QDirIterator::Subdirectories);
	while(iterator.hasNext())
	{
		iterator.next();
		if(!iterator.fileInfo().isDir())
		{
			QString filename = iterator.filePath();
			if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
				list.append(filename);
		}
	}
	std::cout << list.size() << " fichier(s) CityGML trouve(s)." << std::endl;

	int cpt = 0;
	for(QString Path:list)
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

		for(int x = (int)MinTile.x; x <= (int)MaxTile.x; x += TileX)
		{
			for(int y = (int)MinTile.y; y <= (int)MaxTile.y; y += TileY)
			{
				std::vector<TextureCityGML*> TexturesList;

				std::cout << "Tuile : " << x/TileX << "_" << y/TileY << std::endl;
				citygml::CityModel* Tuile = TileCityGML(Tile, &TexturesList, TVec2d(x, y), TVec2d(x + TileX, y + TileY), CityGMLPath.toStdString().substr(0, CityGMLPath.toStdString().find_last_of("/")));

				std::string FileName = OutputPath + "/" + std::to_string((int)(x / TileX)) + "_" + std::to_string((int)(y / TileY))  + ".gml";

				FILE * fp = fopen(FileName.c_str(), "rb");
				if(fp == nullptr) //Le fichier correspondant a la tuile courante n'existe pas, on peut donc le creer
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
				for(TextureCityGML* Tex : TexturesList)
					delete Tex;
			}
		}
		delete Tile;

		++cpt;
		std::cout << cpt << " fichier(s) traite(s)." << std::endl;
	}

	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;
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
	if(m_temporalAnim)
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
	m_ui->horizontalSlider->setValue(m_ui->horizontalSlider->value()+incr);
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
void MainWindow::slotCutMNTwithShapefile()
{
	QSettings settings("liris", "virtualcity");
	QString lastdir = settings.value("lastdir").toString();
	QString filename1 = QFileDialog::getOpenFileName(this, "Selectionner le fichier CityGML a traiter.", lastdir);
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
	QString filename2 = QFileDialog::getOpenFileName(this, "Selectionner le fichier Shapefile contenant les polygones de decoupe.", lastdir);
	QFileInfo file2(filename2);
	QString filepath2 = file2.absoluteFilePath();
	QString ext2 = file2.suffix().toLower();
	if(ext2 != "shp")
	{
		std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
		QApplication::restoreOverrideCursor();
		return;
	}
	settings.setValue("lastdir", file2.dir().absolutePath());


	QFileDialog w;
	w.setWindowTitle("Selectionner le dossier de sortie");
	w.setFileMode(QFileDialog::Directory);

	if(w.exec() == 0)
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

	citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString()  + "_" + file2.baseName().toStdString() + ".gml");

	exporter.exportCityModelWithListTextures(*ModelOut, &ListTextures);

	int millisecondes = time.elapsed();
	std::cout << "Traitement termine, fichier MNT decoupe cree. Execution time : " << millisecondes/1000.0 <<std::endl;

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
	if(ext1 != "citygml" && ext1 != "gml")
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
	if(ext2 != "shp")
	{
		std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
		QApplication::restoreOverrideCursor();
		return;
	}
	settings.setValue("lastdir", file2.dir().absolutePath());


	QFileDialog w;
	w.setWindowTitle("Selectionner le dossier de sortie");
	w.setFileMode(QFileDialog::Directory);

	if(w.exec() == 0)
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

	citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString()  + "_MNT_Roads.gml");

	exporter.exportCityModelWithListTextures(*MNT_roads, &ListTextures_Roads);

	citygml::ExporterCityGML exporter2(Folder + "/" + file1.baseName().toStdString()  + "_MNT_Ground.gml");

	exporter2.exportCityModelWithListTextures(*MNT_grounds, &ListTextures_Ground);

	int millisecondes = time.elapsed();
	std::cout << "Traitement termine, fichier MNT road cree. Execution time : " << millisecondes/1000.0 <<std::endl;

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
	if(ext1 != "citygml" && ext1 != "gml")
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
	if(ext2 != "shp")
	{
		std::cout << "Erreur : Le fichier n'est pas un Shapefile." << std::endl;
		QApplication::restoreOverrideCursor();
		return;
	}
	settings.setValue("lastdir", file2.dir().absolutePath());


	QFileDialog w;
	w.setWindowTitle("Selectionner le dossier de sortie");
	w.setFileMode(QFileDialog::Directory);

	if(w.exec() == 0)
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

	citygml::ExporterCityGML exporter(Folder + "/" + file1.baseName().toStdString()  + "_MNT_Vegetation.gml");

	exporter.exportCityModelWithListTextures(*MNT_vegetation, &ListTextures_Vegetation);

	citygml::ExporterCityGML exporter2(Folder + "/" + file1.baseName().toStdString()  + "_MNT_Ground.gml");

	exporter2.exportCityModelWithListTextures(*MNT_grounds, &ListTextures_Ground);

	int millisecondes = time.elapsed();
	std::cout << "Traitement termine, fichier MNT vegetation cree. Execution time : " << millisecondes/1000.0 <<std::endl;

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
/*void MainWindow::test1() //Generation de stats pour etude de visibilite
{
	QTime time;
	time.start();

	for(int cam = 1; cam <= 10; ++cam)
	{
		QString filepath = QString::fromStdString("D:/Dropbox/ResultatsVisibilite/"+std::to_string(cam)+"_Comparaison.csv");

		QFileInfo file(filepath);

		if(file.exists())
		{
			continue;
		}

		std::ofstream ofs;
		ofs.open("D:/Dropbox/ResultatsVisibilite/"+std::to_string(cam)+"_Comparaison.csv", std::ofstream::out);
		std::string Folder = "D:/3DUSE 0.2.5/SkylineOutput_" + std::to_string(cam) + "/";

		int min = -1;
		int max = -1;

		for(int i = 0; i <= 20; ++i)
		{
			std::string dist = std::to_string(i);

			filepath = QString::fromStdString(Folder+dist) + "_Result.shp";

			QFileInfo file(filepath);

			if(file.exists())
			{
				if(min == -1)
					min = i;
				max = i;
			}
			else if(min != -1)
				break;
		}

		std::cout << cam << " : " << min << "   " << max << std::endl;

		filepath = QString::fromStdString(Folder+std::to_string(max)) + "_Result.shp";

		OGRDataSource* DataSource = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);
		OGRLayer* Layer = DataSource->GetLayer(0);

		OGRMultiPoint* ListPointsLod2 = new OGRMultiPoint;
		OGRFeature *Feature;
		Layer->ResetReading();
		while((Feature = Layer->GetNextFeature()) != NULL)
		{
			OGRGeometry* Point = Feature->GetGeometryRef();
			if(Point->getGeometryType() == wkbPoint || Point->getGeometryType() == wkbPoint25D)
				ListPointsLod2->addGeometry(Point);
		}

		delete DataSource;

		for(int i = min; i < max; ++i)
		{
			std::cout << "Avancement LoD1 : " << i << "/" << max << std::endl;
			int cpt1 = 0; //Points de Lod1 qui sont dans Lod2
			int cpt2 = 0; //Points de Lod1 qui ne sont pas dans Lod2
			int cpt3 = 0; //Points de Lod2 qui sont dans Lod1
			int cpt4 = 0; //Points de Lod2 qui ne sont pas dans Lod1

			std::string dist = std::to_string(i);

			filepath = QString::fromStdString(Folder+dist) + "_Result.shp";

			OGRDataSource* DataSource2 = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);
			OGRLayer* Layer2 = DataSource2->GetLayer(0);

			OGRMultiPoint* ListPoints = new OGRMultiPoint;
			OGRFeature *Feature2;
			Layer2->ResetReading();
			while((Feature2 = Layer2->GetNextFeature()) != NULL)
			{
				OGRGeometry* Point = Feature2->GetGeometryRef();
				if(Point->getGeometryType() == wkbPoint || Point->getGeometryType() == wkbPoint25D)
				{
					ListPoints->addGeometry(Point);
					//if(Point->Distance(ListPointsLod2) < 1.0)
					//cpt1++;
					//else
					//cpt2++;
				}
			}
			delete DataSource2;

			for(int j = 0; j < ListPointsLod2->getNumGeometries(); ++j)
			{
				double distance = 1000;
				std::cout << "Avancement Comparaison Points : " << j << " / " << ListPointsLod2->getNumGeometries() << ".\r" << std::flush;
				OGRPoint* Point1 = (OGRPoint*)ListPointsLod2->getGeometryRef(j);
				for(int k = 0; k < ListPoints->getNumGeometries(); ++k)
				{
					OGRPoint* Point2 = (OGRPoint*)ListPoints->getGeometryRef(k);
					float tempdist = sqrt((Point2->getX()-Point1->getX())*(Point2->getX()-Point1->getX()) + (Point2->getY()-Point1->getY())*(Point2->getY()-Point1->getY()) + (Point2->getZ()-Point1->getZ())*(Point2->getZ()-Point1->getZ()));
					if(tempdist < distance)
						distance = tempdist;
				}
				if(distance < 1.0)
					cpt3++;
				else
					cpt4++;
			}
			std::cout << std::endl;

			ofs << "Comparaison LoD1 a partir de " << i << "000m et LoD2 (" << max << "000m)" << std::endl;
			ofs << "LoD1 : " << ListPoints->getNumGeometries() << " points;LoD2 : " << ListPointsLod2->getNumGeometries() << " points" << std::endl;
			ofs << "; Nombre de points" << std::endl;
			//ofs << "Points de Lod1 qui sont dans Lod2 : " << cpt1 << ";" << cpt1/(cpt1 + cpt2) * 100.f << std::endl;
			//ofs << "Points de Lod1 qui ne sont pas dans Lod2 : " << cpt2 << ";" << cpt2/(cpt1 + cpt2) * 100.f << std::endl;
			ofs << "Points de Lod2 qui sont dans Lod1 : ;" << cpt3 << std::endl;
			ofs << "Points de Lod2 qui ne sont pas dans Lod1 : ;" << cpt4 << std::endl << std::endl;

			delete ListPoints;
		}
		delete ListPointsLod2;
		ofs.close();
	}

	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;
}*/
void MainWindow::test1()
{
	QTime time;
	time.start();

	std::string LiDAR2012 = "C://Users//FredLiris//Downloads//Grand Lyon LiDAR//Grand Lyon CHANGEMENTS CRAPONNE//LAS 2012//1833_5173_1_1_1.laz";
	std::string LiDAR2015 = "C://Users//FredLiris//Downloads//Grand Lyon LiDAR//Grand Lyon CHANGEMENTS CRAPONNE//LAS 2015//1833_5173_2015_1_1_1.las";

	vcity::app().getAlgo().CompareTwoLidar(LiDAR2012, LiDAR2015);

	//vcity::app().getAlgo().ConvertLasToPCD();
	//vcity::app().getAlgo().ExtractGround();
	//vcity::app().getAlgo().ExtractBuildings();
	//vcity::app().getAlgo().RemoveGroundWithTIN();
	//vcity::app().getAlgo().CompareBuildings();
	//vcity::app().getAlgo().ConstructRoofs();

	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;

	return;

	////// Recupere les ilots partages en ilot Bati et ilot non bati (terrain) decoupes par les routes et les extrude en 3D grace aux informations de hauteurs
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
void MainWindow::test2() //Generation de stats pour etude de visibilite
{
	QTime time;
	time.start();

	for(int cam = 1; cam <= 10; ++cam)
	{
		std::ofstream ofs;
		ofs.open("D:/3DUSE 0.2.5/"+std::to_string(cam)+"_Skyline.csv", std::ofstream::out);
		std::string Folder = "D:/3DUSE 0.2.5/SkylineOutput_" + std::to_string(cam) + "/";

		int min = -1;
		int max = -1;

		QString filepath;

		for(int i = 0; i <= 20; ++i)
		{
			std::string dist = std::to_string(i);

			filepath = QString::fromStdString(Folder+dist) + "_SkylineLine.shp";

			QFileInfo file(filepath);

			if(file.exists())
			{
				if(min == -1)
					min = i;
				max = i;
			}
			else if(min != -1)
				break;
		}

		OGRDataSource* DS = OGRSFDriverRegistrar::Open((Folder + std::to_string(max)+ "_Viewpoint.shp").c_str(), FALSE);
		OGRLayer* L = DS->GetLayer(0);
		OGRFeature *F;
		L->ResetReading();
		F = L->GetNextFeature();
		OGRPoint* Camera = (OGRPoint*)F->GetGeometryRef()->clone();

		delete DS;

		std::cout << cam << " : " << min << "   " << max << std::endl;

		filepath = QString::fromStdString(Folder+std::to_string(max)) + "_SkylineLine.shp";

		OGRDataSource* DataSource = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);
		OGRLayer* Layer = DataSource->GetLayer(0);

		OGRLineString * LineLoD2;
		OGRFeature *Feature;
		Layer->ResetReading();
		Feature = Layer->GetNextFeature();
		LineLoD2 = (OGRLineString*)Feature->GetGeometryRef()->clone();

		delete DataSource;

		for(int i = min; i < max; ++i)
		{
			std::cout << "Avancement : " << i << std::endl;

			std::string dist = std::to_string(i);

			filepath = QString::fromStdString(Folder+dist) + "_SkylineLine.shp";

			OGRDataSource* DataSource2 = OGRSFDriverRegistrar::Open(filepath.toStdString().c_str(), FALSE);
			OGRLayer* Layer2 = DataSource2->GetLayer(0);

			OGRLineString * Line;
			OGRFeature *Feature2;
			Layer2->ResetReading();
			Feature2 = Layer2->GetNextFeature();
			Line = (OGRLineString*)Feature2->GetGeometryRef()->clone();

			delete DataSource2;

			//std::cout << Line->getNumPoints() << " - " << LineLoD2->getNumPoints() << std::endl;

			std::vector<float> Dist;

			ofs << "LoD2 a une distance de " << max << std::endl;
			ofs << "Comparaison LoD1 a partir de " << i << "000m et LoD2" << std::endl;

			//OGRMultiLineString* MLS = new OGRMultiLineString;

			for(int k = 0; k < LineLoD2->getNumPoints(); ++k)
			{
				OGRPoint* Point1 = new OGRPoint;
				LineLoD2->getPoint(k, Point1);

				double DistMin = 100000;

				for(int j = 0; j < Line->getNumPoints(); ++j)
				{
					OGRPoint* Point2 = new OGRPoint;
					Line->getPoint(j, Point2);

					TVec2d AB(Point1->getX() - Camera->getX(), Point1->getY() - Camera->getY());
					TVec2d AC(Point2->getX() - Camera->getX(), Point2->getY() - Camera->getY());

					/*std::cout << Point1->getX() << " " << Point1->getY() << " " << Point1->getZ() << std::endl;
					std::cout << Point2->getX() << " " << Point2->getY() << " " << Point2->getZ() << std::endl;

					std::cout << Camera->getX() << " " << Camera->getY() << " " << Camera->getZ() << std::endl;

					std::cout << AB << std::endl;
					std::cout << AC << std::endl;

					std::cout << AB.x * AC.y - AB.y * AC.x << std::endl;

					int a;
					std::cin >> a;*/


					if(AB.x * AC.y - AB.y * AC.x < 0.01)
					{
						double Distance = sqrt((Point2->getX()-Point1->getX())*(Point2->getX()-Point1->getX()) + (Point2->getY()-Point1->getY())*(Point2->getY()-Point1->getY()) + (Point2->getZ()-Point1->getZ())*(Point2->getZ()-Point1->getZ()));
						if(DistMin > Distance)
							DistMin = Distance;
					}

					//delete Point1;
					//delete Point2;
					//OGRLineString* LS = new OGRLineString;
					//LS->addPoint(Point1);
					//LS->addPoint(Point2);
					//MLS->addGeometryDirectly(LS);
					delete Point2;

				}
				if(DistMin == 100000)
				{
					std::cout << k << " Erreur DistMin" << std::endl;
					DistMin = Point1->Distance(Line);
					//int a;
					//std::cin >> a;
				}
				ofs << DistMin << ";";
				Dist.push_back(DistMin);

				delete Point1;
			}

			ofs << std::endl << std::endl;

			delete Line;

			//SaveGeometrytoShape("MLS.shp", MLS);
		}
		delete LineLoD2;
		ofs.close();
	}

	int millisecondes = time.elapsed();
	std::cout << "Execution time : " << millisecondes/1000.0 <<std::endl;
}
/*void MainWindow::test2()
{
//Creation d'ilots a partir de Shapefile contenant des routes
OGRDataSource* Routes = OGRSFDriverRegistrar::Open("C:/Users/Game Trap/Downloads/Data/Lyon01/Routes_Lyon01.shp", TRUE);
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
}*/
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test3()
{
	//FusionTiles(); //Fusion des fichiers CityGML contenus dans deux dossiers : sert a fusionner les tiles donc deux fichiers du meme nom seront fusionnes en un fichier contenant tous leurs objets a la suite.

	//// FusionLODs : prend deux fichiers modelisant les batiments avec deux lods differents et les fusionne en un seul
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
	/*BelvedereDB::Get().Setup("C:/VCityData/Tile/","Test");
	std::vector<std::pair<std::string,PolygonData>> top = BelvedereDB::Get().GetTop(5);

	std::ofstream ofs("C:/VCityBuild/SkylineOutput/TopPoly.csv",std::ofstream::out);

	ofs << "PolygonId" << ";" << "Time Seen" << ";" << "CityObjectId" << std::endl;
	for(std::pair<std::string,PolygonData> p : top)
	{
	ofs << p.first << ";" << p.second.HitCount << ";" << p.second.CityObjectId << std::endl;
	}
	ofs.close();*/

	/*ProcessCL("C:/VCityBuild/SkylineOutput/1841_5175.dat","1841_5175");
	ProcessCL("C:/VCityBuild/SkylineOutput/1841_5176.dat","1841_5176");
	ProcessCL("C:/VCityBuild/SkylineOutput/1842_5175.dat","1842_5175");
	ProcessCL("C:/VCityBuild/SkylineOutput/1842_5176.dat","1842_5176");*/

	//ExtrudeAlignementTree();

	/*LASreadOpener lasreadopener;
	lasreadopener.set_file_name("C:\VCityData\Veget\1841_5175.las");
	LASreader* lasreader = lasreadopener.open();

	OGRMultiPoint* mp = new OGRMultiPoint;

	while (lasreader->read_point())
	{
	OGRPoint* point = new OGRPoint;
	mp->addGeometry(new OGRPoint((lasreader->point).get_x(),(lasreader->point).get_y(),(lasreader->point).get_z()));
	}*/

	std::cout<<std::endl;
	vcity::LayerCityGML* layer = dynamic_cast<vcity::LayerCityGML*>(m_app.getScene().getDefaultLayer("LayerCityGML"));
	citygml::CityModel* model = layer->getTiles()[0]->getCityModel();
	std::vector<temporal::Version*> versions = model->getVersions();
	for (temporal::Version* version : versions)
	{
		std::cout<<"Version \""<<version->getId()<<"\" :"<<std::endl;
		std::vector<citygml::CityObject*>* members = version->getVersionMembers();
		for (std::vector<citygml::CityObject*>::iterator it = members->begin(); it != members->end(); it++)
		{
			std::cout<<"    - member: "<<(*it)->getId()<<std::endl;
		}
	}
	std::cout<<std::endl;
	std::vector<temporal::VersionTransition*> transitions = model->getTransitions();
	for (temporal::VersionTransition* transition : transitions)
	{
		std::cout<<"Transition \""<<transition->getId()<<"\" :"<<std::endl;
		std::cout<<"    - from: "<<transition->from()->getId()<<std::endl;
		std::cout<<"    - to: "<<transition->to()->getId()<<std::endl;
	}
	std::cout<<std::endl;

	std::cout<<"Workspaces:"<<std::endl;
	std::map<std::string,temporal::Workspace> workspaces = model->getWorkspaces();
	for(std::map<std::string,temporal::Workspace>::iterator it = workspaces.begin();it!=workspaces.end();it++){
		std::cout<<it->second.name<<std::endl;
		for(temporal::Version* v : it->second.versions){
			std::cout<<"    - "<<v->getId()<<std::endl;
		}
	}

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

////////////////////////////////////////////////////////////////////////////////
void MainWindow::loadShpFile(const QString& filepath)
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

	if(poDS)
	{
		vcity::URI uriLayer = m_app.getScene().getDefaultLayer("LayerShp")->getURI();
		appGui().getControllerGui().addShpNode(uriLayer, poDS);

		addRecentFile(filepath);

		//m_osgScene->m_layers->addChild(buildOsgGDAL(poDS));
	}

	//OGRSFDriverRegistrar::GetRegistrar()->ReleaseDataSource(poDS);
}
