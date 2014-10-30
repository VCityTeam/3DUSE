// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/mainWindow.hpp"
#include "ui_mainWindow.h"
#include "moc/dialogLoadBBox.hpp"
#include "moc/dialogSettings.hpp"
#include "moc/dialogAbout.hpp"

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

	connect(m_ui->actionChange_Detection, SIGNAL(triggered()), this, SLOT(slotChangeDetection()));
    connect(m_ui->actionCityGML_cut, SIGNAL(triggered()), this, SLOT(slotCityGML_cut()));
    connect(m_ui->actionOBJ_to_CityGML, SIGNAL(triggered()), this, SLOT(slotObjToCityGML()));
    connect(m_ui->actionCut_CityGML_with_Shapefile, SIGNAL(triggered()), this, SLOT(slotCutCityGMLwithShapefile()));

    connect(m_ui->actionTest_1, SIGNAL(triggered()), this, SLOT(test1()));
    connect(m_ui->actionTest_2, SIGNAL(triggered()), this, SLOT(test2()));
    connect(m_ui->actionTest_3, SIGNAL(triggered()), this, SLOT(test3()));
    connect(m_ui->actionTest_4, SIGNAL(triggered()), this, SLOT(test4()));
    connect(m_ui->actionTest_5, SIGNAL(triggered()), this, SLOT(test5()));

    // filter search
    connect(m_ui->filterButton, SIGNAL(clicked()), m_treeView, SLOT(slotFilter()));

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(slotTemporalAnimUpdate()));

    m_ui->horizontalSlider->setEnabled(m_useTemporal);
    m_ui->dateTimeEdit->setEnabled(m_useTemporal);
    m_ui->toolButton->setEnabled(m_useTemporal);

    updateRecentFiles();

    m_treeView->init();

    //m_ui->statusBar->showMessage("none");
    
    setlocale(LC_ALL, "C"); // MT : important for Linux
}
////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow()
{
    delete ShapeGeo;

    delete m_treeView;
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
    if(QDate::currentDate() > QDate(2015, 01, 15))
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
	else if(ext == "assimp" || ext ==  "dae" || ext ==  "blend" || ext ==  "3ds" || ext ==  "ase" || ext ==  "obj" || ext ==  "xgl" || ext ==  "ply" || ext ==  "dxf" || ext ==  "lwo" || ext ==  "lws" ||
		ext == "lxo" || ext ==  "stl" || ext ==  "x" || ext ==  "ac" || ext ==  "ms3d" || ext ==  "scn" || ext ==  "xml" || ext ==  "irrmesh" || ext ==  "irr" ||
		ext == "mdl" || ext ==  "md2" || ext ==  "md3" || ext ==  "pk3" || ext ==  "md5" || ext ==  "smd" || ext ==  "m3" || ext ==  "3d" || ext ==  "q3d" || ext ==  "off" || ext ==  "ter")
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
	else if(ext ==  "asc")
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
    else if(ext == "shp")
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
        m_ui->actionLoad_bbox->setVisible(true);
        m_ui->actionShow_advanced_tools->setVisible(true);
        m_ui->actionHelp->setVisible(true);
        m_ui->actionCityGML_cut->setVisible(true);
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
        //m_ui->menuTest->menuAction()->setVisible(false); //A cacher
        m_ui->actionFix_building->setVisible(false);
        m_ui->actionShadows->setVisible(false);
        m_ui->actionExport_osg->setVisible(false);
        m_ui->actionExport_tiled_osga->setVisible(false);
        m_ui->actionLoad_bbox->setVisible(false);
        m_ui->actionShow_advanced_tools->setVisible(false);
        m_ui->actionHelp->setVisible(false);
        m_ui->tab_16->setVisible(false);
        m_ui->tabWidget->removeTab(1);
        m_ui->widgetTemporal->setVisible(false);
        m_ui->hsplitter_bottom->setVisible(false);
        m_ui->actionShow_temporal_tools->setVisible(false);
        m_ui->actionCityGML_cut->setVisible(false);
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
//Version GDAL
void MainWindow::generateLOD0()
{
    QFileDialog w;
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();
    vcity::app().getAlgo().Folder = Folder;

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
				std::cout<< "GenerateLOD0 on "<< obj->getId() << std::endl;
				OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
				double * heightmax = new double;
				double * heightmin = new double;
                vcity::app().getAlgo().generateLOD0(obj, &Enveloppe, heightmax, heightmin);

                vcity::app().getAlgo().SaveGeometrytoShape(obj->getId()+"_Footprint", Enveloppe);
				citygml::Geometry* geom = vcity::app().getAlgo().ConvertLOD0ToCityGML(obj->getId(), Enveloppe, heightmin);
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
		//vcity::app().getAlgo().generateLOD0(vcity::URI());
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
					std::cout<< "GenerateLOD0 on "<< obj->getId() << std::endl;
					OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
					double * heightmax = new double;
					double * heightmin = new double;

                    vcity::app().getAlgo().generateLOD0(obj, &Enveloppe, heightmax, heightmin);

                    vcity::app().getAlgo().SaveGeometrytoShape(obj->getId()+"_Footprint", Enveloppe);

					citygml::Geometry* geom = vcity::app().getAlgo().ConvertLOD0ToCityGML(obj->getId(), Enveloppe, heightmin);

					citygml::CityObject* obj2 = new citygml::GroundSurface("Footprint");
					obj2->addGeometry(geom);
					obj->insertNode(obj2);

					appGui().getControllerGui().update(uri);

					delete Enveloppe;
					delete heightmax;
					delete heightmin;

                    std::cout<< "LOD0 genere sur "<< obj->getId() << std::endl;
				}
			}
		}
    }

	QApplication::restoreOverrideCursor();
}

/*void MainWindow::generateLOD0()
{
    QFileDialog w;
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();
    vcity::app().getAlgo().Folder = Folder;

    QApplication::setOverrideCursor(Qt::WaitCursor);
    // get all selected nodes (with a uri)
    const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
    if(uris.size() > 0)
    {
        // do all nodes selected
        for(std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
        {
            vcity::app().getAlgo().generateLOD0(*it);

            appGui().getControllerGui().update(*it);
        }
    }
	else
	{
		//vcity::app().getAlgo().generateLOD0(vcity::URI());
		for(vcity::Tile * tile : dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles())
		{
			for(citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
			{
				vcity::URI uri;
				uri.append(appGui().getScene().getDefaultLayer("LayerCityGML")->getName(), "LayerCityGML");
				uri.append(tile->getName(), "Tile");
				uri.append(obj->getId(), "Building");
				uri.setType("Building");

				vcity::app().getAlgo().generateLOD0(uri);

				appGui().getControllerGui().update(uri);
			}
		}
	}

	QApplication::restoreOverrideCursor();
}*/
////////////////////////////////////////////////////////////////////////////////
//Version GDAL
void MainWindow::generateLOD1()
{
    QFileDialog w;
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);
	if(ShapeGeo != nullptr)
	{
		vcity::app().getAlgo().generateLOD1(ShapeGeo, Hauteurs);
		delete ShapeGeo;
		ShapeGeo = nullptr;
	}
	else
	{
        citygml::ExporterCityGML exporter(Folder + "/" + appGui().getScene().getDefaultLayer("LayerCityGML")->getName() +".citygml");
		exporter.initExport();
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
                    vcity::app().getAlgo().generateLOD0(obj, &Enveloppe, heightmax, heightmin);

					citygml::CityObject* LOD1 = vcity::app().getAlgo().ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

					exporter.appendCityObject(*LOD1);
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
                        vcity::app().getAlgo().generateLOD0(obj, &Enveloppe, heightmax, heightmin);

						citygml::CityObject* LOD1 = vcity::app().getAlgo().ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

						exporter.appendCityObject(*LOD1);
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
        exporter.endExport();
        std::cout << "Fichier " << appGui().getScene().getDefaultLayer("LayerCityGML")->getName() +".citygml cree dans " + Folder << std::endl;
	}
    QApplication::restoreOverrideCursor();
}

/*void MainWindow::generateLOD1()
{
    QFileDialog w;
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();

    QApplication::setOverrideCursor(Qt::WaitCursor);
	if(ShapeGeo != nullptr)
	{
		vcity::app().getAlgo().generateLOD1(ShapeGeo, Hauteurs);
		delete ShapeGeo;
		ShapeGeo = nullptr;
	}
	else
	{
        citygml::ExporterCityGML exporter(Folder + "/" + appGui().getScene().getDefaultLayer("LayerCityGML")->getName() +".citygml");
		exporter.initExport();
		// get all selected nodes (with a uri)
		const std::vector<vcity::URI>& uris = vcity::app().getSelectedNodes();
		if(uris.size() > 0)//Si des bâtiments ont été selectionnés
		{
			// do all nodes selected
			for(std::vector<vcity::URI>::const_iterator it = uris.begin(); it < uris.end(); ++it)
			{
				std::cout << it->getStringURI() << std::endl;
				it->resetCursor();
				citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(*it);

				if(obj)
				{
					std::cout<< "GenerateLOD1 on "<< obj->getId() << std::endl;
					geos::geom::Geometry ** Enveloppe = new geos::geom::Geometry *;
					double * heightmax = new double;
					double * heightmin = new double;
					vcity::app().getAlgo().generateLOD0(obj, Enveloppe, heightmax, heightmin);

					citygml::CityObject* LOD1 = vcity::app().getAlgo().ConvertLOD1ToCityGML(obj->getId(), *Enveloppe, heightmax, heightmin);

					exporter.appendCityObject(*LOD1);
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
						geos::geom::Geometry ** Enveloppe = new geos::geom::Geometry *;
						double * heightmax = new double;
						double * heightmin = new double;
						vcity::app().getAlgo().generateLOD0(obj, Enveloppe, heightmax, heightmin);

						citygml::CityObject* LOD1 = vcity::app().getAlgo().ConvertLOD1ToCityGML(obj->getId(), *Enveloppe, heightmax, heightmin);

						exporter.appendCityObject(*LOD1);
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
                return;
            }
		}
		exporter.endExport();
        std::cout << "Fichier " << appGui().getScene().getDefaultLayer("LayerCityGML")->getName() +".citygml cree dans " + Folder << std::endl;
	}
    QApplication::restoreOverrideCursor();
}*/
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
    QFileDialog w;
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();
    vcity::app().getAlgo().Folder = Folder;

    QApplication::setOverrideCursor(Qt::WaitCursor);
    vcity::app().getAlgo().DecoupeCityGML(ShapeGeo, InfoBatiments);
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
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string Folder = w.selectedFiles().at(0).toStdString();
    vcity::app().getAlgo().Folder = Folder;

	QApplication::setOverrideCursor(Qt::WaitCursor);

    //VersionGeos
    //vcity::app().getAlgo().CompareTiles();

    //VersionGDAL
    const std::vector<vcity::Tile *> tiles = dynamic_cast<vcity::LayerCityGML*>(appGui().getScene().getDefaultLayer("LayerCityGML"))->getTiles();

    if(tiles.size() != 2)
    {
        std::cout << "Erreur : Il faut ouvrir deux fichiers CityGML de la même zone, en commençant par le plus ancien." << std::endl;
        QApplication::restoreOverrideCursor();
        return;
    }

    vcity::app().getAlgo().CompareTiles(tiles[0]->getCityModel(),tiles[1]->getCityModel());
    //

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
void buildJson()//Lyon03
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
}

/*void buildJson()//Villeurbanne
{
    //Error ! Mismatch type: 5TVec3IdE expected. Ring/Polygon discarded!

    //QString dataPath("/home/frederic/Telechargements/Data/VILLEURBANNE_BATIS_CITYGML/cut2"); //Découpe Bati (attention au #if dans exportJSON.cpp)
    QString dataPath("/home/frederic/Telechargements/Data/VILLEURBANNE_MNT_CITYGML/cut"); //Découpe Terrain (attention au #if dans exportJSON.cpp)

    std::string basePath("/home/frederic/Documents/JSON/"); //Dossier de sortie
    double offsetX = 1844500.0;
    double offsetY = 5173500.0;
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
////////////////////////////////////////////////////////////////////////////////
void buildJsonLod()// bldg geo ref test #if 1 dans exportJSON.cpp
{
    QString dataPath("/mnt/docs/upload/shp/paris/tiles");
    std::string basePath("/tmp/json/lod0/");
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
            if(filename.endsWith(".shp", Qt::CaseInsensitive))
            {
                //QFileInfo fileInfo(filename);
                std::string id = filename.toStdString();
                id = id.substr(id.find("tile_")+5);
                id = id.substr(0, id.find('.'));
                //std::cout << id; std::cout << " - " << id.substr(0,id.find('_')) << " / " << id.substr(id.find('_')+1) << std::endl;
                int idX = std::stoi(id.substr(0,id.find('_')));
                int idY = std::stoi(id.substr(id.find('_')+1));
                std::string f = "tile_" + std::to_string(idX) + '-' + std::to_string(idY);
                std::cout << filename.toStdString() << " -> " << basePath+f << "\n";

                std::cout << "id : " << idX << ", " << idY << std::endl;

                OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filename.toStdString().c_str(), FALSE);
				buildGeosShape(poDS, &ShapeGeo, &Hauteurs, &InfoBatiments);
                vcity::app().getAlgo().generateLOD1(ShapeGeo, Hauteurs);

                citygml::ExporterJSON exporter;
                exporter.setBasePath(basePath);
                exporter.setOffset(offsetX+stepX*(idX-idOffsetX), offsetY+stepY*(idY-idOffsetY));
                exporter.setTileSize(stepX, stepY);
                citygml::CityModel* model = vcity::app().getAlgo().getCitymodel();
                if(model)
                {
                    exporter.exportCityModel(*model, f, id);
                    delete model;
                }
                OGRSFDriverRegistrar::GetRegistrar()->ReleaseDataSource(poDS);
            }
        }
    }
    std::cout << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test1()
{
	//loadFile("C:/Users/Game Trap/Downloads/Data/Donnees_Sathonay/SATHONAY_CAMP_BATIS_2009.gml");
	//loadFile("C:/Users/Game Trap/Downloads/Data/Donnees_Sathonay/SATHONAY_CAMP_BATIS_2012.gml");
	//vcity::app().getAlgo().CompareTiles();

	 m_osgView->setActive(false); // reduce osg framerate to have better response in Qt ui (it would be better if ui was threaded)

    std::cout<<"Load Scene"<<std::endl;

    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList filenames = QFileDialog::getOpenFileNames(this, "Load scene files", lastdir);

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

			citygml::ExporterCityGML exporter(/*Folder + "/" + */file.baseName().toStdString() +"_LOD1.gml");
			exporter.initExport();

			for(citygml::CityObject * obj : tile->getCityModel()->getCityObjectsRoots())
			{		
				if(obj)
				{
					std::cout<< "GenerateLOD1 on "<< obj->getId() << std::endl;
					OGRMultiPolygon * Enveloppe = new OGRMultiPolygon;
					double * heightmax = new double;
					double * heightmin = new double;
					vcity::app().getAlgo().generateLOD0(obj, &Enveloppe, heightmax, heightmin);

					citygml::CityObject* LOD1 = vcity::app().getAlgo().ConvertLOD1ToCityGML(obj->getId(), Enveloppe, heightmax, heightmin);

					exporter.appendCityObject(*LOD1);

					delete Enveloppe;
					delete heightmax;
					delete heightmin;
				}
			}			
			exporter.endExport();
			std::cout << "Fichier " << file.baseName().toStdString() + "_LOD1.gml cree."<< std::endl;			
			QApplication::restoreOverrideCursor();
		}
    }

    m_osgView->setActive(true); // don't forget to restore high framerate at the end of the ui code (don't forget executions paths)
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test2()
{
    //loadFile("/home/maxime/docs/data/dd_gilles/IGN_Data/dpt_75/BDTOPO-75/BDTOPO/1_DONNEES_LIVRAISON_2011-12-00477/BDT_2-1_SHP_LAMB93_D075-ED113/E_BATI/BATI_INDIFFERENCIE.SHP");
    /*loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13725/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13724/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13725/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13724/export-CityGML/ZoneAExporter.gml");

    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13726/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13726/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1294-13726/export-CityGML/ZoneAExporter.gml");
    loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1294-13725/export-CityGML/ZoneAExporter.gml");*/
	loadFile("C:/Users/Game Trap/Downloads/Data/Lyon01/CADASTRE_SHP/BatiTest.shp");
	loadFile("C:/Users/Game Trap/Downloads/Data/Lyon01/Jeux de test/LYON_1ER_00136.gml");

	vcity::app().getAlgo().DecoupeCityGML(ShapeGeo, InfoBatiments);
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test3()
{
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1295-13728/export-CityGML/ZoneAExporter.gml");
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13728/export-CityGML/ZoneAExporter.gml");
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1296-13727/export-CityGML/ZoneAExporter.gml");
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1297-13727/export-CityGML/ZoneAExporter.gml");

	loadFile("C:/Users/Game Trap/Downloads/Data/Donnees_Sathonay/SATHONAY_CAMP_BATIS_CROP_2009.gml");
	loadFile("C:/Users/Game Trap/Downloads/Data/Donnees_Sathonay/SATHONAY_CAMP_BATIS_CROP_2012.gml");

	vcity::app().getAlgo().CompareTiles();

    // test obj
}
////////////////////////////////////////////////////////////////////////////////
void MainWindow::test4()
{
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1305-13719/export-CityGML/ZoneAExporter.gml");
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1304-13719/export-CityGML/ZoneAExporter.gml");
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1305-13720/export-CityGML/ZoneAExporter.gml");
    //loadFile("/home/maxime/docs/data/dd_gilles/3DPIE_Donnees_IGN_unzip/EXPORT_1304-13720/export-CityGML/ZoneAExporter.gml");

    // test json
    buildJson();
    //buildJsonLod();
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
