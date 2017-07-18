// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include <QSettings>
#include <QFileDialog>
#include <osg/MatrixTransform>
#include <queue>

#include "dialogVisibilite.hpp"
#include "ui_dialogVisibilite.h"
#include "gui/applicationGui.hpp"

#include "libcitygml/export/exportCityGML.hpp"
// FIXME: why the following dependance of a plugin towards the main application
// window ?
#include "gui/moc/mainWindow.hpp"  
#include "filters/ShpExtrusion/ShpExtrusion.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"
#include "AABB.hpp"
#include "../Visibilite.hpp"                      // FIXME: no relative path
#include "../data/BelvedereDB.h"                  // FIXME: no relative path
#include "../FlatRoof.hpp"                        // FIXME: no relative path
#include "../VegetTool.hpp"                       // FIXME: no relative path
#include "../AlignementTree.hpp"                  // FIXME: no relative path

////////////////////////////////////////////////////////////////////////////////
DialogVisibilite::DialogVisibilite(QWidget *parent, MainWindow* mainwindow) :
    QDialog(parent),
    ui(new Ui::DialogVisibilite)
{
    ui->setupUi(this);

    ui->dirLE->setEnabled(false);
    ui->resYSB->setVisible(false);

    connect(ui->dirB, SIGNAL(clicked()), this, SLOT(DirButtonClicked()));
    connect(ui->getParamB, SIGNAL(clicked()), this, SLOT(GetCamParam()));
    connect(ui->applyParamB, SIGNAL(clicked()), this, SLOT(SetCamParam()));
    connect(ui->basicAnalysisB, SIGNAL(clicked()), this, SLOT(BasicMonoTile()));
    connect(ui->multitileBasicB, SIGNAL(clicked()), this, SLOT(BasicMultiTile()));
    connect(ui->panoramaB, SIGNAL(clicked()), this, SLOT(BasicPanorama()));
    connect(ui->panoramaCascadeB, SIGNAL(clicked()), this, SLOT(CascadePanorama()));
    connect(ui->cascadeB, SIGNAL(clicked()), this, SLOT(CascadeMonoTile()));
    connect(ui->multitileCascadeB, SIGNAL(clicked()), this, SLOT(CascadeMultiTile()));
    connect(ui->resetCatB, SIGNAL(clicked()), this, SLOT(ResetCategory()));
    connect(ui->addToBatchB, SIGNAL(clicked()), this, SLOT(CopyPointToBatch()));
    connect(ui->alignementTreeB, SIGNAL(clicked()), this, SLOT(ToolAlignementTree()));
    connect(ui->lidarToGMLB, SIGNAL(clicked()), this, SLOT(ToolLidarToGML()));
    connect(ui->flatRoofB, SIGNAL(clicked()), this, SLOT(ToolFlatRoof()));
    connect(ui->extrudeShpB, SIGNAL(clicked()), this, SLOT(ToolShpExtrusion()));
    connect(ui->rebuildBoxB, SIGNAL(clicked()), this, SLOT(ToolAABBReconstruction()));
    connect(ui->exportTopB, SIGNAL(clicked()), this, SLOT(GetTopPolygon()));
    connect(ui->batchMTB, SIGNAL(clicked()), this, SLOT(BatchMultiTile()));
    connect(ui->SplitSkylinePointsB, SIGNAL(clicked()), this, SLOT(SplitSkylinePoints()));

    QSettings settings("liris", "virtualcity");
    QString tiledir = settings.value("tiledir").toString();
    ui->dirLE->setText(tiledir);

    QString category = settings.value("capturecategory").toString();
    ui->categoryLE->setText(category);

    double deltaDistance = settings.value("capturedeltadistance").toDouble();
    ui->deltaDistanceSB->setValue(deltaDistance);

    QDir outputDir(tiledir + "/SkylineOutput/");
    if (!outputDir.exists(tiledir + "/SkylineOutput/"))
        outputDir.mkpath(outputDir.absolutePath());

    QDir extrudDir(tiledir + "/ShpExtruded/");
    if (!extrudDir.exists(tiledir + "/ShpExtruded/"))
        extrudDir.mkpath(extrudDir.absolutePath());

    ui->projDistanceSB->hide();///////
    ui->projDistLabel->hide();///////

    this->mainwindow = mainwindow;
}
////////////////////////////////////////////////////////////////////////////////
void DialogVisibilite::SplitSkylinePoints()
{
    QString filename = QFileDialog::getOpenFileName(nullptr, "Load shp file");

    QFileInfo file(filename);

    QString ext = file.suffix().toLower();

    if (ext != "shp")
    {
        std::cout << "Error : File is not a shp" << std::endl;
        return;
    }

    OGRGeometryCollection* Buildings = new OGRGeometryCollection;
    OGRGeometryCollection* MNT = new OGRGeometryCollection;
    OGRGeometryCollection* Veget = new OGRGeometryCollection;

    OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filename.toStdString().c_str(), FALSE);

    OGRLayer *poLayer;
    int nbLayers = poDS->GetLayerCount();
    if (nbLayers > 0)
    {
        poLayer = poDS->GetLayer(0);

        OGRFeature *poFeature;
        poLayer->ResetReading();

        while ((poFeature = poLayer->GetNextFeature()) != NULL)
        {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();

            if (poGeometry == NULL || poFeature->GetFieldIndex("ObjectType") == -1)
                continue;

            std::string type = poFeature->GetFieldAsString("ObjectType");

            if (type == "SolidaryVegetationObject")
                Veget->addGeometry(poGeometry);
            else if (type == "Building")
                Buildings->addGeometry(poGeometry);
            else if (type == "TINRelief")
                MNT->addGeometry(poGeometry);
        }
    }

    SaveGeometrytoShape(file.absolutePath().toStdString() + "/" + file.baseName().toStdString() + "Veget.shp", Veget);
    SaveGeometrytoShape(file.absolutePath().toStdString() + "/" + file.baseName().toStdString() + "Buildings.shp", Buildings);
    SaveGeometrytoShape(file.absolutePath().toStdString() + "/" + file.baseName().toStdString() + "MNT.shp", MNT);

    delete Veget;
    delete Buildings;
    delete MNT;
    delete poDS;
}

void DialogVisibilite::DirButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr, "Get Tile Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("tiledir", dirpath);
    ui->dirLE->setText(dirpath);
}

void DialogVisibilite::GetCamParam()
{
    TVec3d offset = mainwindow->m_app.getSettings().getDataProfile().m_offset;
    osg::Camera* cam = mainwindow->m_osgView->m_osgView->getCamera();
    osg::Vec3d pos;
    osg::Vec3d target;
    osg::Vec3d up;
    cam->getViewMatrixAsLookAt(pos, target, up);

    target = target - pos;
    target.normalize();

    ui->posXSB->setValue(pos.x() + offset.x);
    ui->posYSB->setValue(pos.y() + offset.y);
    ui->posZSB->setValue(pos.z() + offset.z);
    ui->dirXSB->setValue(target.x());
    ui->dirYSB->setValue(target.y());
    ui->dirZSB->setValue(target.z());
}

void DialogVisibilite::SetCamParam()
{
    TVec3d offset = mainwindow->m_app.getSettings().getDataProfile().m_offset;
    osg::Camera* cam = mainwindow->m_osgView->m_osgView->getCamera();
    osg::Vec3d pos;
    osg::Vec3d target;
    osg::Vec3d up;

    cam->getViewMatrixAsLookAt(pos, target, up);

    pos = osg::Vec3(ui->posXSB->value() - offset.x, ui->posYSB->value() - offset.y, ui->posZSB->value() - offset.z);

    osg::Vec3d dir = osg::Vec3(ui->dirXSB->value(), ui->dirYSB->value(), ui->dirZSB->value());

    dir.normalize();

    osg::Vec3d right = dir ^ osg::Vec3d(0.0, 0.0, 1.0);
    up = right ^ dir;

    target = pos + dir;

    cam->setViewMatrixAsLookAt(pos, target, up);

    osg::Matrixd mat = cam->getViewMatrix();

    mainwindow->m_osgView->m_osgView->getCameraManipulator()->setByInverseMatrix(mat);
    cam->getViewMatrixAsLookAt(pos, target, up);

    osg::Vec3 d = dir;
    osg::Vec3 u(-d.x() * d.z(), -d.y() * d.z(), d.x() * d.x() + d.y() * d.y());
    u.normalize();

    double C = 0.5;   // Cos of 60 degrees
    double S = 0.866; // Sin of 60 degrees

    std::cout << "Vecteur : " << d.x() << " " << d.y() << " " << d.z() << std::endl;

    ////////////////

    for (int i = 0; i < 5; ++i)
    {
        double x = (u.x()*u.x()*(1 - C) + C)*d.x() + (u.x()*u.y()*(1 - C) - u.z()*S)*d.y() + (u.x()*u.z()*(1 - C) + u.y()*S)*d.z();
        double y = (u.x()*u.y()*(1 - C) + u.z()*S)*d.x() + (u.y()*u.y()*(1 - C) + C)*d.y() + (u.y()*u.z()*(1 - C) - u.x()*S)*d.z();
        double z = (u.x()*u.z()*(1 - C) - u.y()*S)*d.x() + (u.y()*u.z()*(1 - C) + u.x()*S)*d.y() + (u.z()*u.z()*(1 - C) + C)*d.z();

        d = osg::Vec3(x, y, z);

        d.normalize();

        std::cout << "Rotation " << i + 1 << " : "
                  << d.x() << " " << d.y() << " " << d.z() << std::endl;

    }
}

void DialogVisibilite::SetupEmblematicViewExportParameter()
{
    // In order to add a new data set, uncomment exemple and replace fillers <..> by your data
    EmblematicView view;
    view.sky = ui->skySB->value();
    view.building = ui->buildingSB->value();
    view.remarquableBuilding = ui->remarquableSB->value();
    view.vegetation = ui->vegetSB->value();
    view.water = ui->waterSB->value();
    view.terrain = ui->terrainSB->value();
    //view.<myData> = ui-><myDataSB>->value();
    ExportParameter param = ExportParameter::GetGlobalParameter();
    param.emblematicView = view;
    ExportParameter::SetGobalParameter(param);
}

// Create the camera according to user defined data
osg::ref_ptr<osg::Camera> DialogVisibilite::SetupRenderingCamera()
{
    SetupEmblematicViewExportParameter();
    SetCamParam();

    osg::ref_ptr<osg::Camera> cam = new osg::Camera(*mainwindow->m_osgView->m_osgView->getCamera(), osg::CopyOp::DEEP_COPY_ALL);
    osg::Vec3d pos;
    osg::Vec3d target;
    osg::Vec3d up;

    cam->getViewMatrixAsLookAt(pos, target, up);

    TVec3d offset = mainwindow->m_app.getSettings().getDataProfile().m_offset;

    pos = pos + osg::Vec3d(offset.x, offset.y, offset.z);
    target = target + osg::Vec3d(offset.x, offset.y, offset.z);

    cam->setViewMatrixAsLookAt(pos, target, up);

    float fovx = ui->fovxSB->value();
    float fovy = ui->fovySB->value();

    float width = ui->resXSB->value();

    float height = floor(width * fovy / fovx);

    float znear = ui->projDistanceSB->value();

    cam->setProjectionMatrixAsPerspective(fovx, width / height, znear, 2000000.0);

    cam->setViewport(new osg::Viewport(cam->getViewport()->x(), cam->getViewport()->y(), width, height));

    return cam;
}

void DialogVisibilite::BasicPanorama()
{
    osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

    std::string dir = ui->dirLE->text().toStdString();

    QSettings settings("liris", "virtualcity");
    QString category = ui->categoryLE->text();
    settings.setValue("capturecategory", category);
    double deltaDistance = ui->deltaDistanceSB->value();
    settings.setValue("capturedeltadistance", deltaDistance);

    if (dir != "")
    {
        dir += "/";
        BelvedereDB::Get().Setup(dir, category.toStdString(), deltaDistance);
        MultiTilePanoramaAnalyse(dir, cam);
        BelvedereDB::Get().Setup("", "");
    }
}

void DialogVisibilite::BasicMultiTile()
{
    osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

    std::string dir = ui->dirLE->text().toStdString();

    QSettings settings("liris", "virtualcity");
    QString category = ui->categoryLE->text();
    settings.setValue("capturecategory", category);
    double deltaDistance = ui->deltaDistanceSB->value();
    settings.setValue("capturedeltadistance", deltaDistance);

    if (dir != "")
    {
        dir += "/";
        BelvedereDB::Get().Setup(dir, category.toStdString(), deltaDistance);
        MultiTileBasicAnalyse(dir, cam);
        BelvedereDB::Get().Setup("", ""); //Pour reset le setup du BelvedereDB
    }
}

void DialogVisibilite::BasicMonoTile()
{
    osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

    QSettings settings("liris", "virtualcity");
    QString category = ui->categoryLE->text();
    settings.setValue("capturecategory", category);
    double deltaDistance = ui->deltaDistanceSB->value();
    settings.setValue("capturedeltadistance", deltaDistance);

    QString filepath = QFileDialog::getOpenFileName(nullptr, "Load gml file");

    QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

    if (ext == "gml")
    {
        std::vector<std::string> building;
        building.push_back(filepath.toStdString());
        BasisAnalyse(ui->dirLE->text().toStdString(), building, cam);
    }
}


void DialogVisibilite::CascadePanorama()
{
    osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

    QSettings settings("liris", "virtualcity");
    QString category = ui->categoryLE->text();
    settings.setValue("capturecategory", category);
    double deltaDistance = ui->deltaDistanceSB->value();
    settings.setValue("capturedeltadistance", deltaDistance);

    std::string dir = ui->dirLE->text().toStdString();

    int count = ui->cascadeCountSB->value();
    float increment = ui->cascadeIncrementSB->value();

    if (dir != "")
    {
        dir += "/";
        MultiTileCascadePanoramaAnalyse(dir, cam, count, increment);
    }
}

void DialogVisibilite::CascadeMultiTile()
{
    osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

    QSettings settings("liris", "virtualcity");
    QString category = ui->categoryLE->text();
    settings.setValue("capturecategory", category);
    double deltaDistance = ui->deltaDistanceSB->value();
    settings.setValue("capturedeltadistance", deltaDistance);

    std::string dir = ui->dirLE->text().toStdString();

    int count = ui->cascadeCountSB->value();
    float increment = ui->cascadeIncrementSB->value();

    if (dir != "")
    {
        dir += "/";
        MultiTileCascadeAnalyse(dir, cam, count, increment);
    }
}

void DialogVisibilite::CascadeMonoTile()
{
    osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

    QSettings settings("liris", "virtualcity");
    QString category = ui->categoryLE->text();
    settings.setValue("capturecategory", category);
    double deltaDistance = ui->deltaDistanceSB->value();
    settings.setValue("capturedeltadistance", deltaDistance);

    QString filepath = QFileDialog::getOpenFileName(nullptr, "Load gml file");

    QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

    int count = ui->cascadeCountSB->value();
    float increment = ui->cascadeIncrementSB->value();

    if (ext == "gml")
    {
        std::vector<std::string> building;
        building.push_back(filepath.toStdString());
        CascadeAnalyse(ui->dirLE->text().toStdString(), building, cam, count, increment);
    }
}

void DialogVisibilite::ResetCategory()
{
    std::string dir = ui->dirLE->text().toStdString();
    dir += "/";
    QString category = ui->categoryLE->text();
    BelvedereDB::Get().ResetDB(dir, category.toStdString());
}

void DialogVisibilite::ToolAlignementTree()
{
    std::string dir = ui->dirLE->text().toStdString();

    if (dir != "")
    {
        dir += "/";
        ExtrudeAlignementTree(dir);
    }
}

void DialogVisibilite::ToolLidarToGML()
{
    ProcessCL(ProcessLasShpVeget(ui->dirLE->text().toStdString()));
}

void DialogVisibilite::ToolShpExtrusion()
{
    std::string dir = ui->dirLE->text().toStdString();

    if (dir != "")
    {
        dir += "/";
        QString filepath = QFileDialog::getOpenFileName(nullptr, "Load shp file");
        QString savepath = QFileDialog::getSaveFileName(nullptr, "Save CityGML file", "", "CityGML file (*.gml)");

        QFileInfo file(filepath);
        QString ext = file.suffix().toLower();
        if (ext == "shp")
        {
            OGRDataSource* poDS = OGRSFDriverRegistrar::Open(file.absoluteFilePath().toStdString().c_str(), FALSE);
            std::cout << "Shp loaded" << std::endl;
            std::cout << "Processing..." << std::endl;

            citygml::CityModel* ModelOut = ShpExtrusion(poDS, dir);

            delete poDS;

            citygml::ExporterCityGML exporter(savepath.toStdString());
            exporter.exportCityModel(*ModelOut);
            std::cout << "Done exporting" << std::endl;
        }
    }
}

void DialogVisibilite::ToolFlatRoof()
{

    QString filepath = QFileDialog::getOpenFileName(nullptr, "Load gml file");

    QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

    if (ext == "gml")
    {
        DetectionToitsPlats(filepath.toStdString(), 2.0, 0.98);
    }
}

void DialogVisibilite::ToolAABBReconstruction()
{
    std::string dir = ui->dirLE->text().toStdString();

    if (dir != "")
    {
        dir += "/";
        BuildLayersAABBs(dir);
    }
}

void DialogVisibilite::CopyPointToBatch()
{
    std::string dir = ui->dirLE->text().toStdString();

    std::ofstream ofs(dir + "/BatchViewpoints.txt", std::ofstream::app);
    ofs << std::fixed << ui->posXSB->value() << std::endl;
    ofs << std::fixed << ui->posYSB->value() << std::endl;
    ofs << std::fixed << ui->posZSB->value() << std::endl;
    ofs << std::fixed << ui->dirXSB->value() << std::endl;
    ofs << std::fixed << ui->dirYSB->value() << std::endl;
    ofs << std::fixed << ui->dirZSB->value() << std::endl;
    ofs.close();

    std::cout << "Point added to batch." << std::endl;
}

void DialogVisibilite::BatchMultiTile()
{
    std::string dir = ui->dirLE->text().toStdString();
    if (QFile(QString::fromStdString(dir + "/BatchViewpoints.txt")).exists())
    {
        std::ifstream ifs(dir + "/BatchViewpoints.txt", std::ofstream::in);

        std::queue<double> coordTemp;

        char line[512];

        while (ifs.good())
        {
            ifs.getline(line, 512);
            coordTemp.push(atof(line));
        }

        ifs.close();

        std::string dir = ui->dirLE->text().toStdString();

        if (dir != "")
        {
            dir += "/";


            QSettings settings("liris", "virtualcity");
            QString category = ui->categoryLE->text();
            settings.setValue("capturecategory", category);
            double deltaDistance = ui->deltaDistanceSB->value();
            settings.setValue("capturedeltadistance", deltaDistance);

            BelvedereDB::Get().Setup(dir, category.toStdString(), deltaDistance);

            unsigned int cpt = 0;

            while (coordTemp.size() >= 6)
            {
                double posx = coordTemp.front(); coordTemp.pop();
                double posy = coordTemp.front(); coordTemp.pop();
                double posz = coordTemp.front(); coordTemp.pop();
                double dirx = coordTemp.front(); coordTemp.pop();
                double diry = coordTemp.front(); coordTemp.pop();
                double dirz = coordTemp.front(); coordTemp.pop();

                ui->posXSB->setValue(posx);
                ui->posYSB->setValue(posy);
                ui->posZSB->setValue(posz);
                ui->dirXSB->setValue(dirx);
                ui->dirYSB->setValue(diry);
                ui->dirZSB->setValue(dirz);

                osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

                MultiTileBasicAnalyse(dir, cam, std::to_string(cpt) + "_");
                cpt++;
            }

            BelvedereDB::Get().Setup("", "");
        }

    }
}

void DialogVisibilite::GetTopPolygon()
{
    std::string category = ui->categoryLE->text().toStdString();
    std::string dir = ui->dirLE->text().toStdString();

    if (dir != "" && category != "")
    {
        dir += "/";
        BelvedereDB::Get().Setup(dir, category);
        std::map<std::string, std::vector<PolygonData>> top = BelvedereDB::Get().GetTop(ui->exportTopSB->value());

        std::ofstream ofs(dir + "/SkylineOutput/TopPoly.csv", std::ofstream::out);

        ofs << "Tile" << ";" << "PolygonId" << ";" << "Time Seen" << ";" << "CityObjectId" << std::endl;

        for (auto it = top.begin(); it != top.end(); it++)
        {
            ofs << it->first << std::endl;
            for (PolygonData p : it->second)
            {
                ofs << ";" << p.PolygonId << ";" << p.HitCount << ";" << p.CityObjectId << std::endl;
            }
        }
        ofs.close();
    }
}

DialogVisibilite::~DialogVisibilite()
{
    delete ui;
}
