// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogVisibilite.hpp"
#include "ui_dialogVisibilite.h"
#include "gui/applicationGui.hpp"

#include "moc/mainWindow.hpp"
#include "src/visibilite/Visibilite.hpp"
#include "src/visibilite/data/BelvedereDB.h"
#include "src/visibilite/FlatRoof.hpp"
#include "src/visibilite/ShpExtrusion.h"
#include "src/visibilite/VegetTool.hpp"
#include "src/visibilite/AlignementTree.hpp"

#include <QSettings>
#include <QFileDialog>
#include <osg/MatrixTransform>

////////////////////////////////////////////////////////////////////////////////
DialogVisibilite::DialogVisibilite(QWidget *parent, MainWindow* mainwindow) :
    QDialog(parent),
    ui(new Ui::DialogVisibilite)
{
    ui->setupUi(this);

	ui->dirLE->setEnabled(false);
	ui->resYSB->setVisible(false);

	connect(ui->dirB,SIGNAL(clicked()),this,SLOT(DirButtonClicked()));
	connect(ui->getParamB,SIGNAL(clicked()),this,SLOT(GetCamParam()));
	connect(ui->applyParamB,SIGNAL(clicked()),this,SLOT(SetCamParam()));
	connect(ui->basicAnalysisB,SIGNAL(clicked()),this,SLOT(BasicMonoTile()));
	connect(ui->multitileBasicB,SIGNAL(clicked()),this,SLOT(BasicMultiTile()));
	connect(ui->panoramaB,SIGNAL(clicked()),this,SLOT(BasicPanorama()));
	connect(ui->panoramaCascadeB,SIGNAL(clicked()),this,SLOT(CascadePanorama()));
	connect(ui->cascadeB,SIGNAL(clicked()),this,SLOT(CascadeMonoTile()));
	connect(ui->multitileCascadeB,SIGNAL(clicked()),this,SLOT(CascadeMultiTile()));
	connect(ui->resetCatB,SIGNAL(clicked()),this,SLOT(ResetCategory()));

	QSettings settings("liris", "virtualcity");
    QString tiledir = settings.value("tiledir").toString();
	ui->dirLE->setText(tiledir);

	QString caterogy = settings.value("capturecategory").toString();
	ui->categoryLE->setText(caterogy);

	double deltaDistance = settings.value("capturedeltadistance").toDouble();
	ui->deltaDistanceSB->setValue(deltaDistance);

	this->mainwindow = mainwindow;
}
////////////////////////////////////////////////////////////////////////////////

void DialogVisibilite::DirButtonClicked()
{
	QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Get Tile Directory");
	QSettings settings("liris", "virtualcity");
    settings.setValue("tiledir",dirpath);
	ui->dirLE->setText(dirpath);
}

void DialogVisibilite::GetCamParam()
{
	TVec3d offset = mainwindow->m_app.getSettings().getDataProfile().m_offset;
	osg::Camera* cam = mainwindow->m_osgView->m_osgView->getCamera();
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	target = target - pos;
	target.normalize();

	ui->posXSB->setValue(pos.x()+offset.x);
	ui->posYSB->setValue(pos.y()+offset.y);
	ui->posZSB->setValue(pos.z()+offset.z);
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

	cam->getViewMatrixAsLookAt(pos,target,up);

	pos = osg::Vec3(ui->posXSB->value()-offset.x,ui->posYSB->value()-offset.y,ui->posZSB->value()-offset.z);
	target = pos + osg::Vec3(ui->dirXSB->value(),ui->dirYSB->value(),ui->dirZSB->value());
	

	cam->setViewMatrixAsLookAt(pos,target,up);
	
	osg::Matrixd mat = cam->getViewMatrix();

	mainwindow->m_osgView->m_osgView->getCameraManipulator()->setByInverseMatrix(mat);
	cam->getViewMatrixAsLookAt(pos,target,up);

}

void DialogVisibilite::SetupEmblematicViewExportParameter()
{
	EmblematicView view;
	view.sky = ui->skySB->value();
	view.building = ui->buildingSB->value();
	view.remarquableBuilding = ui->remarquableSB->value();
	view.vegetation = ui->vegetSB->value();
	view.water = ui->waterSB->value();
	view.terrain = ui->terrainSB->value();
	ExportParameter param = ExportParameter::GetGlobalParameter();
	param.emblematicView = view;
	ExportParameter::SetGobalParameter(param);
}

osg::ref_ptr<osg::Camera> DialogVisibilite::SetupRenderingCamera()
{
	SetupEmblematicViewExportParameter();
	SetCamParam();

	osg::ref_ptr<osg::Camera> cam = new osg::Camera(*mainwindow->m_osgView->m_osgView->getCamera(),osg::CopyOp::DEEP_COPY_ALL);
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;

	cam->getViewMatrixAsLookAt(pos,target,up);

	TVec3d offset = mainwindow->m_app.getSettings().getDataProfile().m_offset;

	pos = pos + osg::Vec3d(offset.x,offset.y,offset.z);
	target = target + osg::Vec3d(offset.x,offset.y,offset.z);
	
	cam->setViewMatrixAsLookAt(pos,target,up);


	float fovx = ui->fovxSB->value();
	float fovy = ui->fovySB->value();

	float width = ui->resXSB->value();

	float height = width * fovy/fovx;

	float znear = ui->projDistanceSB->value();

	cam->setProjectionMatrixAsPerspective(fovx,width/height,znear,2000.0);

	cam->setViewport(new osg::Viewport(cam->getViewport()->x(),cam->getViewport()->y(),width,height));

	return cam;
}

void DialogVisibilite::BasicPanorama()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	std::string dir = ui->dirLE->text().toStdString();

	QSettings settings("liris", "virtualcity");
	QString caterogy = ui->categoryLE->text(); 
	settings.setValue("capturecategory",caterogy);
	double deltaDistance = ui->deltaDistanceSB->value();
	settings.setValue("capturedeltadistance",deltaDistance);
	
	if(dir != "")
	{
		dir+="/";
		BelvedereDB::Get().Setup(dir,caterogy.toStdString(),deltaDistance);
		MultiTilePanoramaAnalyse(dir,cam);
		BelvedereDB::Get().Setup("","");
	}
}

void DialogVisibilite::BasicMultiTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	std::string dir = ui->dirLE->text().toStdString();

	QSettings settings("liris", "virtualcity");
	QString caterogy = ui->categoryLE->text(); 
	settings.setValue("capturecategory",caterogy);
	double deltaDistance = ui->deltaDistanceSB->value();
	settings.setValue("capturedeltadistance",deltaDistance);

	if(dir != "")
	{
		dir+="/";
		BelvedereDB::Get().Setup(dir,caterogy.toStdString(),deltaDistance);
		MultiTileBasicAnalyse(dir,cam);
		BelvedereDB::Get().Setup("","");
	}
}

void DialogVisibilite::BasicMonoTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	QSettings settings("liris", "virtualcity");
	QString caterogy = ui->categoryLE->text(); 
	settings.setValue("capturecategory",caterogy);
	double deltaDistance = ui->deltaDistanceSB->value();
	settings.setValue("capturedeltadistance",deltaDistance);

	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load gml file");

	QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

	if(ext == "gml")
    {
		std::vector<std::string> building;
		building.push_back(filepath.toStdString());
		BasisAnalyse(building,cam);
	}
}


void DialogVisibilite::CascadePanorama()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	QSettings settings("liris", "virtualcity");
	QString caterogy = ui->categoryLE->text(); 
	settings.setValue("capturecategory",caterogy);
	double deltaDistance = ui->deltaDistanceSB->value();
	settings.setValue("capturedeltadistance",deltaDistance);

	std::string dir = ui->dirLE->text().toStdString();

	int count = ui->cascadeCountSB->value();
	float increment = ui->cascadeIncrementSB->value();
	
	if(dir != "")
	{
		dir+="/";
		MultiTileCascadePanoramaAnalyse(dir,cam,count,increment);
	}
}

void DialogVisibilite::CascadeMultiTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	QSettings settings("liris", "virtualcity");
	QString caterogy = ui->categoryLE->text(); 
	settings.setValue("capturecategory",caterogy);
	double deltaDistance = ui->deltaDistanceSB->value();
	settings.setValue("capturedeltadistance",deltaDistance);

	std::string dir = ui->dirLE->text().toStdString();

	int count = ui->cascadeCountSB->value();
	float increment = ui->cascadeIncrementSB->value();
	
	if(dir != "")
	{
		dir+="/";
		MultiTileCascadeAnalyse(dir,cam,count,increment);
	}
}

void DialogVisibilite::CascadeMonoTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	QSettings settings("liris", "virtualcity");
	QString caterogy = ui->categoryLE->text(); 
	settings.setValue("capturecategory",caterogy);
	double deltaDistance = ui->deltaDistanceSB->value();
	settings.setValue("capturedeltadistance",deltaDistance);

	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load gml file");

	QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

	int count = ui->cascadeCountSB->value();
	float increment = ui->cascadeIncrementSB->value();

	if(ext == "gml")
    {
		std::vector<std::string> building;
		building.push_back(filepath.toStdString());
		CascadeAnalyse(building,cam,count,increment);
	}
}

void DialogVisibilite::ResetCategory()
{
	std::string dir = ui->dirLE->text().toStdString();
	dir+="/";
	QString caterogy = ui->categoryLE->text(); 
	BelvedereDB::Get().ResetDB(dir,caterogy.toStdString());
}

void DialogVisibilite::ToolAlignementTree()
{
	ExtrudeAlignementTree();
}

void DialogVisibilite::ToolLidarToGML()
{
	ProcessCL(ProcessLasShpVeget());
}

void DialogVisibilite::ToolShpExtrusion()
{
	ShpExtruction();
}

void DialogVisibilite::ToolFlatRoof()
{

	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load gml file");

	QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

	if(ext == "gml")
    {
		DetectionToitsPlats(filepath.toStdString(),2.0,0.98);
	}
}

void DialogVisibilite::ToolAABBReconstruction()
{
	std::string dir = ui->dirLE->text().toStdString();

	if(dir != "")
	{
		dir+="/";
		BuildAABB(dir);
	}
}

DialogVisibilite::~DialogVisibilite()
{
    delete ui;
}