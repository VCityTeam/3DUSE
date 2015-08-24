// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogVisibilite.hpp"
#include "ui_dialogVisibilite.h"
#include "gui/applicationGui.hpp"

#include "moc/mainWindow.hpp"

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

	this->mainwindow = mainwindow;
}
////////////////////////////////////////////////////////////////////////////////

void DialogVisibilite::DirButtonClicked()
{
	QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Get Tile Directory");
	ui->dirLE->setText(dirpath);
}

void DialogVisibilite::GetCamParam()
{
	osg::Camera* cam = mainwindow->m_osgView->m_osgView->getCamera();
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;
	cam->getViewMatrixAsLookAt(pos,target,up);

	target = target - pos;
	target.normalize();

	ui->posXSB->setValue(pos.x());
	ui->posYSB->setValue(pos.y());
	ui->posZSB->setValue(pos.z());
	ui->dirXSB->setValue(target.x());
	ui->dirYSB->setValue(target.y());
	ui->dirZSB->setValue(target.z());

}

void DialogVisibilite::SetCamParam()
{
	osg::Camera* cam = mainwindow->m_osgView->m_osgView->getCamera();
	osg::Vec3d pos;
	osg::Vec3d target;
	osg::Vec3d up;

	cam->getViewMatrixAsLookAt(pos,target,up);

	pos = osg::Vec3(ui->posXSB->value(),ui->posYSB->value(),ui->posZSB->value());
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
	
	if(dir != "")
	{
		dir+="/";
		MultiTilePanoramaAnalyse(dir,mainwindow->m_app.getSettings().getDataProfile().m_offset,cam);
	}
}

void DialogVisibilite::BasicMultiTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	std::string dir = ui->dirLE->text().toStdString();
	
	if(dir != "")
	{
		dir+="/";
		MultiTileBasicAnalyse(dir,mainwindow->m_app.getSettings().getDataProfile().m_offset,cam);
	}
}

void DialogVisibilite::BasicMonoTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load gml file");

	QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

	if(ext == "gml")
    {
		std::vector<std::string> building;
		building.push_back(filepath.toStdString());
		BasisAnalyse(building,mainwindow->m_app.getSettings().getDataProfile().m_offset,cam);
	}
}


void DialogVisibilite::CascadePanorama()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	std::string dir = ui->dirLE->text().toStdString();

	int count = ui->cascadeCountSB->value();
	float increment = ui->cascadeIncrementSB->value();
	
	if(dir != "")
	{
		dir+="/";
		MultiTileCascadePanoramaAnalyse(dir,mainwindow->m_app.getSettings().getDataProfile().m_offset,cam,count,increment);
	}
}

void DialogVisibilite::CascadeMultiTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	std::string dir = ui->dirLE->text().toStdString();

	int count = ui->cascadeCountSB->value();
	float increment = ui->cascadeIncrementSB->value();
	
	if(dir != "")
	{
		dir+="/";
		MultiTileCascadeAnalyse(dir,mainwindow->m_app.getSettings().getDataProfile().m_offset,cam,count,increment);
	}
}

void DialogVisibilite::CascadeMonoTile()
{
	osg::ref_ptr<osg::Camera> cam = SetupRenderingCamera();

	QString filepath = QFileDialog::getOpenFileName(nullptr,"Load gml file");

	QFileInfo file(filepath);

    QString ext = file.suffix().toLower();

	int count = ui->cascadeCountSB->value();
	float increment = ui->cascadeIncrementSB->value();

	if(ext == "gml")
    {
		std::vector<std::string> building;
		building.push_back(filepath.toStdString());
		CascadeAnalyse(building,mainwindow->m_app.getSettings().getDataProfile().m_offset,cam,count,increment);
	}
}

DialogVisibilite::~DialogVisibilite()
{
    delete ui;
}