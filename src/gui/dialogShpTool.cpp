// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogShpTool.hpp"
#include "ui_dialogShpTool.h"
#include "gui/applicationGui.hpp"

#include "moc/mainWindow.hpp"

#include <QSettings>
#include <QFileDialog>
#include <osg/MatrixTransform>

////////////////////////////////////////////////////////////////////////////////
DialogShpTool::DialogShpTool(QWidget *parent, MainWindow* mainwindow) :
    QDialog(parent),
    ui(new Ui::DialogShpTool)
{
    ui->setupUi(this);
	this->mainwindow = mainwindow;

	connect(ui->heightSpinBox, SIGNAL(valueChanged(double)), this, SLOT(HeightValueChanged(double)));
}
////////////////////////////////////////////////////////////////////////////////

void DialogShpTool::Setup(const vcity::URI& uri)
{
	std::cout << "Setup Shp : " << std::endl;
	node = mainwindow->m_osgScene->getNode(uri);
	std::cout << node->getName() << std::endl;
	osg::Vec3d translate = GetMatrixTransform()->getMatrix().getTrans();
	ui->heightSpinBox->setValue(translate.z());
	ui->labelNode->setText(("Shp : "+node->getName()).c_str());
}


DialogShpTool::~DialogShpTool()
{
    delete ui;
}

void DialogShpTool::HeightValueChanged(double d)
{
	osg::Matrixd mat;
	mat.makeTranslate(0.0,0.0,d);
	GetMatrixTransform()->setMatrix(mat);
}

osg::MatrixTransform* DialogShpTool::GetMatrixTransform()
{
	osg::Group* group = node->asGroup();

	if(group == nullptr)
	{
		std::cout << "Shp Node is not a group, abording operation." << std::endl;
		return nullptr;
	}

	osg::Transform* transform = group->asTransform();

	if(transform == nullptr)
	{
		std::cout << "Shp Group is not a Transform, abording operation." << std::endl;
		return nullptr;
	}

	osg::MatrixTransform* matrixTransform = transform->asMatrixTransform();

	if(transform == nullptr)
	{
		std::cout << "Shp Transform is not a MatrixTransform, abording operation." << std::endl;
		return nullptr;
	}

	return matrixTransform;
}
