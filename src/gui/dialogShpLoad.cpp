// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogShpLoad.hpp"
#include "ui_dialogShpLoad.h"
#include "gui/applicationGui.hpp"

#include "moc/mainWindow.hpp"

#include <QSettings>
#include <QFileDialog>
#include <osg/MatrixTransform>

////////////////////////////////////////////////////////////////////////////////
DialogShpLoad::DialogShpLoad(QWidget *parent, MainWindow* mainwindow) :
    QDialog(parent),
    ui(new Ui::DialogShpLoad)
{
    ui->setupUi(this);
	this->mainwindow = mainwindow;

}
////////////////////////////////////////////////////////////////////////////////

DialogShpLoad::~DialogShpLoad()
{
    delete ui;
}

