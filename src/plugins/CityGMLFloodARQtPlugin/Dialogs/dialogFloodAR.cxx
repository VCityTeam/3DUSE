#include "dialogFloodAR.hpp"
#include "ui_dialogFloodAR.h"
#include <QMessageBox>

#include "../FloodARTools.hpp"
#include "AABB.hpp"
#include "processes/ShpExtrusion.hpp"

////////////////////////////////////////////////////////////////////////////////
dialogFloodAR::dialogFloodAR(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::dialogFloodAR)
{
	ui->setupUi(this);
  connect(ui->btn_exit, SIGNAL(clicked()), this, SLOT(close()));
  connect(ui->btn_wkingDir, SIGNAL(clicked()), this, SLOT(browseWorkingDirectory()));
  connect(ui->btn_ASCcut_in, SIGNAL(clicked()), this, SLOT(browseInputASCCut()));
	connect(ui->btn_ASCcut_exec, SIGNAL(clicked()), this, SLOT(cutASC()));
	connect(ui->btn_ASCtoWater_exec, SIGNAL(clicked()), this, SLOT(ASCtoWater()));
	connect(ui->btn_ASCtoTerrain_exec, SIGNAL(clicked()), this, SLOT(ASCtoTerrain()));
	connect(ui->btn_ShpExt_in, SIGNAL(clicked()), this, SLOT(browseInputShpExt()));
	connect(ui->btn_ShpExt_exec, SIGNAL(clicked()), this, SLOT(ShpExtrusion()));
	connect(ui->btn_texCut_in, SIGNAL(clicked()), this, SLOT(browseInputTextureCut()));
	connect(ui->btn_texCut_exec, SIGNAL(clicked()), this, SLOT(textureCut()));
  
  // Disable SHP Extrusion "SHP file" input
  ui->btn_ShpExt_in->setEnabled(false);
	ui->lineEdit_ShpExt_in->setEnabled(false);
	ui->label_15->setEnabled(false);

  // init ASCCut radio group
  ui->radioBtn_ASCCut_terrain->setChecked(true);
}
////////////////////////////////////////////////////////////////////////////////
dialogFloodAR::~dialogFloodAR()
{
	delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseWorkingDirectory()
{
  QString path = QFileDialog::getExistingDirectory(this, "Select working directory");
  ui->lineEdit_wkingDir->setText(path);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputASCCut()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCcut_src->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::cutASC()
{
	QFileInfo file(ui->lineEdit_ASCcut_src->text());
	QDir dir(ui->lineEdit_wkingDir->text());
	int tileSizeX = ui->spinBox_tileSize_x->value();
	int tileSizeY = ui->spinBox_tileSize_y->value();
  bool isTerrain = ui->radioBtn_ASCCut_terrain->isChecked();
	if (!file.exists())
	{
		QMessageBox msgBox;
		msgBox.setText("Input file not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	if (!dir.exists() || ui->lineEdit_wkingDir->text().toStdString() == "")
	{
		QMessageBox msgBox;
		msgBox.setText("Output directory not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	if (!(tileSizeX > 0 && tileSizeY > 0))
	{
		QMessageBox msgBox;
		msgBox.setText("Invalid Tile Size");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	QString ext = file.suffix().toLower();
	if (ext == "asc")
	{
		FloodAR::cutASC(file.absoluteFilePath().toStdString(), dir.absolutePath().toStdString(), tileSizeX, tileSizeY, isTerrain);
	}
	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Tiling finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputTextureCut()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select texture file", "", "");
	ui->lineEdit_texCut_src->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::textureCut()
{
	QFileInfo file(ui->lineEdit_texCut_src->text());
	int tileSizeX = ui->spinBox_txTileSize_x->value();
	int tileSizeY = ui->spinBox_txTileSize_y->value();
	if (!file.exists())
	{
		QMessageBox msgBox;
		msgBox.setText("Input file not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	if (!(tileSizeX > 0 && tileSizeY > 0))
	{
		QMessageBox msgBox;
		msgBox.setText("Invalid Tile Size");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}

	FloodAR::cutPicture(file.absoluteFilePath().toStdString(), ui->lineEdit_wkingDir->text().toStdString(), tileSizeX, tileSizeY);

	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Tiling finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ASCtoWater()
{
	float prec = ui->dbSpinBox_ASCtoWater_prec->value();
	QDateTime creaDate = ui->dtEdit_creationDate->dateTime();
  QDir dir(ui->lineEdit_wkingDir->text());
  
  FloodAR::ASCtoWaterAuto(dir.absolutePath().toStdString(), prec, creaDate.toString(Qt::ISODate).toStdString());
	
	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Conversion finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ASCtoTerrain()
{
  std::string dir = ui->lineEdit_wkingDir->text().toStdString();
	FloodAR::ASCtoTerrain(dir);

	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Conversion finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputShpExt()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select SHP file", "", "SHP files (*.shp)");
	ui->lineEdit_ShpExt_in->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ShpExtrusion()
{
	std::string dir = ui->lineEdit_wkingDir->text().toStdString();
	if (dir != "")
	{
		dir += "/";
		BuildAABB(dir);
		ShpExtruction(dir);
	}
}
