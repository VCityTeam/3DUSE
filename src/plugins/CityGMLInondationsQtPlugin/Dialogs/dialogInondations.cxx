#include "dialogInondations.hpp"
#include "ui_dialogInondations.h"
#include <QMessageBox>

#include "../FloodARTools.hpp"
#include "AABB.hpp"
#include "processes/ShpExtrusion.hpp"

////////////////////////////////////////////////////////////////////////////////
dialogInondations::dialogInondations(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::dialogInondations)
{
	ui->setupUi(this);
	connect(ui->btn_ASCcut_in, SIGNAL(clicked()), this, SLOT(browseInputDirASCCut()));
	connect(ui->btn_ASCcut_out, SIGNAL(clicked()), this, SLOT(browseOutputDirASCCut()));
	connect(ui->btn_ASCcut_exec, SIGNAL(clicked()), this, SLOT(cutASC()));
	connect(ui->btn_ASCtoWater_src, SIGNAL(clicked()), this, SLOT(browseInputASCtoWater()));
	connect(ui->chkBox_ASCtoWater_simplify, SIGNAL(stateChanged(int)), this, SLOT(enablePolygonsParams(int)));
	connect(ui->chkBox_ASCtoWater_time, SIGNAL(stateChanged(int)), this, SLOT(enableTemporalParams(int)));
	connect(ui->btn_ASCtoWater_exec, SIGNAL(clicked()), this, SLOT(ASCtoWater()));
	connect(ui->btn_ASCtoTerrain_src1, SIGNAL(clicked()), this, SLOT(browseInput1ASCtoTerrain()));
	connect(ui->ben_ASCtoTerrain_src2, SIGNAL(clicked()), this, SLOT(browseInput2ASCtoTerrain()));
	connect(ui->btn_ASCtoTerrain_tex, SIGNAL(clicked()), this, SLOT(browseTextureASCtoTerrain()));
	connect(ui->chkBox_fusion, SIGNAL(stateChanged(int)), this, SLOT(enableASCFusion(int)));
	connect(ui->chkBox_tex, SIGNAL(stateChanged(int)), this, SLOT(enableTextures(int)));
	connect(ui->btn_ASCtoTerrain_exec, SIGNAL(clicked()), this, SLOT(ASCtoTerrain()));
	connect(ui->btn_ShpExt_dir, SIGNAL(clicked()), this, SLOT(browseInputDirShpExt()));
	connect(ui->btn_ShpExt_in, SIGNAL(clicked()), this, SLOT(browseInputShpExt()));
	connect(ui->btn_ShpExt_exec, SIGNAL(clicked()), this, SLOT(ShpExtrusion()));

	ui->btn_ShpExt_in->setEnabled(false);
	ui->lineEdit_ShpExt_in->setEnabled(false);
	ui->label_15->setEnabled(false);
}
////////////////////////////////////////////////////////////////////////////////
dialogInondations::~dialogInondations()
{
	delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseInputDirASCCut()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCcut_src->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseOutputDirASCCut()
{
	QString path = QFileDialog::getExistingDirectory(this, "Select output directory");
	ui->lineEdit_ASCcut_out->setText(path);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::cutASC()
{
	QFileInfo file(ui->lineEdit_ASCcut_src->text());
	QDir dir(ui->lineEdit_ASCcut_out->text());
	int tileSizeX = ui->spinBox_tileSize_x->value();
	int tileSizeY = ui->spinBox_tileSize_y->value();
	if (!file.exists())
	{
		QMessageBox msgBox;
		msgBox.setText("Input file not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	if (!dir.exists() || ui->lineEdit_ASCcut_out->text().toStdString() == "")
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
		FloodAR::cutASC(file.absoluteFilePath().toStdString(), dir.absolutePath().toStdString(), tileSizeX, tileSizeY);
	}
	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Tiling finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseInputASCtoWater()
{
	QStringList filenames = QFileDialog::getOpenFileNames(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoWater_src->setText(filenames.join(";"));
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::enablePolygonsParams(int state)
{
	switch (state)
	{
	case Qt::Checked:
		ui->label_8->setEnabled(true);
		ui->dbSpinBox_ASCtoWater_prec->setEnabled(true);
		ui->label_9->setEnabled(true);
		break;
	default:
		ui->label_8->setEnabled(false);
		ui->dbSpinBox_ASCtoWater_prec->setEnabled(false);
		ui->label_9->setEnabled(false);
	}
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::enableTemporalParams(int state)
{
	switch (state)
	{
	case Qt::Checked:
		ui->label_10->setEnabled(true);
		ui->label_11->setEnabled(true);
		ui->dtEdit_creationDate->setEnabled(true);
		ui->dtEdit_terminationDate->setEnabled(true);

		break;
	default:
		ui->label_10->setEnabled(false);
		ui->label_11->setEnabled(false);
		ui->dtEdit_creationDate->setEnabled(false);
		ui->dtEdit_terminationDate->setEnabled(false);
	}
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::ASCtoWater()
{
	bool polygonsImport = ui->chkBox_ASCtoWater_simplify->isChecked();
	float prec = ui->dbSpinBox_ASCtoWater_prec->value();
	bool tempImport = ui->chkBox_ASCtoWater_time->isChecked();
	QDateTime creaDate = ui->dtEdit_creationDate->dateTime();
	QDateTime termDate = ui->dtEdit_terminationDate->dateTime();
	if (tempImport && !(creaDate < termDate))
	{
		tempImport = false;
		QMessageBox msgBox;
		msgBox.setText("Invalid temporal settings!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
	}
	QStringList filenames = ui->lineEdit_ASCtoWater_src->text().split(";");
	for (int i = 0; i < filenames.size(); i++)
	{
		QFileInfo file = filenames[i];
		if (!file.exists())
		{
			QMessageBox msgBox;
			msgBox.setText("Input file not found!");
			msgBox.setIcon(QMessageBox::Critical);
			msgBox.exec();
			return;
		}
		FloodAR::ASCtoWater(file.absoluteFilePath().toStdString(), polygonsImport, prec, tempImport, creaDate.toString(Qt::ISODate).toStdString(), termDate.toString(Qt::ISODate).toStdString());
	}
	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Conversion finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////

void dialogInondations::browseInput1ASCtoTerrain()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoTerrain1->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseInput2ASCtoTerrain()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoTerrain2->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseTextureASCtoTerrain()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select texture file file");
	ui->lineEdit_ASCtoTerrainTex->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::enableASCFusion(int state)
{
	switch (state)
	{
	case Qt::Checked:
		ui->label_14->setEnabled(true);
		ui->lineEdit_ASCtoTerrain2->setEnabled(true);
		ui->ben_ASCtoTerrain_src2->setEnabled(true);
		break;
	default:
		ui->label_14->setEnabled(false);
		ui->lineEdit_ASCtoTerrain2->setEnabled(false);
		ui->ben_ASCtoTerrain_src2->setEnabled(false);
	}
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::enableTextures(int state)
{
	switch (state)
	{
	case Qt::Checked:
		ui->lineEdit_ASCtoTerrainTex->setEnabled(true);
		ui->btn_ASCtoTerrain_tex->setEnabled(true);
		break;
	default:
		ui->lineEdit_ASCtoTerrainTex->setEnabled(false);
		ui->btn_ASCtoTerrain_tex->setEnabled(false);
	}
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::ASCtoTerrain()
{
	bool fusion = ui->chkBox_fusion->isChecked();
	QFileInfo file(ui->lineEdit_ASCtoTerrain1->text());
	bool addTextures = ui->chkBox_tex->isChecked();
	QFileInfo texturesPath(ui->lineEdit_ASCtoTerrainTex->text());
	if (!file.exists())
	{
		QMessageBox msgBox;
		msgBox.setText("Input file not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	QFileInfo file2(ui->lineEdit_ASCtoTerrain2->text());
	if (fusion && !file2.exists())
	{
		QMessageBox msgBox;
		msgBox.setText("Input file not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	
	FloodAR::ASCtoTerrain(file.absoluteFilePath().toStdString(), fusion, file2.absoluteFilePath().toStdString(), addTextures, texturesPath.absoluteFilePath().toStdString());

	std::cout << "Job done!" << std::endl;
	QMessageBox msgBox;
	msgBox.setText("Conversion finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseInputShpExt()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select SHP file", "", "SHP files (*.shp)");
	ui->lineEdit_ShpExt_in->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::browseInputDirShpExt()
{
	QString path = QFileDialog::getExistingDirectory(this, "Select data directory");
	ui->lineEdit_ShpExt_dir->setText(path);
}
////////////////////////////////////////////////////////////////////////////////
void dialogInondations::ShpExtrusion()
{
	std::string dir = ui->lineEdit_ShpExt_dir->text().toStdString();
	if (dir != "")
	{
		dir += "/";
		BuildAABB(dir);
		ShpExtruction(dir);
	}
}
