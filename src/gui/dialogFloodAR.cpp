#include "moc/dialogFloodAR.hpp"
#include "ui_dialogFloodAR.h"
#include <QFileDialog>
#include <QMessageBox>

#include "gui/osg/osgMnt.hpp"
#include "import/importerASC.hpp"
#include "export/exportCityGML.hpp"

////////////////////////////////////////////////////////////////////////////////
dialogFloodAR::dialogFloodAR(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::dialogFloodAR)
{
	ui->setupUi(this);
	connect(ui->btn_ASCcut_in,SIGNAL(clicked()),this,SLOT(browseInputDirASCCut()));
	connect(ui->btn_ASCcut_out,SIGNAL(clicked()),this,SLOT(browseOutputDirASCCut()));
	connect(ui->btn_ASCcut_exec,SIGNAL(clicked()),this,SLOT(cutASC()));
	connect(ui->btn_ASCtoWater_src,SIGNAL(clicked()),this,SLOT(browseInputASCtoWater()));	
	connect(ui->chkBox_ASCtoWater_simplify ,SIGNAL(stateChanged(int)),this,SLOT( enablePolygonsParams(int)));
	connect(ui->chkBox_ASCtoWater_time ,SIGNAL(stateChanged(int)),this,SLOT( enableTemporalParams(int)));
	connect(ui->btn_ASCtoWater_exec ,SIGNAL(clicked()),this,SLOT( ASCtoWater()));
	connect(ui->btn_ASCtoTerrain_src1 ,SIGNAL(clicked()),this,SLOT( browseInput1ASCtoTerrain()));
	connect(ui->ben_ASCtoTerrain_src2 ,SIGNAL(clicked()),this,SLOT( browseInput2ASCtoTerrain()));
	connect(ui->btn_ASCtoTerrain_tex ,SIGNAL(clicked()),this,SLOT( browseTextureASCtoTerrain()));
	connect(ui->chkBox_fusion ,SIGNAL(stateChanged(int)),this,SLOT( enableASCFusion(int)));
	connect(ui->chkBox_tex ,SIGNAL(stateChanged(int)),this,SLOT( enableTextures(int)));
	connect(ui->btn_ASCtoTerrain_exec ,SIGNAL(clicked()),this,SLOT( ASCtoTerrain()));

	//ui->btn_ASCtoWater_exec->setEnabled(false);
	ui->btn_ASCtoTerrain_exec->setEnabled(false);
}
////////////////////////////////////////////////////////////////////////////////
dialogFloodAR::~dialogFloodAR()
{
	delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputDirASCCut()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCcut_src->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseOutputDirASCCut()
{
	QString path = QFileDialog::getExistingDirectory(this,"Select output directory");
	ui->lineEdit_ASCcut_out->setText(path);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::cutASC()
{
	//cutASC is in ImporterASC now, but maybe this should be changed?
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
	if (!dir.exists()||ui->lineEdit_ASCcut_out->text().toStdString()=="")
	{
		QMessageBox msgBox;
		msgBox.setText("Output directory not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	if (!(tileSizeX>0 && tileSizeY>0))
	{
		QMessageBox msgBox;
		msgBox.setText("Invalid Tile Size");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}
	QString ext = file.suffix().toLower();
	if (ext=="asc")
	{
		//reading file
		citygml::ImporterASC* importer = new citygml::ImporterASC();
		MNT* asc = new MNT();
		if (asc->charge(ui->lineEdit_ASCcut_src->text().toStdString().c_str(), "ASC"))
		{
			importer->cutASC(asc,ui->lineEdit_ASCcut_out->text().toStdString(),file.baseName().toStdString(), tileSizeX, tileSizeY );
		}
		delete importer;
		delete asc;
	}
	std::cout<<"Job done!"<<std::endl;
	QMessageBox msgBox;
	msgBox.setText("Tiling finished!");
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputASCtoWater()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoWater_src->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::enablePolygonsParams(int state)
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
void dialogFloodAR::enableTemporalParams(int state)
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
void dialogFloodAR::ASCtoWater()
{
	bool polygonsImport = ui->chkBox_ASCtoWater_simplify->isChecked();
	float prec = ui->dbSpinBox_ASCtoWater_prec->value();
	bool tempImport = ui->chkBox_ASCtoWater_time->isChecked();
	QDateTime creaDate = ui->dtEdit_creationDate->dateTime();
	QDateTime termDate = ui->dtEdit_terminationDate->dateTime();

	QFileInfo file(ui->lineEdit_ASCtoWater_src->text());
	if (!file.exists())
	{
		QMessageBox msgBox;
		msgBox.setText("Input file not found!");
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();
		return;
	}

	citygml::CityModel* model = new citygml::CityModel();
	std::cout<<"CONVERTING FILE "<<file.baseName().toStdString()<<std::endl;
	QString ext = file.suffix().toLower();
	if (ext=="asc")
	{
		//lecture du fichier
		citygml::ImporterASC* importer = new citygml::ImporterASC();
		MNT* asc = new MNT();
		if (asc->charge(file.absoluteFilePath().toStdString().c_str(), "ASC"))
		{
			//conversion en structure CityGML
			if (polygonsImport)
			{
				model = importer->waterToCityGMLPolygons(asc,prec);
			}
			else
			{
				model = importer->waterToCityGML(asc);
			}
			delete importer;
			delete asc;
		}
	}
	//Add temporal info
	if (tempImport)
	{
		if (!(creaDate<termDate))
		{
			QMessageBox msgBox;
			msgBox.setText("Invalid temporal settings!");
			msgBox.setIcon(QMessageBox::Critical);
			msgBox.exec();
		}
		else
			for (citygml::CityObject* obj : model->getCityObjectsRoots())
			{
				obj->setAttribute("creationDate",creaDate.toString(Qt::ISODate).toStdString());
				obj->setAttribute("terminationDate",termDate.toString(Qt::ISODate).toStdString());
			}
	}
	//export en CityGML
	std::cout<<"Export ...";
	if (model->size()!=0)
	{
		citygml::ExporterCityGML exporter((file.path()+'/'+file.baseName()+".gml").toStdString());
		exporter.exportCityModel(*model);
		std::cout<<"OK!"<<std::endl;
	}
	else std::cout<<std::endl<<"Export aborted: empty CityModel!"<<std::endl;
	delete model;
	std::cout<<"Job done!"<<std::endl;
}
////////////////////////////////////////////////////////////////////////////////

void dialogFloodAR::browseInput1ASCtoTerrain()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoTerrain1->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInput2ASCtoTerrain()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select ASC source file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoTerrain2->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseTextureASCtoTerrain()
{
	QString filename = QFileDialog::getOpenFileName(this, "Select texture file file", "", "ASC files (*.asc)");
	ui->lineEdit_ASCtoTerrainTex->setText(filename);
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::enableASCFusion(int state)
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
void dialogFloodAR::enableTextures(int state)
{
	switch (state)
	{
	case Qt::Checked:
		ui->lineEdit_ASCtoTerrainTex->setEnabled(true);
		ui->btn_ASCtoTerrain_tex->setEnabled(true);
		ui->chkBox_worldfile->setEnabled(true);
		break;
	default:
		ui->lineEdit_ASCtoTerrainTex->setEnabled(false);
		ui->btn_ASCtoTerrain_tex->setEnabled(false);
		ui->chkBox_worldfile->setEnabled(false);
	}
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ASCtoTerrain()
{

}
////////////////////////////////////////////////////////////////////////////////
