#include "moc/DialogYearOfConst.hpp"
#include "ui_DialogYearOfConst.h"
#include "gui/applicationGui.hpp"

DialogYearOfConst::DialogYearOfConst(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogYearOfConst)
{
    ui->setupUi(this);
	ui->comboBox->insertItem(0,"",QVariant(0));
	ui->comboBox->insertItem(1,"yearOfConstruction",QVariant(1));
	ui->comboBox->insertItem(2,"creationDate",QVariant(2));
	//currentIndexChanged
	connect(ui->comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(indexChanged(int)));
}

DialogYearOfConst::~DialogYearOfConst()
{
    delete ui;
}

void DialogYearOfConst::editDates(const vcity::URI& uri)
{
	citygml::CityObject* obj = nullptr;
	uri.resetCursor();
    obj = vcity::app().getScene().getCityObjectNode(uri);
	osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);

    bool a = node->getUserValue("yearOfConstruction", yearOfConstruction);
	bool b = node->getUserValue("creationDate", creationDate);

	if (a)
	{
		ui->comboBox->setCurrentIndex(1);
		ui->dateConstrEdit->setEnabled(true);
		ui->dateConstrEdit->setDate(QDate(yearOfConstruction,1,1));
	} 
	else if (b)
	{
		ui->comboBox->setCurrentIndex(2);
		ui->dateConstrEdit->setEnabled(true);
		std::string str_cDate = std::to_string(creationDate);
		QDate cDate = QDate::fromString(QString::fromStdString(str_cDate),QString("yyyyMMdd"));
		ui->dateConstrEdit->setDate(cDate);
	}
	else
	{
		ui->comboBox->setCurrentIndex(0);
		ui->dateConstrEdit->setEnabled(false);
	}

	//ui->checkBoxConst->setChecked(a);
	//ui->dateConstrEdit->setEnabled(a);
	//if (a) 
	//{
	//	//(yearOfConstruction,1,1);
	//	ui->dateConstrEdit->setDate(QDate(yearOfConstruction,1,1));
	//}

	//window execution
	int res = exec();
	if (res)
	{
		QDate creaDate = ui->dateConstrEdit->date();
		int index = ui->comboBox->currentIndex();
		switch (index)
		{
		case 1:
			obj->setAttribute("yearOfConstruction",creaDate.toString("yyyy").toStdString());
			node->setUserValue("yearOfConstruction",creaDate.year());
			break;
		case 2:
			{
				obj->setAttribute("creationDate",creaDate.toString("yyyy-MM-dd").toStdString());
				int cDate = (creaDate.toString("yyyyMMdd")).toInt();
				node->getUserValue("creationDate",cDate);
			}
			break;
		default:
			//do nothing
			break;
		}
		//QDate yoC = ui->dateConstrEdit->date();

		//if (ui->checkBoxConst->isChecked())
		//{
		//	obj->setAttribute("yearOfConstruction",yoC.toString("yyyy").toStdString());
		//	node->setUserValue("yearOfConstruction",yoC.year());
		//}
	}
}

void DialogYearOfConst::indexChanged(int currentIndex)
{
	switch(currentIndex)
	{
	case 0:
		ui->dateConstrEdit->setEnabled(false);
		break;
	case 1:
		{
			ui->dateConstrEdit->setEnabled(true);
			ui->dateConstrEdit->setDate(QDate(yearOfConstruction,1,1));
		}
		break;
	case 2:
		{
			ui->dateConstrEdit->setEnabled(true);
			std::string str_cDate = std::to_string(creationDate);
			QDate cDate = QDate::fromString(QString::fromStdString(str_cDate),QString("yyyyMMdd"));
			ui->dateConstrEdit->setDate(cDate);
		}
		break;
	default:
		//do nothing
		break;
	}
}