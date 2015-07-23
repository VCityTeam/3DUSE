#include "moc/DialogYearOfDemol.hpp"
#include "ui_dialogYearOfDemol.h"
#include "gui/applicationGui.hpp"

DialogYearOfDemol::DialogYearOfDemol(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogYearOfDemol)
{
    ui->setupUi(this);
	ui->comboBox->insertItem(0,"",QVariant(0));
	ui->comboBox->insertItem(1,"yearOfDemolition",QVariant(1));
	ui->comboBox->insertItem(2,"terminationDate",QVariant(2));
	//currentIndexChanged
	connect(ui->comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(indexChanged(int)));
}

DialogYearOfDemol::~DialogYearOfDemol()
{
    delete ui;
}

void DialogYearOfDemol::editDates(const vcity::URI& uri)
{
	citygml::CityObject* obj = nullptr;
	uri.resetCursor();
    obj = vcity::app().getScene().getCityObjectNode(uri);
	osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);

    bool a = node->getUserValue("yearOfDemolition", yearOfDemolition);
	bool b = node->getUserValue("terminationDate", terminationDate);

	if (a)
	{
		ui->comboBox->setCurrentIndex(1);
		ui->dateDemolEdit->setEnabled(true);
		ui->dateDemolEdit->setDate(QDate(yearOfDemolition,1,1));
	} 
	else if (b)
	{
		ui->comboBox->setCurrentIndex(2);
		ui->dateDemolEdit->setEnabled(true);
		std::string str_tDate = std::to_string(terminationDate);
		QDate tDate = QDate::fromString(QString::fromStdString(str_tDate),QString("yyyyMMdd"));
		ui->dateDemolEdit->setDate(tDate);
	}
	else
	{
		ui->comboBox->setCurrentIndex(0);
		ui->dateDemolEdit->setEnabled(false);
	}

	//window execution
	int res = exec();
	if (res)
	{
		QDate destDate = ui->dateDemolEdit->date();
		int index = ui->comboBox->currentIndex();
		switch (index)
		{
		case 1:
			obj->setAttribute("yearOfDemolition",destDate.toString("yyyy").toStdString());
			node->setUserValue("yearOfDemolition",destDate.year());
			break;
		case 2:
			{
				obj->setAttribute("terminationDate",destDate.toString("yyyy-MM-dd").toStdString());
				int tDate = (destDate.toString("yyyyMMdd")).toInt();
				node->getUserValue("terminationDate",tDate);
			}
			break;
		default:
			//do nothing
			break;
		}
	}
}

void DialogYearOfDemol::indexChanged(int currentIndex)
{
	switch(currentIndex)
	{
	case 0:
		ui->dateDemolEdit->setEnabled(false);
		break;
	case 1:
		{
			ui->dateDemolEdit->setEnabled(true);
			ui->dateDemolEdit->setDate(QDate(yearOfDemolition,1,1));
		}
		break;
	case 2:
		{
			ui->dateDemolEdit->setEnabled(true);
			std::string str_tDate = std::to_string(terminationDate);
			QDate tDate = QDate::fromString(QString::fromStdString(str_tDate),QString("yyyyMMdd"));
			ui->dateDemolEdit->setDate(tDate);
		}
		break;
	default:
		//do nothing
		break;
	}
}