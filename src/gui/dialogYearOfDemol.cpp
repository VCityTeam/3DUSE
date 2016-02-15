#include "moc/dialogYearOfDemol.hpp"
#include "ui_dialogYearOfDemol.h"
#include "gui/applicationGui.hpp"
#include "osg/ValueObject"

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
	if (uri.getType() == "Tile")
	{
		editTileDates(uri);
	}
	else
	{
		editObjectDates(uri);
	}
}

void DialogYearOfDemol::editTileDates(const vcity::URI& uri)
	// Called if object is a Tile
{
	ui->comboBox->setCurrentIndex(0);
	ui->dateDemolEdit->setEnabled(false);
	yearOfDemolition = 2000;
	terminationDate = "2000-01-01T00:00:00";

	//window execution
	int res = exec();
	if (res)
	{
		vcity::Tile* tile = vcity::app().getScene().getTile(uri);
		uri.resetCursor();
		osg::ref_ptr<osg::Group> grp = appGui().getOsgScene()->getNode(uri)->asGroup();
		QDateTime destDate = ui->dateDemolEdit->dateTime();
		int index = ui->comboBox->currentIndex();
		// Edit OSG nodes (Tile children)
		if(grp)
		{
			for(unsigned int i=0; i<grp->getNumChildren(); ++i)
			{
				osg::ref_ptr<osg::Node> child = grp->getChild(i);
				switch (index)
				{
				case 1: //user added/edited yearOfDemolition
					child->setUserValue("yearOfDemolition",destDate.date().year());
					break;
				case 2: //user added/edited terminationDate
					child->setUserValue("terminationDate",destDate.toString(Qt::ISODate).toStdString());
					break;
				default:
					//do nothing
					break;
				}
			}
		}
		// Edit CityGML Objects
		for(citygml::CityObject* o : tile->getCityModel()->getCityObjectsRoots())
		{
			switch (index)
			{
			case 1: //user added/edited terminationDate
				o->setAttribute("yearOfDemolition",destDate.toString("yyyy").toStdString());
				break;
			case 2: //user added/edited terminationDate
				{
					o->setAttribute("terminationDate",destDate.toString(Qt::ISODate).toStdString());
				}
				break;
			default:
				//do nothing
				break;
			}
		}
	}
}

void DialogYearOfDemol::editObjectDates(const vcity::URI& uri)
	// Called if object is a not a Tile
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
		ui->dateDemolEdit->setDateTime(QDateTime(QDate(yearOfDemolition,1,1)));
		terminationDate = std::to_string(yearOfDemolition)+"-01-01T00:00:00";
	} 
	else if (b)
	{
		ui->comboBox->setCurrentIndex(2);
		ui->dateDemolEdit->setEnabled(true);
		std::string str_tDate = terminationDate;
		QDateTime tDate = QDateTime::fromString(QString::fromStdString(str_tDate),Qt::ISODate);
		ui->dateDemolEdit->setDateTime(tDate);
		yearOfDemolition = tDate.date().year();
	}
	else
	{
		ui->comboBox->setCurrentIndex(0);
		ui->dateDemolEdit->setEnabled(false);
		yearOfDemolition = 2000;
		terminationDate = "2000-01-01T00:00:00";
	}

	//window execution
	int res = exec();
	if (res)
	{
		QDateTime destDate = ui->dateDemolEdit->dateTime();
		int index = ui->comboBox->currentIndex();
		switch (index)
		{
		case 1: //user added/edited yearOfDemolition
			obj->setAttribute("yearOfDemolition",destDate.toString("yyyy").toStdString());
			node->setUserValue("yearOfDemolition",destDate.date().year());
			break;
		case 2: //user added/edited terminationDate
			obj->setAttribute("terminationDate",destDate.toString(Qt::ISODate).toStdString());
			node->setUserValue("terminationDate",destDate.toString(Qt::ISODate).toStdString());
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
			ui->dateDemolEdit->setDateTime(QDateTime(QDate(yearOfDemolition,1,1)));
		}
		break;
	case 2:
		{
			ui->dateDemolEdit->setEnabled(true);
			std::string str_tDate = terminationDate;
			QDateTime tDate = QDateTime::fromString(QString::fromStdString(str_tDate),Qt::ISODate);
			ui->dateDemolEdit->setDateTime(tDate);
		}
		break;
	default:
		//do nothing
		break;
	}
}