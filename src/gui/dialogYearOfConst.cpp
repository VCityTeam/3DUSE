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
	if (uri.getType() == "Tile")
	{
		editTileDates(uri);
	}
	else
	{
		editObjectDates(uri);
	}
}

void DialogYearOfConst::editTileDates(const vcity::URI& uri)
{

	ui->comboBox->setCurrentIndex(0);
	ui->dateConstrEdit->setEnabled(false);

	//window execution
	int res = exec();
	if (res)
	{
		vcity::Tile* tile = vcity::app().getScene().getTile(uri);
		uri.resetCursor();
		osg::ref_ptr<osg::Group> grp = appGui().getOsgScene()->getNode(uri)->asGroup();
		QDate creaDate = ui->dateConstrEdit->date();
		int index = ui->comboBox->currentIndex();
		if(grp)
		{
			for(unsigned int i=0; i<grp->getNumChildren(); ++i)
			{
				osg::ref_ptr<osg::Node> child = grp->getChild(i);
				switch (index)
				{
				case 1:
					//o->setAttribute("yearOfConstruction",creaDate.toString("yyyy").toStdString());
					child->setUserValue("yearOfConstruction",creaDate.year());
					break;
				case 2:
					{
						//o->setAttribute("creationDate",creaDate.toString("yyyy-MM-dd").toStdString());
						int cDate = (creaDate.toString("yyyyMMdd")).toInt();
						child->getUserValue("creationDate",cDate);
					}
					break;
				default:
					//do nothing
					break;
				}
			}
		}
		for(citygml::CityObject* o : tile->getCityModel()->getCityObjectsRoots())
        {
			switch (index)
			{
			case 1:
				o->setAttribute("yearOfConstruction",creaDate.toString("yyyy").toStdString());
				//node->setUserValue("yearOfConstruction",creaDate.year());
				break;
			case 2:
				{
					o->setAttribute("creationDate",creaDate.toString("yyyy-MM-dd").toStdString());
					int cDate = (creaDate.toString("yyyyMMdd")).toInt();
					//node->getUserValue("creationDate",cDate);
				}
				break;
			default:
				//do nothing
				break;
			}               
        }

	}
}

void DialogYearOfConst::editObjectDates(const vcity::URI& uri)
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
				node->setUserValue("creationDate",cDate);
			}
			break;
		default:
			//do nothing
			break;
		}
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