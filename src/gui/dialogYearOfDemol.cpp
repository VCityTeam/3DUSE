#include "moc/DialogYearOfDemol.hpp"
#include "ui_dialogYearOfDemol.h"
#include "gui/applicationGui.hpp"

DialogYearOfDemol::DialogYearOfDemol(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogYearOfDemol)
{
    ui->setupUi(this);
	connect(ui->checkBoxDemol,SIGNAL(toggled(bool)),this,SLOT(constChecked(bool)));
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

    int yearOfDemolition;

    bool a = node->getUserValue("yearOfDemolition", yearOfDemolition);

	ui->checkBoxDemol->setChecked(a);
	ui->dateDemolEdit->setEnabled(a);
	if (a) 
	{
		//(yearOfDemolition,1,1);
		ui->dateDemolEdit->setDate(QDate(yearOfDemolition,1,1));
	}

	//window execution
	int res = exec();
	if (res)
	{
		QDate yoD = ui->dateDemolEdit->date();

		if (ui->checkBoxDemol->isChecked())
		{
			obj->setAttribute("yearOfDemolition",yoD.toString("yyyy").toStdString());
			node->setUserValue("yearOfDemolition",yoD.year());
		}
	}
}

void DialogYearOfDemol::constChecked(bool checked)
{
	ui->dateDemolEdit->setEnabled(checked);
}
