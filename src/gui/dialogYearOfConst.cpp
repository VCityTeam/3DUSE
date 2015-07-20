#include "moc/DialogYearOfConst.hpp"
#include "ui_DialogYearOfConst.h"
#include "gui/applicationGui.hpp"

DialogYearOfConst::DialogYearOfConst(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogYearOfConst)
{
    ui->setupUi(this);
	connect(ui->checkBoxConst,SIGNAL(toggled(bool)),this,SLOT(constChecked(bool)));
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

    int yearOfConstruction;

    bool a = node->getUserValue("yearOfConstruction", yearOfConstruction);

	ui->checkBoxConst->setChecked(a);
	ui->dateConstrEdit->setEnabled(a);
	if (a) 
	{
		//(yearOfConstruction,1,1);
		ui->dateConstrEdit->setDate(QDate(yearOfConstruction,1,1));
	}

	//window execution
	int res = exec();
	if (res)
	{
		QDate yoC = ui->dateConstrEdit->date();

		if (ui->checkBoxConst->isChecked())
		{
			obj->setAttribute("yearOfConstruction",yoC.toString("yyyy").toStdString());
			node->setUserValue("yearOfConstruction",yoC.year());
		}
	}
}

void DialogYearOfConst::constChecked(bool checked)
{
	ui->dateConstrEdit->setEnabled(checked);
}
