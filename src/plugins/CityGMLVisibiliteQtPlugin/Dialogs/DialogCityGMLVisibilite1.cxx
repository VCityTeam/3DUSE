#include "DialogCityGMLVisibilite1.h"
#include "ui_DialogCityGMLVisibilite1.h"
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLVisibilite1::DialogCityGMLVisibilite1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCityGMLVisibilite1)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLVisibilite1::~DialogCityGMLVisibilite1()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLVisibilite1::setBoundingBox(unsigned int xmin, unsigned int ymin, unsigned int xmax, unsigned int ymax)
{
    ui->lineEdit_Xmin->setText(QString::number(xmin));
    ui->lineEdit_Ymin->setText(QString::number(ymin));
	ui->lineEdit_Xmax->setText(QString::number(xmax));
	ui->lineEdit_Ymax->setText(QString::number(ymax));
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLVisibilite1::getBoundingBox(unsigned int& xmin, unsigned int& ymin, unsigned int& xmax, unsigned int& ymax)
{
	xmin = ui->lineEdit_Xmin->text().toUInt();
	ymin = ui->lineEdit_Ymin->text().toUInt();
	xmax = ui->lineEdit_Xmax->text().toUInt();
	ymax = ui->lineEdit_Ymax->text().toUInt();
}
////////////////////////////////////////////////////////////////////////////////
