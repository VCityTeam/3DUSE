// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "DialogCityGMLCut1.h"
#include "ui_DialogCityGMLCut1.h"
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLCut1::DialogCityGMLCut1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCityGMLCut1)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLCut1::~DialogCityGMLCut1()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLCut1::setGMLFiles(QString IN, QString OUT)
{
    ui->lineEdit_IN->setText(IN);
    ui->lineEdit_OUT->setText(OUT);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLCut1::getGMLFiles(QString& IN, QString& OUT)
{
	IN = ui->lineEdit_IN->text();
	OUT = ui->lineEdit_OUT->text();
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLCut1::setBoundingBox(unsigned int xmin, unsigned int ymin, unsigned int xmax, unsigned int ymax)
{
    ui->lineEdit_Xmin->setText(QString::number(xmin));
    ui->lineEdit_Ymin->setText(QString::number(ymin));
	ui->lineEdit_Xmax->setText(QString::number(xmax));
	ui->lineEdit_Ymax->setText(QString::number(ymax));
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLCut1::getBoundingBox(unsigned int& xmin, unsigned int& ymin, unsigned int& xmax, unsigned int& ymax)
{
	xmin = ui->lineEdit_Xmin->text().toUInt();
	ymin = ui->lineEdit_Ymin->text().toUInt();
	xmax = ui->lineEdit_Xmax->text().toUInt();
	ymax = ui->lineEdit_Ymax->text().toUInt();
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLCut1::setVerbose(bool check)
{
    ui->checkBox_Verbose->setChecked(check);
}
////////////////////////////////////////////////////////////////////////////////
bool DialogCityGMLCut1::getVerbose()
{
	return ui->checkBox_Verbose->isChecked();
}
////////////////////////////////////////////////////////////////////////////////
