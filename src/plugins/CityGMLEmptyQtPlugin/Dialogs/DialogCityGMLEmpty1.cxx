// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "DialogCityGMLEmpty1.h"
#include "ui_DialogCityGMLEmpty1.h"
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLEmpty1::DialogCityGMLEmpty1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCityGMLEmpty1)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLEmpty1::~DialogCityGMLEmpty1()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLEmpty1::setBoundingBox(unsigned int xmin, unsigned int ymin, unsigned int xmax, unsigned int ymax)
{
    ui->lineEdit_Xmin->setText(QString::number(xmin));
    ui->lineEdit_Ymin->setText(QString::number(ymin));
	ui->lineEdit_Xmax->setText(QString::number(xmax));
	ui->lineEdit_Ymax->setText(QString::number(ymax));
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLEmpty1::getBoundingBox(unsigned int& xmin, unsigned int& ymin, unsigned int& xmax, unsigned int& ymax)
{
	xmin = ui->lineEdit_Xmin->text().toUInt();
	ymin = ui->lineEdit_Ymin->text().toUInt();
	xmax = ui->lineEdit_Xmax->text().toUInt();
	ymax = ui->lineEdit_Ymax->text().toUInt();
}
////////////////////////////////////////////////////////////////////////////////
