#include "moc/dialogEditBldg.hpp"
#include "ui_dialogEditBldg.h"
////////////////////////////////////////////////////////////////////////////////
DialogEditBldg::DialogEditBldg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEditBldg)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogEditBldg::~DialogEditBldg()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::setName(const QString& str)
{
    ui->lineEdit->setText(str);
}
////////////////////////////////////////////////////////////////////////////////
QString DialogEditBldg::getName() const
{
    return ui->lineEdit->text();
}
////////////////////////////////////////////////////////////////////////////////
/*void DialogEditBldg::setEnvelope(double a, double b, double c, double d)
{
    ui->lineEditLowerBoundX->setText(QString::number(a));
    ui->lineEditLowerBoundY->setText(QString::number(b));
    //ui->lineEditUpperBoundX->setText(QString::number(c));
    //ui->lineEditUpperBoundY->setText(QString::number(d));
}*/
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::setOffset(double x, double y)
{
    ui->lineEditLowerBoundX->setText(QString::number(x));
    ui->lineEditLowerBoundY->setText(QString::number(y));
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::getOffset(double& x, double& y)
{
 x = ui->lineEditLowerBoundX->text().toDouble();
 y = ui->lineEditLowerBoundY->text().toDouble();
}
////////////////////////////////////////////////////////////////////////////////
