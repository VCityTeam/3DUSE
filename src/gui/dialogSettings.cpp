#include "moc/dialogSettings.hpp"
#include "ui_dialogSettings.h"
#include "core/application.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogSettings::DialogSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogSettings)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogSettings::~DialogSettings()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogSettings::doSettings()
{
    ui->lineEditLowerBoundX->setText("0");
    ui->lineEditLowerBoundY->setText("0");
    ui->lineEditUpperBoundX->setText("0");
    ui->lineEditUpperBoundY->setText("0");

    ui->lineTileSizeX->setText("500");
    ui->lineTileSizeY->setText("500");

    ui->checkBoxTextures->setChecked(vcity::app().getSettings().m_loadTextures);

    if(exec())
    {
        vcity::app().getDataProfile().m_bboxLowerBound = TVec3d(ui->lineEditLowerBoundX->text().toDouble(), ui->lineEditLowerBoundY->text().toDouble());
        vcity::app().getDataProfile().m_bboxUpperBound = TVec3d(ui->lineEditUpperBoundX->text().toDouble(), ui->lineEditUpperBoundY->text().toDouble());

        vcity::app().getDataProfile().m_xStep = ui->lineTileSizeX->text().toFloat();
        vcity::app().getDataProfile().m_yStep = ui->lineTileSizeY->text().toFloat();

        vcity::app().getSettings().m_loadTextures = ui->checkBoxTextures->isChecked();
    }
}
////////////////////////////////////////////////////////////////////////////////
