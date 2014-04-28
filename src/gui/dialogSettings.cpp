#include "moc/dialogSettings.hpp"
#include "ui_dialogSettings.h"
#include "gui/applicationGui.hpp"
#include <QSettings>
#include "moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogSettings::DialogSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogSettings)
{
    ui->setupUi(this);
    if(appGui().getMainWindow()->m_unlockLevel < 1)
    {
        ui->tabWidget->removeTab(2); // hide debug tab if not admin
    }
}
////////////////////////////////////////////////////////////////////////////////
DialogSettings::~DialogSettings()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogSettings::doSettings()
{
    ui->lineEditLowerBoundX->setText(QString::number(vcity::app().getSettings().getDataProfile().m_bboxLowerBound.x, 'g', 15));
    ui->lineEditLowerBoundY->setText(QString::number(vcity::app().getSettings().getDataProfile().m_bboxLowerBound.y, 'g', 15));
    ui->lineEditUpperBoundX->setText(QString::number(vcity::app().getSettings().getDataProfile().m_bboxUpperBound.x, 'g', 15));
    ui->lineEditUpperBoundY->setText(QString::number(vcity::app().getSettings().getDataProfile().m_bboxUpperBound.y, 'g', 15));

    ui->lineEditOffsetX->setText(QString::number(vcity::app().getSettings().getDataProfile().m_offset.x, 'g', 15));
    ui->lineEditOffsetY->setText(QString::number(vcity::app().getSettings().getDataProfile().m_offset.y, 'g', 15));

    ui->lineTileSizeX->setText(QString::number(vcity::app().getSettings().getDataProfile().m_xStep));
    ui->lineTileSizeY->setText(QString::number(vcity::app().getSettings().getDataProfile().m_yStep));

    ui->checkBoxTextures->setChecked(vcity::app().getSettings().m_loadTextures);

    if(exec())
    {
        vcity::app().getSettings().getDataProfile().m_bboxLowerBound = TVec3d(ui->lineEditLowerBoundX->text().toDouble(), ui->lineEditLowerBoundY->text().toDouble());
        vcity::app().getSettings().getDataProfile().m_bboxUpperBound = TVec3d(ui->lineEditUpperBoundX->text().toDouble(), ui->lineEditUpperBoundY->text().toDouble());

        vcity::app().getSettings().getDataProfile().m_xStep = ui->lineTileSizeX->text().toFloat();
        vcity::app().getSettings().getDataProfile().m_yStep = ui->lineTileSizeY->text().toFloat();

        vcity::app().getSettings().m_loadTextures = ui->checkBoxTextures->isChecked();
        QSettings settings("liris", "virtualcity");
        settings.setValue("loadtextures", vcity::app().getSettings().m_loadTextures);

        // admin
        if(!ui->lineEdit_AdminPass->text().isEmpty())
        {
            appGui().getMainWindow()->unlockFeatures(ui->lineEdit_AdminPass->text());
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
