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
    if(!appGui().getMainWindow()->m_adminMode)
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
        QSettings settings("liris", "virtualcity");
        settings.setValue("loadtextures", vcity::app().getSettings().m_loadTextures);

        // admin
        if(!ui->lineEdit_AdminPass->text().isEmpty())
        {
            appGui().getMainWindow()->adminMode(ui->lineEdit_AdminPass->text() == "pass");
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
