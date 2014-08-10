#include "moc/dialogSettings.hpp"
#include "ui_dialogSettings.h"
#include "gui/applicationGui.hpp"
#include <QSettings>
#include <QFileDialog>
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

    connect(ui->comboBoxDataProfile, SIGNAL(activated(int)), this, SLOT(chooseDataProfileSlot(int)));
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(choosePathSlot()));
}
////////////////////////////////////////////////////////////////////////////////
DialogSettings::~DialogSettings()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogSettings::doSettings()
{
    ui->comboBoxDataProfile->addItem("None");
    ui->comboBoxDataProfile->addItem("Paris");
    ui->comboBoxDataProfile->addItem("Lyon");

    ui->comboBoxDataProfile->setCurrentIndex(vcity::app().getSettings().getDataProfile().m_id);

    ui->comboBoxSRS->addItem("lambert93");

    setFromDataProfile(appGui().getSettings().getDataProfile());

    ui->checkBoxTextures->setChecked(vcity::app().getSettings().m_loadTextures);

    if(exec())
    {
        vcity::app().getSettings().getDataProfile() = m_tmpDP;
        setDataProfileFromUI(vcity::app().getSettings().getDataProfile());
        appGui().getOsgScene()->updateGrid();

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
void DialogSettings::choosePathSlot()
{
    QString path = QFileDialog::getExistingDirectory(this, "Choose data source path");
    ui->lineEditDataPath->setText(path);
}
////////////////////////////////////////////////////////////////////////////////
void DialogSettings::chooseDataProfileSlot(int i)
{
    //std::cout << ui->comboBoxDataProfile->itemText(i).toStdString() << std::endl;
    switch(i)
    {
    case 0:
        m_tmpDP = vcity::createDataProfileNone();
        break;
    case 1:
        m_tmpDP = vcity::createDataProfileParis();
        break;
    case 2:
        m_tmpDP = vcity::createDataProfileLyon();
        break;
    default:
        m_tmpDP = vcity::createDataProfileDefault();
        break;
    }

    setFromDataProfile(m_tmpDP);
}
////////////////////////////////////////////////////////////////////////////////
void DialogSettings::setFromDataProfile(const vcity::DataProfile& dp)
{
    ui->lineEditLowerBoundX->setText(QString::number(dp.m_bboxLowerBound.x, 'g', 15));
    ui->lineEditLowerBoundY->setText(QString::number(dp.m_bboxLowerBound.y, 'g', 15));
    ui->lineEditUpperBoundX->setText(QString::number(dp.m_bboxUpperBound.x, 'g', 15));
    ui->lineEditUpperBoundY->setText(QString::number(dp.m_bboxUpperBound.y, 'g', 15));

    ui->lineEditOffsetX->setText(QString::number(dp.m_offset.x, 'g', 15));
    ui->lineEditOffsetY->setText(QString::number(dp.m_offset.y, 'g', 15));

    ui->lineTileSizeX->setText(QString::number(dp.m_xStep));
    ui->lineTileSizeY->setText(QString::number(dp.m_yStep));

    // save parameter
    QSettings settings("liris", "virtualcity");
    settings.setValue("dataprofile", dp.m_name.c_str());
}
////////////////////////////////////////////////////////////////////////////////
void DialogSettings::setDataProfileFromUI(vcity::DataProfile& dp)
{
    dp.m_bboxLowerBound = TVec3d(ui->lineEditLowerBoundX->text().toDouble(), ui->lineEditLowerBoundY->text().toDouble());
    dp.m_bboxUpperBound = TVec3d(ui->lineEditUpperBoundX->text().toDouble(), ui->lineEditUpperBoundY->text().toDouble());

    dp.m_offset.x = ui->lineEditOffsetX->text().toDouble();
    dp.m_offset.y = ui->lineEditOffsetY->text().toDouble();

    dp.m_xStep = ui->lineTileSizeX->text().toFloat();
    dp.m_yStep = ui->lineTileSizeY->text().toFloat();
}
////////////////////////////////////////////////////////////////////////////////
