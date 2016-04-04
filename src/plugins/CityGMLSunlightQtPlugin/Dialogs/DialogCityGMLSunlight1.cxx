#include "DialogCityGMLSunlight1.h"
#include "ui_DialogCityGMLSunlight1.h"

#include <QDir>
#include <QSettings>
#include <QFileDialog>

////////////////////////////////////////////////////////////////////////////////
DialogCityGMLSunlight1::DialogCityGMLSunlight1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCityGMLSunlight1)
{
    ui->setupUi(this);

    //Disable Writing into LineEdits
    ui->dirTiles_LE->setEnabled(false);
    ui->AnnualSunPath_LE->setEnabled(false);

    //Connect Buttons
    connect(ui->dirTiles_B,SIGNAL(clicked()),this,SLOT(DirTilesButtonClicked()));
    connect(ui->AnnualSunPath_B,SIGNAL(clicked()),this,SLOT(AnnualSunPathButtonClicked()));

    //Gets previous tile directory
    QSettings settings("liris", "virtualcity");
    QString tiledir = settings.value("tiledir").toString();
    ui->dirTiles_LE->setText(tiledir);

    //Gets previous Annual SunPath file
    QString annualsunpath = settings.value("annualsunpathfile").toString();
    ui->dirTiles_LE->setText(annualsunpath);

    //Create output folders
    QDir outputDir("./SunlightOutput/");
    if(!outputDir.exists("./SunlightOutput/"))
        outputDir.mkpath(outputDir.absolutePath());

    QDir outputDirBati("./SunlightOutput/_BATI");
    if(!outputDirBati.exists("./SunlightOutput/_BATI"))
        outputDirBati.mkpath(outputDirBati.absolutePath());

    QDir outputDirMnt("./SunlightOutput/_MNT");
    if(!outputDirMnt.exists("./SunlightOutput/_MNT"))
        outputDirMnt.mkpath(outputDirMnt.absolutePath());
}
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLSunlight1::~DialogCityGMLSunlight1()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::DirTilesButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Get Tile Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("tiledir",dirpath);
    ui->dirTiles_LE->setText(dirpath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::AnnualSunPathButtonClicked()
{
    QString filepath = QFileDialog::getOpenFileName(nullptr,"Get Annual SunPath File");
    QSettings settings("liris", "virtualcity");
    settings.setValue("annualsunpathfile",filepath);
    ui->AnnualSunPath_LE->setText(filepath);
}
////////////////////////////////////////////////////////////////////////////////
