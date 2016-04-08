#include "DialogCityGMLSunlight1.h"
#include "ui_DialogCityGMLSunlight1.h"
#include "../SunlightDetection.h"
#include "../FileInfo.h"

#include <iostream>

#include <QDir>
#include <QSettings>
#include <QFileDialog>

////////////////////////////////////////////////////////////////////////////////
DialogCityGMLSunlight1::DialogCityGMLSunlight1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCityGMLSunlight1)
{
    ui->setupUi(this);

    //** Connect Buttons
    //Files Management
    connect(ui->dirFiles_B,SIGNAL(clicked()),this,SLOT(DirFilesButtonClicked()));
    connect(ui->AddFile_B,SIGNAL(clicked()),this,SLOT(AddFileButtonClicked()));
    connect(ui->RemoveFile_B,SIGNAL(clicked()),this,SLOT(RemoveFileButtonClicked()));
    connect(ui->AddAllFiles_B,SIGNAL(clicked()),this,SLOT(AddAllFilesButtonClicked()));
    connect(ui->ClearAllFiles_B,SIGNAL(clicked()),this,SLOT(ClearAllFilesButtonClicked()));
    //AnnualSunPath
    connect(ui->AnnualSunPath_B,SIGNAL(clicked()),this,SLOT(AnnualSunPathButtonClicked()));
    //Start calculations
    connect(ui->CreateSunlightFiles_B,SIGNAL(clicked()),this,SLOT(CreateSunlightFilesButtonClicked()));

    //*** Pre-fill Path and Lists
    //Gets previous tile directory
    QSettings settings("liris", "virtualcity");
    QString filedir = settings.value("filedir").toString();
    ui->dirFiles_LE->setText(filedir);

    //Add files of this directory into NonSelectedFiles List
    AddItemsFromDirToList(filedir);

    //Gets previous Annual SunPath file
    QString annualsunpath = settings.value("annualsunpathfile").toString();
    ui->AnnualSunPath_LE->setText(annualsunpath);

    //Gets previous dates
    QString format = "yyyy-MM-dd";

    QString sStartDate = settings.value("sunlightstartdate").toString();
    QDate startDate = QDate::fromString(sStartDate,format);
    ui->StartDate->setDate(startDate);

    QString sEndDate = settings.value("sunlightenddate").toString();
    QDate endDate = QDate::fromString(sEndDate,format);
    ui->EndDate->setDate(endDate);

    //*** Create output folders
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
void DialogCityGMLSunlight1::AddItemsFromDirToList(QString dirpath)
{
    QDir dirfiles(dirpath);
    for(QFileInfo f : dirfiles.entryInfoList())
    {
        if(f.isDir() == true && (f.fileName() == "_BATI" || f.fileName() == "_MNT"))
        {
            QDir dir(f.filePath());

            for(QString file : dir.entryList())
            {
                if(file.contains(".gml"))
                {
                    QString filename = f.fileName() + "/" + file;
                    ui->NonSelectedFiles_List->addItem(filename);
                }
            }
        }
    }
}

void DialogCityGMLSunlight1::DirFilesButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Get File Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("filedir",dirpath);

    //Add files to NonSelectedFiles list
    ui->NonSelectedFiles_List->clear();
    AddItemsFromDirToList(dirpath);

    //Add path to directory in LineEdit
    ui->dirFiles_LE->setText(dirpath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::AddFileButtonClicked()
{
    for(QListWidgetItem* item : ui->NonSelectedFiles_List->selectedItems())
    {
        ui->SelectedFiles_List->addItem(item->text());
    }

    qDeleteAll(ui->NonSelectedFiles_List->selectedItems());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::RemoveFileButtonClicked()
{
    for(QListWidgetItem* item : ui->SelectedFiles_List->selectedItems())
    {
        ui->NonSelectedFiles_List->addItem(item->text());
    }

    qDeleteAll(ui->SelectedFiles_List->selectedItems());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::AddAllFilesButtonClicked()
{
    for(int i = 0; i < ui->NonSelectedFiles_List->count(); ++i)
    {
        QString sItem = ui->NonSelectedFiles_List->item(i)->text();
        ui->SelectedFiles_List->addItem(sItem);
    }

    ui->NonSelectedFiles_List->clear();
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::ClearAllFilesButtonClicked()
{
    for(int i = 0; i < ui->SelectedFiles_List->count(); ++i)
    {
        QString sItem = ui->SelectedFiles_List->item(i)->text();
        ui->NonSelectedFiles_List->addItem(sItem);
    }

    ui->SelectedFiles_List->clear();
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
void DialogCityGMLSunlight1::CreateSunlightFilesButtonClicked()
{
    //Get directory containing files
    QString fileDir = ui->dirFiles_LE->text();

    //Get filenames to compute sunlight for
    std::vector<FileInfo*> files;
    for(int i = 0; i < ui->SelectedFiles_List->count(); ++i)
    {
        FileInfo* f = new FileInfo(fileDir.toStdString() + "/" + ui->SelectedFiles_List->item(i)->text().toStdString());
        files.push_back(f);
    }

    //Get path to annual sunpath file
    std::string sunpathFile = ui->AnnualSunPath_LE->text().toStdString();

    //Get dates
    QString format = "yyyy-MM-dd";

    QString startDate = ui->StartDate->date().toString(format);
    QString endDate = ui->EndDate->date().toString(format);

    //Add them to the settings
    QSettings settings("liris", "virtualcity");

    settings.setValue("sunlightstartdate",startDate);
    settings.setValue("sunlightenddate",endDate);

    if(fileDir != "" && sunpathFile != "")
    {
        fileDir+="/";
        SunlightDetection(fileDir.toStdString(), files, sunpathFile, startDate.toStdString(), endDate.toStdString());
    }   
}
////////////////////////////////////////////////////////////////////////////////
