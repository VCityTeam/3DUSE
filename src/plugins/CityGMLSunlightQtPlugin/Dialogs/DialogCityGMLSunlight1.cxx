#include "DialogCityGMLSunlight1.h"
#include "ui_DialogCityGMLSunlight1.h"
#include "../SunlightDetection.h"
#include "../FileInfo.h"

#include <QDir>
#include <QSettings>
#include <QFileDialog>

#include "gui/applicationGui.hpp"
#include "gui/moc/mainWindow.hpp"

////////////////////////////////////////////////////////////////////////////////
DialogCityGMLSunlight1::DialogCityGMLSunlight1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCityGMLSunlight1)
{
    //this->setWindowFlags(Qt::WindowMinimizeButtonHint); Flag to add minimize button -> not working

    ui->setupUi(this);

    this->setModal(false);

    //disable stop visualisation button
    ui->StopVisu_B->setEnabled(false);

    //** Connect Buttons
    // First tab : Calculations
    //Files Management
    connect(ui->dirFiles_B,SIGNAL(clicked()),this,SLOT(DirFilesButtonClicked()));
    connect(ui->AddFile_B,SIGNAL(clicked()),this,SLOT(AddFileButtonClicked()));
    connect(ui->RemoveFile_B,SIGNAL(clicked()),this,SLOT(RemoveFileButtonClicked()));
    connect(ui->AddAllFiles_B,SIGNAL(clicked()),this,SLOT(AddAllFilesButtonClicked()));
    connect(ui->ClearAllFiles_B,SIGNAL(clicked()),this,SLOT(ClearAllFilesButtonClicked()));
    //AnnualSunPath
    connect(ui->AnnualSunPath_B,SIGNAL(clicked()),this,SLOT(AnnualSunPathButtonClicked()));
    //Dates
    connect(ui->StartDate,SIGNAL(dateChanged(QDate)),this,SLOT(StartDateChanged()));
    connect(ui->EndDate,SIGNAL(dateChanged(QDate)),this,SLOT(EndDateChanged()));
    //Output
    connect(ui->OutputDir_B,SIGNAL(clicked()),this,SLOT(OutpoutDirButtonClicked()));
    //Create Sunlight Files
    connect(ui->CreateSunlightFiles_B,SIGNAL(clicked()),this,SLOT(CreateSunlightFilesButtonClicked()));  

    // Second Tab : Visualisation
    connect(ui->InputDirectory_B,SIGNAL(clicked()),this,SLOT(InputDirButtonClicked()));
    connect(ui->VisuAddFile_B,SIGNAL(clicked()),this,SLOT(VisuAddFileButtonClicked()));
    connect(ui->VisuRemoveFile_B,SIGNAL(clicked()),this,SLOT(VisuRemoveFileButtonClicked()));
    connect(ui->VisuAddAllFiles_B,SIGNAL(clicked()),this,SLOT(VisuAddAllFilesButtonClicked()));
    connect(ui->VisuClearAllFiles_B,SIGNAL(clicked()),this,SLOT(VisuClearAllFilesButtonClicked()));
    connect(ui->VisudirFiles_B,SIGNAL(clicked()),this,SLOT(VisuDirFilesButtonClicked()));
    connect(ui->StartVisu_B,SIGNAL(clicked()),this,SLOT(StartVisuButtonClicked()));
    connect(ui->StopVisu_B,SIGNAL(clicked()),this,SLOT(StopVisuButtonClicked()));

    //*** Pre-fill Path and Lists
    //Gets previous tile directory
    QSettings settings("liris", "virtualcity");
    QString filedir = settings.value("filedir").toString();
    ui->dirFiles_LE->setText(filedir);

    //Fill Selected Files list with selected files from last utilisation
    int size = settings.beginReadArray("visuselectedfiles1");

    for(int i = 0; i < size; ++i)
    {
        //Add them to QSettings
        settings.setArrayIndex(i);
        ui->SelectedFiles_List->addItem(settings.value("selectedfile").toString());
    }

    settings.endArray();

    //Add files of this directory into NonSelectedFiles List
    AddItemsFromDirToList(filedir);

    //Gets previous Annual SunPath file
    QString annualsunpath = settings.value("annualsunpathfile").toString();
    ui->AnnualSunPath_LE->setText(annualsunpath);

    //Gets previous output directory
    QString outputDir = settings.value("outputdirsunlight").toString();
    ui->OutputDirectory_LE->setText(outputDir);

    //Gets previous visu input directory
    QString inputDir = settings.value("inputdirsunlightvisu").toString();
    ui->InputDirectory_LE->setText(inputDir);

    //Add files from this input directory into VisuNonSelectedFiles List
    AddComputedFilesToList(inputDir);

    //Fill visu files dir
    QString filesdir = settings.value("dirfilessunlightvisu").toString();
    ui->VisudirFiles_LE->setText(filesdir);

    //Gets previous dates
    QString format = "yyyy-MM-dd";

    QString sStartDate = settings.value("sunlightstartdate").toString();
    QDate startDate = QDate::fromString(sStartDate,format);
    ui->StartDate->setDate(startDate);
    ui->VisuStartDate->setDate(startDate);

    QString sEndDate = settings.value("sunlightenddate").toString();
    QDate endDate = QDate::fromString(sEndDate,format);
    ui->EndDate->setDate(endDate);
    ui->VisuEndDate->setDate(endDate);

}
////////////////////////////////////////////////////////////////////////////////
DialogCityGMLSunlight1::~DialogCityGMLSunlight1()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::DirFilesButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Choose File Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("filedir",dirpath);

    //Clear Lists and Add files to NonSelectedFiles list
    ui->SelectedFiles_List->clear();
    ui->NonSelectedFiles_List->clear();
    AddItemsFromDirToList(dirpath);

    //Add path to directory in LineEdit
    ui->VisudirFiles_LE->setText(dirpath);
    ui->dirFiles_LE->setText(dirpath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::AddFileButtonClicked()
{
    for(QListWidgetItem* item : ui->NonSelectedFiles_List->selectedItems()) //For all selected items
    {
        ui->SelectedFiles_List->addItem(item->text());
    }

    qDeleteAll(ui->NonSelectedFiles_List->selectedItems()); //Delete selected items from non-selected list
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
    QString filepath = QFileDialog::getOpenFileName(nullptr,"Choose Annual SunPath File");
    QSettings settings("liris", "virtualcity");
    settings.setValue("annualsunpathfile",filepath);
    ui->AnnualSunPath_LE->setText(filepath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::OutpoutDirButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Choose Output Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("outputdirsunlight",dirpath);

    //Add path to directory in LineEdit
    ui->OutputDirectory_LE->setText(dirpath);
    ui->InputDirectory_LE->setText(dirpath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::CreateSunlightFilesButtonClicked()
{
    //Get directory containing files
    QString fileDir = ui->dirFiles_LE->text();

    //Get filenames to compute sunlight for and add them to Qsettings
    std::vector<FileInfo*> files;

    QSettings settings("liris", "virtualcity");
    settings.beginWriteArray("visuselectedfiles1");

    for(int i = 0; i < ui->SelectedFiles_List->count(); ++i)
    {
        //Add them to QSettings
        settings.setArrayIndex(i);
        settings.setValue("selectedfile", ui->SelectedFiles_List->item(i)->text());

        //Get filepath
        FileInfo* f = new FileInfo(fileDir.toStdString() + "/" + ui->SelectedFiles_List->item(i)->text().toStdString());
        files.push_back(f);
    }

    settings.endArray();

    //Get path to annual sunpath file
    std::string sunpathFile = ui->AnnualSunPath_LE->text().toStdString();

    //Get dates
    QString format = "yyyy-MM-dd";

    QString startDate = ui->StartDate->date().toString(format);
    QString endDate = ui->EndDate->date().toString(format);

    //Add them to the settings
    settings.setValue("sunlightstartdate",startDate);
    settings.setValue("sunlightenddate",endDate);

    //Gets Output directory
    QString outputDir = ui->OutputDirectory_LE->text();

    if(fileDir != "" && sunpathFile != "" && outputDir != "")
    {
        fileDir += "/";
        outputDir += "/";
        SunlightDetection(fileDir.toStdString(), files, sunpathFile, startDate.toStdString(), endDate.toStdString(), outputDir);
    }   

    //After calculation, change Visu inputdir to Calculation output dir and rerun the input dir scan in visualise tab
    ui->InputDirectory_LE->setText(ui->OutputDirectory_LE->text());
    AddComputedFilesToList(ui->InputDirectory_LE->text());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::StartDateChanged()
{
    ui->VisuStartDate->setDate(ui->StartDate->date());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::EndDateChanged()
{
    ui->VisuEndDate->setDate(ui->EndDate->date());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::InputDirButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Choose Sunlight Input Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("inputdirsunlightvisu",dirpath);

    //Add files to VisuNonSelectedFiles list
    ui->VisuNonSelectedFiles_List->clear();
    ui->VisuSelectedFiles_List->clear();
    AddComputedFilesToList(dirpath);

    //Add path to directory in LineEdit
    ui->InputDirectory_LE->setText(dirpath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::VisuAddFileButtonClicked()
{
    for(QListWidgetItem* item : ui->VisuNonSelectedFiles_List->selectedItems())
    {
        ui->VisuSelectedFiles_List->addItem(item->text());
    }

    qDeleteAll(ui->VisuNonSelectedFiles_List->selectedItems());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::VisuRemoveFileButtonClicked()
{
    for(QListWidgetItem* item : ui->VisuSelectedFiles_List->selectedItems())
    {
        ui->VisuNonSelectedFiles_List->addItem(item->text());
    }

    qDeleteAll(ui->VisuSelectedFiles_List->selectedItems());
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::VisuAddAllFilesButtonClicked()
{
    for(int i = 0; i < ui->VisuNonSelectedFiles_List->count(); ++i)
    {
        QString sItem = ui->VisuNonSelectedFiles_List->item(i)->text();
        ui->VisuSelectedFiles_List->addItem(sItem);
    }

    ui->VisuNonSelectedFiles_List->clear();
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::VisuClearAllFilesButtonClicked()
{
    for(int i = 0; i < ui->VisuSelectedFiles_List->count(); ++i)
    {
        QString sItem = ui->VisuSelectedFiles_List->item(i)->text();
        ui->VisuNonSelectedFiles_List->addItem(sItem);
    }

    ui->VisuSelectedFiles_List->clear();
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::VisuDirFilesButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Choose Files Directory");
    QSettings settings("liris", "virtualcity");
    settings.setValue("dirfilessunlightvisu",dirpath);

    ui->VisudirFiles_LE->setText(dirpath);
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::StartVisuButtonClicked()
{
    // ** GUI management

    //Disable all buttons and enables stop visualisation button
    ui->InputDirectory_B->setEnabled(false);
    ui->InputDirectory_LE->setEnabled(false);
    ui->VisuNonSelectedFiles_List->setEnabled(false);
    ui->VisuSelectedFiles_List->setEnabled(false);
    ui->VisuAddFile_B->setEnabled(false);
    ui->VisuRemoveFile_B->setEnabled(false);
    ui->VisuAddAllFiles_B->setEnabled(false);
    ui->VisuClearAllFiles_B->setEnabled(false);
    ui->VisudirFiles_LE->setEnabled(false);
    ui->VisudirFiles_B->setEnabled(false);
    ui->VisuStartDate->setEnabled(false);
    ui->VisuEndDate->setEnabled(false);
    ui->TimeIncrement_ComboBox->setEnabled(false);
    ui->StartVisu_B->setEnabled(false);

    ui->StopVisu_B->setEnabled(true);

    //Change start date and end date in mainwindow
    QSettings settings ("liris","virtualcity");

    //Add dates to settings
    QDateTime startDate = ui->VisuStartDate->dateTime();
    vcity::app().getSettings().m_startDate = startDate.toString(Qt::ISODate).toStdString();
    settings.setValue("startDate",startDate.toString(Qt::ISODate));

    QDateTime endDate = ui->VisuEndDate->dateTime();
    endDate = endDate.addDays(1); //In plugin, endDateTime is displayed with time at 23:59 which match endDateTime+1 at 00:00 in settings, so we need to add one day to the displayed day to update settings the right way
    vcity::app().getSettings().m_endDate = endDate.toString(Qt::ISODate).toStdString();
    settings.setValue("endDate",endDate.toString(Qt::ISODate));

    //Change dates in MainWindow
    appGui().getMainWindow()->initTemporalTools();

    //Enable time in mainwindow
    appGui().getMainWindow()->ChangecheckBoxTemporalToolsState();


    //*** Load Selected files
    QStringList filepaths;

    for(int i = 0; i < ui->VisuSelectedFiles_List->count(); ++i)
    {
        QString fp = ui->VisudirFiles_LE->text() + "/" + ui->VisuSelectedFiles_List->item(i)->text() + ".gml"; //filepath

        appGui().getMainWindow()->loadFile(fp);

        filepaths.append(ui->InputDirectory_LE->text() + "/" + ui->VisuSelectedFiles_List->item(i)->text());
    }

    //Signal to notify CityGMLSunlightQtPlugin that visualization is activated
    emit startVisu(filepaths, startDate, endDate);

    // Tests to bring mainwindow foreground and send Sunlight background
    //appGui()->->setWindowState(Qt::WindowActive);
//    appGui().getMainWindow()->raise();
//    appGui().getMainWindow()->activateWindow();
//    appGui().getMainWindow()->showNormal();
    //this->lower();
    //appGui().getMainWindow()->raise();
    //appGui().getMainWindow()->setWindowState(Qt::WindowActive);

//    appGui().getMainWindow()->setWindowState( (windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
//    appGui().getMainWindow()->raise();  // for MacOS
//    appGui().getMainWindow()->activateWindow(); // for Windows

}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::StopVisuButtonClicked()
{
    //Enable all buttons and disables stop visualisation
    ui->InputDirectory_B->setEnabled(true);
    ui->InputDirectory_LE->setEnabled(true);
    ui->VisuNonSelectedFiles_List->setEnabled(true);
    ui->VisuSelectedFiles_List->setEnabled(true);
    ui->VisuAddFile_B->setEnabled(true);
    ui->VisuRemoveFile_B->setEnabled(true);
    ui->VisuAddAllFiles_B->setEnabled(true);
    ui->VisuClearAllFiles_B->setEnabled(true);
    ui->VisudirFiles_LE->setEnabled(true);
    ui->VisudirFiles_B->setEnabled(true);
    ui->VisuStartDate->setEnabled(true);
    ui->VisuEndDate->setEnabled(true);
    ui->TimeIncrement_ComboBox->setEnabled(true);
    ui->StartVisu_B->setEnabled(true);

    ui->StopVisu_B->setEnabled(false);

    //Disable time in mainwindow
    appGui().getMainWindow()->ChangecheckBoxTemporalToolsState();

    emit stopVisu();

}
////////////////////////////////////////////////////////////////////////////////
bool DialogCityGMLSunlight1::ListContains(QListWidget* list, QString item)
{
    bool found = false;

    for (int i = 0; i < list->count(); ++i)
    {
        if (list->item(i)->text() == item)
        {
            found = true;
            break;
        }
    }

    return found;
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::AddItemsFromDirToList(QString dirpath)
{
    QDir dirfiles(dirpath);
    for(QFileInfo f : dirfiles.entryInfoList()) //for each file/folder in dirfiles
    {
        if(f.isDir() == true && (f.fileName() == "_BATI" || f.fileName() == "_MNT")) //we only compute sunlight for _BATI and _MNT
        {
            QDir dir(f.filePath());

            for(QString file : dir.entryList()) //for all files
            {
                if(file.contains(".gml"))
                {
                    QString filename = f.fileName() + "/" + file; // filename ex : _BATI/3070_10383

                    if(!ListContains(ui->SelectedFiles_List,filename))  //if not already in selected list
                        ui->NonSelectedFiles_List->addItem(filename);
                }
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogCityGMLSunlight1::AddComputedFilesToList(QString dirpath)
{
    QDir dirfiles(dirpath);
    for(QFileInfo f : dirfiles.entryInfoList())
    {
        if(f.isDir() == true && (f.fileName() == "_BATI" || f.fileName() == "_MNT"))
        {
            QDir dir(f.filePath());

            for(QFileInfo file : dir.entryInfoList())
            {
                if(file.isDir() && file.fileName() != "." && file.fileName() != "..")
                {
                    QString filename = f.fileName() + "/" + file.fileName();
                    ui->VisuNonSelectedFiles_List->addItem(filename);
                }
            }
        }
    }
}
