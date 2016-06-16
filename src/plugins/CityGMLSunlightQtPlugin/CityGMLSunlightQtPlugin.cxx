#include "CityGMLSunlightQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>
#include <QDir>

#include <iostream>
#include <sstream>

#include "Dialogs/DialogCityGMLSunlight1.h"
#include "core/dateTime.hpp"
#include "gui/osg/osgScene.hpp"

int CityGMLSunlightQtPlugin::init(void)
{
    DialogCityGMLSunlight1* dialSunlight = new DialogCityGMLSunlight1(mw);

    m_visu = false;

    connect(appGui().getMainWindow(),SIGNAL(activateVisuSunlightPlugin(QDateTime)),this,SLOT(ChangePolyColor(QDateTime))); //when current datetime changes in mainwindow
    connect(dialSunlight,SIGNAL(startVisu(QStringList, QDateTime, QDateTime)),this,SLOT(startVisuAction(QStringList, QDateTime, QDateTime))); //when click on start visu button
    connect(dialSunlight,SIGNAL(stopVisu()),this,SLOT(stopVisuAction())); //when click on stop visu button

    dialSunlight->show();

    int res = 0;

    return res;
}

void CityGMLSunlightQtPlugin::ChangePolyColor(QDateTime datetime)
{
    if(m_visu) //If visualization started
    {
        int idatetime = encodeDateTime(datetime); //encode datetime into int value
        appGui().getOsgScene()->changePolyColor(m_sunlightInfo[idatetime]);
    }
}

void CityGMLSunlightQtPlugin::startVisuAction(QStringList filepaths, QDateTime startDate, QDateTime endDate)
{
    loadSunlightFiles(filepaths,startDate,endDate);
    m_visu = true;
}

void CityGMLSunlightQtPlugin::stopVisuAction()
{
    //Delete maps of map m_sunlightInfo
    for(auto iterator = m_sunlightInfo.begin(); iterator != m_sunlightInfo.end(); iterator++)
        delete iterator->second;

    m_sunlightInfo.clear();
    m_visu = false;
}

void CityGMLSunlightQtPlugin::loadFile(QString filepath)
{
    QFile file(filepath);

    if(file.open(QIODevice::ReadOnly)) //If file opens succesfully
    {
        while(!file.atEnd())
        {
            QString line = file.readLine(); //Get current line
            QStringList cells = line.split(';');

            QString datetime = cells.first(); //Get cell of first column
            int idatetime = encodeDateTime(datetime.toStdString());

            std::string polygonId = cells.at(1).toStdString(); //Get PolygonId
            std::string sSunlight = cells.at(2).toStdString(); //Get sunlight information as string

            // cast string to boolean value
            bool bSunlight;
            std::istringstream(sSunlight) >> bSunlight;

            //Add value to map
            if(m_sunlightInfo[idatetime] == NULL)
            {
                m_sunlightInfo[idatetime] = new std::map<std::string,bool>();
            }
            (*(m_sunlightInfo[idatetime]))[polygonId] = bSunlight;
        }
    }

}

void CityGMLSunlightQtPlugin::loadSunlightFiles(QStringList filepaths, QDateTime startDate, QDateTime endDate)
{
    int iStartDate = encodeDateTime(startDate);
    int iEndDate = encodeDateTime(endDate);

    for(int i = 0 ; i < filepaths.size() ; ++i) //For all selected files to load
    {
        QDir dir(filepaths.at(i));

        if(dir.exists())
        {
            int loadedFileNb = 0;
            for(QFileInfo f : dir.entryInfoList()) // Each f is a file containing all sunlight infos for a given day and tile
            {
                std::string sf = f.fileName().toStdString();

                if(sf.size() < 3) //for an unknown reason, entryInfoList() returns files named ' ', '.', '..'. This is to skip this cases
                    continue;

                size_t pos = sf.find(".csv");
                std::string f_date = sf.substr(0,pos); //get the date from the name of the file

                int iF_date = encodeDateTime(f_date,0); //Date represented by the file encoded into int

                if(iF_date >= iStartDate && iF_date < iEndDate) //if the file contains sunlight info in the interval of visualization
                {
                    std::cout << "Load Sunlight infos from file : " << dir.absoluteFilePath(f.fileName()).toStdString() << std::endl;
                    loadFile(dir.absoluteFilePath(f.fileName()));

                    ++loadedFileNb;
                }
            }

            //Check if all needed sunlight infos for visu have been loaded
            int daysNb = (iEndDate - iStartDate) / 24;

            if(loadedFileNb < daysNb)
                std::cout << "Warning : Sunlight is not computed for all days of the period for file : " << filepaths.at(i).toStdString() << std::endl;
        }
        else
        {
            std::cout << "Warning : Sunlight is not computed for file : " << filepaths.at(i).toStdString() << std::endl;
        }
    }
}

#if QT_VERSION < 0x050000 // (for QT4)
Q_EXPORT_PLUGIN2(CityGMLSunlightQtPlugin, CityGMLSunlightQtPlugin)
#endif
