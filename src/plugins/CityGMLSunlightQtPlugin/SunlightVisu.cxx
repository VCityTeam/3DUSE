#include "SunlightVisu.h"

#include <QDir>
#include <iostream>
#include <sstream>

#include "core/dateTime.hpp"

SunlightVisu::SunlightVisu()
{
}

void SunlightVisu::loadFile(QString filepath)
{
    QFile file(filepath);

    if(file.open(QIODevice::ReadOnly)) //If file opens succesfully
    {
        while(!file.atEnd())
        {
            QString line = file.readLine(); //Get current line
            QStringList cells = line.split(';');

            QString datetime = cells.first(); //Get first column
            int idatetime = encodeDateTime(datetime.toStdString());

            std::string polygonId = cells.at(1).toStdString(); //Get PolygonId
            std::string sSunlight = cells.at(2).toStdString(); //Get sunlight information as string

            // cast string to boolean value
            bool bSunlight;
            std::istringstream(sSunlight) >> bSunlight;

            //Add value to map
            m_sunlightInfo[idatetime][polygonId] = bSunlight;

        }
    }

}

void SunlightVisu::loadSunlightFiles(QStringList filepaths, QDateTime startDate, QDateTime endDate)
{
    int iStartDate = encodeDateTime(startDate);
    int iEndDate = encodeDateTime(endDate);

    for(int i = 0 ; i < filepaths.size() ; ++i)
    {
        QDir dir(filepaths.at(i));

        if(dir.exists())
        {
            int loadedFileNb = 0;
            for(QFileInfo f : dir.entryInfoList())
            {
                std::string sf = f.fileName().toStdString();

                if(sf.size() < 3)
                    continue;

                int pos = sf.find(".csv");
                std::string f_date = sf.substr(0,pos);

                int iF_date = encodeDateTime(f_date,0);

                if(iF_date >= iStartDate && iF_date < iEndDate)
                {
                    std::cout << "Load Sunlight infos from file : " << dir.absoluteFilePath(f.fileName()).toStdString() << std::endl;
                    loadFile(dir.absoluteFilePath(f.fileName()));

                    ++loadedFileNb;
                }
            }

            int daysNb = (iEndDate - iStartDate) / 24;
//            std::cout << "daysNB : " << daysNb << std::endl;
//            std::cout << "Computed files : " << loadedFileNb << std::endl;
            if(loadedFileNb < daysNb)
                std::cout << "Warning : Sunlight is not computed for all days of the period for file : " << filepaths.at(i).toStdString() << std::endl;
        }
        else
        {
            std::cout << "Warning : Sunlight is not computed for file : " << filepaths.at(i).toStdString() << std::endl;
        }
    }
}

void SunlightVisu::VisualiseSunlight(QDateTime dateTime)
{



}

void SunlightVisu::activate(QDateTime d)
{
    std::cout << "bah " << std::endl;
}
