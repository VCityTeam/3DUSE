#ifndef SUNLIGHTVISU_H
#define SUNLIGHTVISU_H

#include <map>

#include <QDateTime>

class SunlightVisu
{
public:
    SunlightVisu();

    std::map<int,std::map<std::string,bool>> sunlightInfo; // int: dateTime ; string : idPolygon ; bool : sunny
};

std::map<int,std::map<std::string,bool>>* loadSunlightFiles(QStringList filepaths, QDateTime startDate, QDateTime endDate);
void VisualiseSunlight(QDateTime dateTime);

#endif // SUNLIGHTVISU_H
