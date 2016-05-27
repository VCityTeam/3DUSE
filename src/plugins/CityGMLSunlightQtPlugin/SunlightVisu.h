#ifndef SUNLIGHTVISU_H
#define SUNLIGHTVISU_H

#include <map>
#include <QDateTime>

class SunlightVisu : public QObject
{
//Q_OBJECT

public:
    SunlightVisu();
    void loadSunlightFiles(QStringList filepaths, QDateTime startDate, QDateTime endDate);

    void VisualiseSunlight(QDateTime dateTime);

public slots:
    void activate(QDateTime d);

private:
    void loadFile(QString filepath);
    std::map<int,std::map<std::string,bool>> m_sunlightInfo; // int: dateTime ; string : idPolygon ; bool : sunny
};



#endif // SUNLIGHTVISU_H
