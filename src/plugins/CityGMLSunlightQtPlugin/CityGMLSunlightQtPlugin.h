#ifndef CITYGMLSunlightQTPLUGIN_H
#define CITYGMLSunlightQTPLUGIN_H

#include "../gui/pluginInterface.h"

#include <QStringList>
#include "Dialogs/DialogCityGMLSunlight1.h"

class CityGMLSunlightQtPlugin :
  public QObject,
  public Generic_PluginInterface
{
    Q_OBJECT
    Q_INTERFACES(Generic_PluginInterface)	
#if (WITH_QT5) // see at the end of .cxx for QT4
    Q_PLUGIN_METADATA(IID "CityGMLSunlightQtPlugin")
#endif
        
    QStringList Generic_plugins() const
	{
        return QStringList() << "CityGMLSunlightQtPlugin";
	}

    bool Generic_plugin(const QString &plugin)
    {
        init();
        return true;
    }

    void loadSunlightFiles(QStringList filepaths, QDateTime startDate, QDateTime endDate);
    void VisualiseSunlight(QDateTime dateTime);

public slots:
    void ChangePolyColor(QDateTime datetime);
    void startVisuAction(QStringList filepaths, QDateTime startDate, QDateTime endDate);
    void stopVisuAction();

private:
    int init(void);
    void loadFile(QString filepath);

    std::map<int,std::map<std::string,bool>> m_sunlightInfo; // int: dateTime ; string : idPolygon ; bool : sunny
    bool m_visu; // true if visu start button clicked, false otherwise
};

#endif // CITYGMLSunlightQTPLUGIN_H
