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

private:
    int init(void);
};

#endif // CITYGMLSunlightQTPLUGIN_H
