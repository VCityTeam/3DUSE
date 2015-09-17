#ifndef CityGMLVisibiliteQtPlugin_H
#define CityGMLVisibiliteQtPlugin_H

#include "../gui/pluginInterface.h"

#include <QStringList>
#include "Dialogs/DialogCityGMLVisibilite1.h"

class CityGMLVisibiliteQtPlugin : 
  public QObject,
  public Generic_PluginInterface
{
    Q_OBJECT
    Q_INTERFACES(Generic_PluginInterface)	
#if (WITH_QT5) // see at the end of .cxx for QT4
    Q_PLUGIN_METADATA(IID "CityGMLVisibiliteQtPlugin")
#endif
        
    QStringList Generic_plugins() const
	{
		return QStringList() << "CityGMLVisibiliteQtPlugin";
	}

    bool Generic_plugin(const QString &plugin)
    {
		your_code_here();

		return true;
    }

private:
	int your_code_here(void);
};

#endif // CityGMLVisibiliteQtPlugin_H
