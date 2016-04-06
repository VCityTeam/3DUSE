#ifndef CITYGMLInondationsQTPLUGIN_H
#define CITYGMLInondationsQTPLUGIN_H

#include "../gui/pluginInterface.h"

#include <QStringList>
#include "Dialogs/dialogInondations.hpp"

class CityGMLInondationsQtPlugin : 
  public QObject,
  public Generic_PluginInterface
{
    Q_OBJECT
    Q_INTERFACES(Generic_PluginInterface)	
#if (WITH_QT5) // see at the end of .cxx for QT4
    Q_PLUGIN_METADATA(IID "CityGMLInondationsQtPlugin")
#endif
        
    QStringList Generic_plugins() const
	{
		return QStringList() << "CityGMLInondationsQtPlugin";
	}

    bool Generic_plugin(const QString &plugin)
    {
		dialogInondations dial1;
		if (dial1.exec() == QDialog::Accepted)
		{
			your_code_here();

			return true;
		}

		return false;
    }

private:
	int your_code_here();
};

#endif // CITYGMLInondationsQTPLUGIN_H
