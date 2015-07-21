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
		DialogCityGMLVisibilite1 dial1;
		//dial1.setBoundingBox(1845500, 5174000, 1846000, 5174500);
		if (dial1.exec() == QDialog::Accepted)
		{
			unsigned int xmin,ymin,xmax,ymax;
			dial1.getBoundingBox(xmin,ymin,xmax,ymax);

			your_code_here(xmin, ymin, xmax ,ymax);

			return true;
		}

		return false;
    }

private:
	int your_code_here(unsigned int ui_G_xmin, unsigned int ui_G_ymin, unsigned int ui_G_xmax, unsigned int ui_G_ymax);
};

#endif // CityGMLVisibiliteQtPlugin_H
