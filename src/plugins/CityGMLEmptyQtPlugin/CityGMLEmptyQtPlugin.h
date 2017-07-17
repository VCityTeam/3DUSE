// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef CITYGMLEmptyQTPLUGIN_H
#define CITYGMLEmptyQTPLUGIN_H

#include "gui/pluginInterface.h"

#include <QStringList>
#include "Dialogs/DialogCityGMLEmpty1.h"

class CityGMLEmptyQtPlugin : 
  public QObject,
  public Generic_PluginInterface
{
    Q_OBJECT
    Q_INTERFACES(Generic_PluginInterface)	
#if(BUILD_GUI_QT5) // Refer to the end of .cxx for QT4 equivalent
    Q_PLUGIN_METADATA(IID "CityGMLEmptyQtPlugin")
#endif
        
    QStringList Generic_plugins() const
	{
		return QStringList() << "CityGMLEmptyQtPlugin";
	}

    bool Generic_plugin(const QString &plugin)
    {
		DialogCityGMLEmpty1 dial1;
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

#endif // CITYGMLEmptyQTPLUGIN_H
