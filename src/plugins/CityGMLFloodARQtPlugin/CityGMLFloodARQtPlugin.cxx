// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "CityGMLFloodARQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

// add your object header here if you call code...
// ...

int CityGMLFloodARQtPlugin::your_code_here()
{
	// add your object instantiation here if you call code...
	// ...
	
	fprintf(stdout, "\n[BEGIN] : cityGMLFloodAR\n");
	// add your code (or call your object) here...
	// ...
	int res = 0;
	fprintf(stdout, "[ END ] : cityGMLFloodAR\n");

	if (res)
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLFloodARQtPlugin: error, see terminal.").exec();
	else
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLFloodARQtPlugin: end of process, see terminal.").exec();

	return res;
}

#if( defined(BUILD_GUI_QT4) )
Q_EXPORT_PLUGIN2(CityGMLFloodARQtPlugin, CityGMLFloodARQtPlugin)
#endif
