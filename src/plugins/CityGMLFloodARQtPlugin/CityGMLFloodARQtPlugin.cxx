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
