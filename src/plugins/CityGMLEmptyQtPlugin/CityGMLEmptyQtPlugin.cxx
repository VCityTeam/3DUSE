#include "CityGMLEmptyQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

// add your object header here if you call code...
// ...

int CityGMLEmptyQtPlugin::your_code_here(unsigned int ui_G_xmin, unsigned int ui_G_ymin, unsigned int ui_G_xmax, unsigned int ui_G_ymax)
{
	// add your object instantiation here if you call code...
	// ...
	
	fprintf(stdout, "\n[BEGIN] : cityGMLEmpty\n");
	// add your code (or call your object) here...
	// ...
	int res = 0;
	fprintf(stdout, "[ END ] : cityGMLEmpty\n");

	if (res)
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLEmptyQtPlugin: error, see terminal.").exec();
	else
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLEmptyQtPlugin: end of process, see terminal.").exec();

	return res;
}

#if( defined(BUILD_GUI_QT4) )
Q_EXPORT_PLUGIN2(CityGMLEmptyQtPlugin, CityGMLEmptyQtPlugin)
#endif
