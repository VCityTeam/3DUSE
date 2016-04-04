#include "CityGMLSunlightQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

// add your object header here if you call code...
// ...

int CityGMLSunlightQtPlugin::your_code_here(unsigned int ui_G_xmin, unsigned int ui_G_ymin, unsigned int ui_G_xmax, unsigned int ui_G_ymax)
{
	// add your object instantiation here if you call code...
	// ...
	
    fprintf(stdout, "\n[BEGIN] : cityGMLSunlight\n");
	// add your code (or call your object) here...
	// ...
	int res = 0;
    fprintf(stdout, "[ END ] : cityGMLSunlight\n");

	if (res)
        QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLSunlightQtPlugin: error, see terminal.").exec();
	else
        QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLSunlightQtPlugin: end of process, see terminal.").exec();

	return res;
}

#if QT_VERSION < 0x050000 // (for QT4)
Q_EXPORT_PLUGIN2(CityGMLSunlightQtPlugin, CityGMLSunlightQtPlugin)
#endif
