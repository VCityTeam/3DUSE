#include "CityGMLInondationsQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

// add your object header here if you call code...
// ...

int CityGMLInondationsQtPlugin::your_code_here()
{
	// add your object instantiation here if you call code...
	// ...
	
	fprintf(stdout, "\n[BEGIN] : cityGMLInondations\n");
	// add your code (or call your object) here...
	// ...
	int res = 0;
	fprintf(stdout, "[ END ] : cityGMLInondations\n");

	if (res)
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLInondationsQtPlugin: error, see terminal.").exec();
	else
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLInondationsQtPlugin: end of process, see terminal.").exec();

	return res;
}

#if QT_VERSION < 0x050000 // (for QT4)
Q_EXPORT_PLUGIN2(CityGMLInondationsQtPlugin, CityGMLInondationsQtPlugin)
#endif
