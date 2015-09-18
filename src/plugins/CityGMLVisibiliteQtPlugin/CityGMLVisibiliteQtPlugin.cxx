#include "CityGMLVisibiliteQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

// add your object header here if you call code...
#include "Dialogs/dialogVisibilite.hpp"

int CityGMLVisibiliteQtPlugin::your_code_here(void)
{
	// add your code (or call your object) here...
	DialogVisibilite* dialVisibilite = new DialogVisibilite(mw, mw);
	dialVisibilite->show();

	int res = 0;

	return res;
}

#if QT_VERSION < 0x050000 // (for QT4)
Q_EXPORT_PLUGIN2(CityGMLVisibiliteQtPlugin, CityGMLVisibiliteQtPlugin)
#endif
