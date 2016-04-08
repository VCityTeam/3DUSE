#include "CityGMLSunlightQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

// add your object header here if you call code...
#include "Dialogs/DialogCityGMLSunlight1.h"

int CityGMLSunlightQtPlugin::your_code_here(void)
{
    // add your code (or call your object) here...
    DialogCityGMLSunlight1* dialSunlight = new DialogCityGMLSunlight1(mw);
    dialSunlight->show();

    int res = 0;

    return res;
}

#if QT_VERSION < 0x050000 // (for QT4)
Q_EXPORT_PLUGIN2(CityGMLSunlightQtPlugin, CityGMLSunlightQtPlugin)
#endif
