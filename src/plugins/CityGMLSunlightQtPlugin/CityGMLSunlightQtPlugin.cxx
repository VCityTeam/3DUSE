#include "CityGMLSunlightQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

#include "Dialogs/DialogCityGMLSunlight1.h"

int CityGMLSunlightQtPlugin::init(void)
{
    DialogCityGMLSunlight1* dialSunlight = new DialogCityGMLSunlight1(mw);

    dialSunlight->show();

    int res = 0;

    return res;
}

#if QT_VERSION < 0x050000 // (for QT4)
Q_EXPORT_PLUGIN2(CityGMLSunlightQtPlugin, CityGMLSunlightQtPlugin)
#endif
