#include "CityGMLCutQtPlugin.h"

#include <QtPlugin>
#include <QMessageBox>

#include "../../utils/cmdline/cityGMLCut.h"

int CityGMLCutQtPlugin::cmdline(QString s_IN, QString s_OUT, unsigned int ui_G_xmin, unsigned int ui_G_ymin, unsigned int ui_G_xmax, unsigned int ui_G_ymax, bool b_VERBOSE)
{
	CityGMLCut cityGMLCut;
	
	fprintf(stdout, "\n[BEGIN] : cityGMLCut\n");
	int res = cityGMLCut.Run((char *)s_IN.toStdString().c_str(), (char *)s_OUT.toStdString().c_str(), ui_G_xmin, ui_G_ymin, ui_G_xmax, ui_G_ymax, b_VERBOSE);
	fprintf(stdout, "[ END ] : cityGMLCut\n");

	if (res)
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLCutQtPlugin: error, see terminal.").exec();
	else
		QMessageBox(QMessageBox::Information,  "3DUSE", "CityGMLCutQtPlugin: end of process, see terminal.").exec();

	return res;
}

#if( defined(BUILD_GUI_QT4) )
Q_EXPORT_PLUGIN2(CityGMLCutQtPlugin, CityGMLCutQtPlugin)
#endif
