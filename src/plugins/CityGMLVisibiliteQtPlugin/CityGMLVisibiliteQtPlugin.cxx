// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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

#if( defined(BUILD_GUI_QT4) )
Q_EXPORT_PLUGIN2(CityGMLVisibiliteQtPlugin, CityGMLVisibiliteQtPlugin)
#endif
