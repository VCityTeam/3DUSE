// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogDoc.hpp"
#include "ui_dialogDoc.h"
#include "gui/applicationGui.hpp"
#include "moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogDoc::DialogDoc(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogDoc)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogDoc::~DialogDoc()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogDoc::addDoc(const vcity::URI& uri)
{
    appGui().getMainWindow()->m_osgView->setActive(false);

    uri.resetCursor();
    citygml::CityObject* obj = vcity::app().getScene().getCityObjectNode(uri);

    if (obj && exec())
    {

    }

    appGui().getMainWindow()->m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
