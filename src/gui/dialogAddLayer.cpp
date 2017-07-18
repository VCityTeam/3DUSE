// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogAddLayer.hpp"
#include "ui_dialogAddLayer.h"
#include "gui/applicationGui.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogAddLayer::DialogAddLayer(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogAddLayer)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogAddLayer::~DialogAddLayer()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogAddLayer::addLayer()
{
    if (exec() && !ui->lineEdit->text().isEmpty())
    {
        appGui().getControllerGui().addLayer(ui->lineEdit->text().toStdString());
    }
}
////////////////////////////////////////////////////////////////////////////////
