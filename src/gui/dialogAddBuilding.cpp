// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogAddBuilding.hpp"
#include "ui_dialogAddBuilding.h"
////////////////////////////////////////////////////////////////////////////////
DialogAddBuilding::DialogAddBuilding(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogAddBuilding)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogAddBuilding::~DialogAddBuilding()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
