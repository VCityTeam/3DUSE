// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogEditTile.hpp"
#include "ui_dialogEditTile.h"
#include "gui/applicationGui.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogEditTile::DialogEditTile(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEditTile)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogEditTile::~DialogEditTile()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditTile::editTile(const vcity::URI& uri)
{
    uri.resetCursor();
    vcity::Tile* tile = vcity::app().getScene().getTile(uri);
    if (tile)
    {
        setName(tile->getName().c_str());

        if (exec() && !getName().isEmpty())
        {
            appGui().getControllerGui().setTileName(uri, getName().toStdString());
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditTile::setName(const QString& str)
{
    ui->lineEdit->setText(str);
}
////////////////////////////////////////////////////////////////////////////////
QString DialogEditTile::getName() const
{
    return ui->lineEdit->text();
}
////////////////////////////////////////////////////////////////////////////////
