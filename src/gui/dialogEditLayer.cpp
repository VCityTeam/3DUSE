// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogEditLayer.hpp"
#include "ui_dialogEditLayer.h"
#include "gui/applicationGui.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogEditLayer::DialogEditLayer(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEditLayer)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogEditLayer::~DialogEditLayer()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditLayer::editLayer(const vcity::URI& uri)
{
    vcity::abstractLayer* abstractlayer = vcity::app().getScene().getLayer(uri);
    if (abstractlayer)
    {
        setName(abstractlayer->getName().c_str());

        if (exec() && !getName().isEmpty())
        {
            appGui().getControllerGui().setLayerName(uri, getName().toStdString());
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditLayer::setName(const QString& str)
{
    ui->lineEdit->setText(str);
}
////////////////////////////////////////////////////////////////////////////////
QString DialogEditLayer::getName() const
{
    return ui->lineEdit->text();
}
////////////////////////////////////////////////////////////////////////////////
