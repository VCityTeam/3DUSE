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
    vcity::Layer* layer = vcity::app().getScene().getLayer(uri);
    if(layer)
    {
        setName(layer->getName().c_str());

        if(exec() && !getName().isEmpty())
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
