// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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

    if(obj && exec())
    {

    }

    appGui().getMainWindow()->m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
