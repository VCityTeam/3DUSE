#include "moc/DialogExportAlias.hpp"
#include "ui_DialogExportAlias.h"
#include "gui/applicationGui.hpp"

DialogExportAlias::DialogExportAlias(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogExportAlias)
{
    ui->setupUi(this);
}

DialogExportAlias::~DialogExportAlias()
{
    delete ui;
}