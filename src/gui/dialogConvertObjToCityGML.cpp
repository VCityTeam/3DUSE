////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogConvertObjToCityGML.hpp"
#include "ui_dialogConvertObjToCityGML.h"
#include "gui/applicationGui.hpp"
#include "importerAssimp.hpp"
#include "libcitygml/export/exportCityGML.hpp"
#include <QFileDialog>
////////////////////////////////////////////////////////////////////////////////
DialogConvertObjToCityGML::DialogConvertObjToCityGML(double OffsetX, double OffsetY, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogConvertObjToCityGML)
{
    ui->setupUi(this);

    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(chooseCityGMLPathSlot()));
    connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(chooseOutputPathSlot()));
    connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(ConvertObjToCityGMLSlot()));

    offsetX = OffsetX;
    offsetY = OffsetY;
}
////////////////////////////////////////////////////////////////////////////////
DialogConvertObjToCityGML::~DialogConvertObjToCityGML()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogConvertObjToCityGML::chooseCityGMLPathSlot()
{
    OBJfilenames = QFileDialog::getOpenFileNames(this, "Convert OBJ to CityGML");
}
////////////////////////////////////////////////////////////////////////////////
void DialogConvertObjToCityGML::chooseOutputPathSlot()
{
    QString path = QFileDialog::getExistingDirectory(this, "Choose Output path.");
    ui->lineEditDataPath_2->setText(path);
}
////////////////////////////////////////////////////////////////////////////////
void DialogConvertObjToCityGML::ConvertObjToCityGMLSlot()
{
    for (int i = 0; i < OBJfilenames.count(); ++i)
    {
        QFileInfo file(OBJfilenames[i]);
        QString ext = file.suffix().toLower();
        if (ext == "obj")
        {
            citygml::ImporterAssimp importer;
            importer.setOffset(offsetX, offsetY);

            citygml::CityModel* model = importer.import(file.absoluteFilePath().toStdString(), true, ui->Prefix->text());

            citygml::ExporterCityGML exporter((ui->lineEditDataPath_2->text() + '/' + file.baseName() + ".gml").toStdString());
            exporter.exportCityModel(*model);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
