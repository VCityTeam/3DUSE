#include "moc/dialogOBJCityGML.hpp"
#include "ui_dialogOBJCityGML.h"
#include "gui/applicationGui.hpp"
#include "moc/mainWindow.hpp"
#include "import/importerAssimp.hpp"
#include "export/exportCityGML.hpp"

#include <QFileDialog>

DialogOBJCityGML::DialogOBJCityGML(QWidget *parent):
	QDialog(parent),
	ui(new Ui::DialogOBJCityGML)
{
	ui->setupUi(this);
	ui->roofBox->setEnabled(false);
}


DialogOBJCityGML::~DialogOBJCityGML(void)
{
}

void DialogOBJCityGML::convertOBJ()
{
	int res = exec();
	if (res)
	{
		QStringList filenames = QFileDialog::getOpenFileNames(this, "Convert OBJ to CityGML","","OBJ files (*.obj)");
		for(int i = 0; i < filenames.count(); ++i)
		{
			QFileInfo file(filenames[i]);
			QString ext = file.suffix().toLower();
			if(ext == "obj")
			{
				citygml::ImporterAssimp importer;
				if(ui->offsetBox->isChecked()) importer.setOffset(vcity::app().getSettings().getDataProfile().m_offset.x, vcity::app().getSettings().getDataProfile().m_offset.y);
				else importer.setOffset(0, 0);
				std::cout<<"Loading...";
				citygml::CityModel* model = importer.import(file.absoluteFilePath().toStdString());
				std::cout<<" Done!"<<std::endl;
				std::cout<<"Export...";
				citygml::ExporterCityGML exporter((file.path()+'/'+file.baseName()+".gml").toStdString());
				exporter.exportCityModel(*model);
				std::cout<<" Done!"<<std::endl;
			}
		}
	}
}