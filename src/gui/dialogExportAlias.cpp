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

void DialogExportAlias::editAlias(vcity::URI uri,std::map<vcity::URI*,std::string>* map)
{
	ui->lineEdit->clear;

	//search if the URI is already in the map
	std::map<vcity::URI,std::string>::iterator it = map->find(&uri);
	if(it!=map->end)
	{
		//get the alias in the ui edit field
		std::string alias = it->second;
		ui->lineEdit->setText(QString::fromStdString(alias));
	}

	// window exec
	int res = exec();
	if (res)
	{
		std::string alias = ui->lineEdit->text().toStdString;
	//		check legitimity ?
	//		update the map
		map[&(uri)]=alias;
	}
}