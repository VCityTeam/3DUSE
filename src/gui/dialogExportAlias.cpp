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

void DialogExportAlias::editAlias(vcity::URI uri,std::map<std::string,std::string>* map)
{
	ui->lineEdit->clear();

	//search if the URI is already in the map
	//std::cout<<"Searching object in map"<<std::endl;
	std::map<std::string,std::string>::iterator it = map->find(uri.getStringURI());
	if(it != map->end())
	{
		//get the alias in the ui edit field
		std::string alias = it->second;
		//std::cout<<"- object found, current alias is \""<<alias<<"\""<<std::endl;
		ui->lineEdit->setText(QString::fromStdString(alias));
	}
	else
	{
		//std::cout<<"- object not found"<<std::endl;
	}
	// window exec
	int res = exec();
	if (res)
	{
		std::string alias = ui->lineEdit->text().toStdString();
		if (alias.empty())
		{
			if(it != map->end()) map->erase(it);
		}
		else
		{
			(*map)[uri.getStringURI()] = alias;
		}
	}
}