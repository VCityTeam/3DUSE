// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "moc/dialogLink.hpp"
#include "ui_dialogLink.h"
#include "gui/applicationGui.hpp"
#include "moc/mainWindow.hpp"

#include <QSettings>
#include <QFileDialog>

DialogLink::DialogLink(QWidget *parent):
	QDialog(parent),
    ui(new Ui::DialogLink)
{
	ui->setupUi(this);
	connect(ui->pushButton,SIGNAL(clicked()), this, SLOT(loadNewObjects()));
	filenames.clear();
	initComboBox();
}

DialogLink::~DialogLink()
{
    delete ui;
}

void DialogLink::initComboBox()
{
	QStringList cityObjectTypes;
	cityObjectTypes 
		<< "Building"				//0
		<< "Room"					//1
		<< "BuildingInstallation"	//2
		<< "BuildingFurniture"		//3
		<< "Door"					//4
		<< "Window"					//5
		<< "CityFurniture"			//6
		<< "Track"					//7
		<< "Road"					//8
		<< "Railway"				//9
		<< "Square"					//10
		<< "PlantCover"				//11
		<< "SolitaryVegetationObject"//12
		<< "WaterBody"				//13
		<< "TINRelief"				//14
		<< "LandUse"				//15
		<< "Tunnel"					//16
		<< "Bridge"					//17
		<< "BridgeConstructionElement"//18
		<< "BridgeInstallation"		//19
		<< "BridgePart"				//20
		<< "BuildingPart"			//21
		<< "WallSurface"			//22
		<< "RoofSurface"			//23
		<< "GroundSurface"			//24
		<< "ClosureSurface"			//25
		<< "FloorSurface"			//26
		<< "InteriorWallSurface"	//27
        << "CeilingSurface" ; 		//28
    ui->comboBox->addItems(cityObjectTypes);
}

citygml::CityObject* DialogLink::createXLink(int index)
{
	switch (index) {
	case 0 : return new citygml::Building(""); break;
	case 1 : return new citygml::Room(""); break;
	case 2 : return new citygml::BuildingInstallation(""); break;
	case 3 : return new citygml::BuildingFurniture(""); break;
	case 4 : return new citygml::Door(""); break;
	case 5 : return new citygml::Window(""); break;
	case 6 : return new citygml::CityFurniture(""); break;
	case 7 : return new citygml::Track(""); break;
	case 8 : return new citygml::Road(""); break;
	case 9 : return new citygml::Railway(""); break;
	case 10 : return new citygml::Square(""); break;
	case 11 : return new citygml::PlantCover(""); break;
	case 12 : return new citygml::SolitaryVegetationObject(""); break;
	case 13 : return new citygml::WaterBody(""); break;
	case 14 : return new citygml::TINRelief(""); break;
	case 15 : return new citygml::LandUse(""); break;
	case 16 : return new citygml::Tunnel(""); break;
	case 17 : return new citygml::Bridge(""); break;
	case 18 : return new citygml::BridgeConstructionElement(""); break;
	case 19 : return new citygml::BridgeInstallation(""); break;
	case 20 : return new citygml::BridgePart(""); break;
	case 21 : return new citygml::BuildingPart(""); break;
	case 22 : return new citygml::WallSurface(""); break;
	case 23 : return new citygml::RoofSurface(""); break;
	case 24 : return new citygml::GroundSurface(""); break;
	case 25 : return new citygml::ClosureSurface(""); break;
	case 26 : return new citygml::FloorSurface(""); break;
	case 27 : return new citygml::InteriorWallSurface(""); break;
	case 28 : return new citygml::CeilingSurface(""); break;
	default : return new citygml::GenericCityObject(""); break;
	}
}

void DialogLink::loadNewObjects()
{
	QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    filenames = QFileDialog::getOpenFileNames(this, "Load scene files", lastdir);
}

void DialogLink::addLink(const vcity::URI& uri)
{
	appGui().getMainWindow()->m_osgView->setActive(false);
	uri.resetCursor();
	citygml::CityObject* parent = vcity::app().getScene().getCityObjectNode(uri);

	//execution
	int res = exec();
	
	if(res)
	{
		citygml::CityObject * link = createXLink(ui->comboBox->currentIndex());	
		std::string type = ui->comboBox->currentText().toStdString();

		parent->getChildren().push_back(link);
		link->_isXlink = citygml::xLinkState::UNLINKED;
		link->setAttribute( "xlink", ui->lineEdit->text().toStdString(), false );
		QString item2text;
		QSettings settings("liris", "virtualcity");
		for(const QString& filename : filenames)
        {
            if(!filename.isEmpty())
            {
                citygml::ParserParams params;
                citygml::CityModel* mdl = citygml::load(filename.toStdString(), params);
                citygml::CityObject* obj = mdl->getCityObjectsRoots()[0];
                obj->m_path = filename.toStdString();
					
                item2text = obj->getId().c_str();
                QFileInfo file(filename);
                settings.setValue("lastdir", file.dir().absolutePath());

				link->addXLinkTarget(obj);
				link->_isXlink = citygml::xLinkState::LINKED;
				obj->_isXlink = citygml::xLinkState::TARGET;
				uri.resetCursor();
				citygml::CityModel* model = vcity::app().getScene().getTile(uri)->getCityModel();
				model->addCityObject(obj);
				model->addCityObjectAsRoot(obj);
            }
        }
		uri.resetCursor();
		appGui().getTreeView()->addCityObject(appGui().getTreeView()->getNode(uri),link);
	}
	appGui().getMainWindow()->m_osgView->setActive(true);
}
