// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "CityGMLFusion.h"

#include "export/exportCityGML.hpp"
#include <QFileDialog>
#include <QSettings>
#include <QDirIterator>
#include <sstream>
#include <iomanip>
#include "tile.hpp"

void CallFusion()
{
    //// FusionLODs : prend deux fichiers modelisant les batiments avec deux lods differents et les fusionne en un seul
    QSettings settings("liris", "virtualcity");
    QString lastdir = settings.value("lastdir").toString();
    QStringList File1 = QFileDialog::getOpenFileNames(0, "Selectionner le premier fichier.", lastdir); //Use "this" instead of "0" in mainWindow.cpp

    QFileInfo file1temp(File1[0]);
    QString file1path = file1temp.absoluteFilePath();
    QFileInfo file1(file1path);

    QString ext = file1.suffix().toLower();
    if (ext != "citygml" && ext != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML" << std::endl;
        return;
    }

    QStringList File2 = QFileDialog::getOpenFileNames(0, "Selectionner le second fichier.", lastdir);

    QFileInfo file2temp(File2[0]);
    QString file2path = file2temp.absoluteFilePath();
    QFileInfo file2(file2path);

    ext = file2.suffix().toLower();
    if (ext != "citygml" && ext != "gml")
    {
        std::cout << "Erreur : Le fichier n'est pas un CityGML" << std::endl;
        return;
    }

    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier de sortie");
    w.setFileMode(QFileDialog::Directory);

    if (w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }
    std::string Folder = w.selectedFiles().at(0).toStdString() + "/" + file2.baseName().toStdString() + "_Fusion.gml";

    std::cout << "load citygml file : " << file1path.toStdString() << std::endl;
    vcity::Tile* tile1 = new vcity::Tile(file1path.toStdString());
    std::cout << "load citygml file : " << file2path.toStdString() << std::endl;
    vcity::Tile* tile2 = new vcity::Tile(file2path.toStdString());

    citygml::CityModel * City1 = tile1->getCityModel();
    citygml::CityModel * City2 = tile2->getCityModel();

    FusionLODs(City1, City2);

    citygml::ExporterCityGML exporter(Folder);

    exporter.exportCityModel(*City2);
}

void FusionLODs(citygml::CityModel * City1, citygml::CityModel * City2) //// Prend deux fichiers modelisant les batiments avec deux lods differents et les fusionne en un seul
{
    citygml::CityObjects City1Obj = City1->getCityObjectsRoots();
    citygml::CityObjects City2Obj = City2->getCityObjectsRoots();

    for (citygml::CityObject * obj1 : City1Obj)
    {
        if (obj1)
        {
            bool test = false; //Passera a true si obj1 trouve son equivalent dans City2
            for (citygml::CityObject * obj2 : City2Obj)
            {
                if (obj2)
                {
                    if (obj1->getId() == obj2->getId()) //Fusion des geometries de obj1 et obj2 dans obj2
                    {
                        const std::vector<citygml::CityObject *> Surfaces = obj1->getChildren();
                        for (citygml::CityObject * Surface : Surfaces)
                        {
                            obj2->insertNode(Surface);
                        }

                        test = true;
                        break;
                    }
                }
            }
            if (!test) //Ajout de l'obj1 dans City2
            {
                City2->addCityObject(obj1);
            }
        }
    }
}
