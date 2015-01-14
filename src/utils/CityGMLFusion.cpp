#include "CityGMLFusion.h"

#include "export/exportCityGML.hpp"
#include <QFileDialog>
#include <QSettings>
#include <QDirIterator>
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <sstream>
#include <iomanip>


void MergeTiles(std::string Path, citygml::CityModel* GML1, citygml::CityModel* GML2) //A SUPPRIMER ?
{
    citygml::ExporterCityGML exporter(Path);

    std::vector<const citygml::CityObject*> Objs;

    if(!GML1 || !GML2)
        return;

    for(citygml::CityObject * obj : GML1->getCityObjectsRoots())
    {
        if(obj)
            Objs.push_back(obj);
    }
    for(citygml::CityObject * obj : GML2->getCityObjectsRoots())
    {
        if(obj)
            Objs.push_back(obj);
    }
    exporter.exportCityObject(Objs);
}
void MergeTiles(std::string Path, xmlNodePtr GML1, xmlNodePtr GML2)
{
    xmlDocPtr out_doc = NULL;			// output document pointer
    xmlNodePtr out_root_node = NULL;	// output root node pointer

    // creates a new document
    out_doc = xmlNewDoc(BAD_CAST "1.0");

    xmlNodePtr n1 = GML1;
    xmlNodePtr n2 = GML2;
    if (xmlStrEqual(n1->name, BAD_CAST "CityModel"))
    {
        if (xmlStrEqual(n2->name, BAD_CAST "CityModel")) //Pour faire une fusion des Root des deux CityGML
        {
            xmlNodePtr C1 = n1->properties->children;
            xmlNodePtr C2 = n2->properties->children;

            xmlNodePtr copy_node1 = xmlCopyNode(n1, 2);
            copy_node1->properties->children = xmlTextMerge(C1, C2);
            xmlDocSetRootElement(out_doc, copy_node1);
            out_root_node = xmlDocGetRootElement(out_doc);
        }
        else
        {
            xmlFreeDoc(out_doc);
            return;
        }

        double Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;

        for (n1 = n1->children; n1 != NULL; n1 = n1->next)
        {
            if (xmlStrEqual(n1->name, BAD_CAST "cityObjectMember") || xmlStrEqual(n1->name, BAD_CAST "appearanceMember"))
            {
                xmlNodePtr copy_node2 = xmlCopyNode(n1, 1);
                xmlAddChild(out_root_node, copy_node2);
            }
            else if (xmlStrEqual(n1->name, BAD_CAST "boundedBy"))
            {
                xmlNodePtr n3;
                for (n3 = n1->children->children; n3 != NULL; n3 = n3->next)
                {
                    ////////////// ATTENTION : LES DONNES DE PARIS NUTILISENT PAS DE CORNER, MAIS DEUX GML:POS ///////////////// A PRENDRE EN COMPTE SI NECESSAIRE
                    if (xmlStrEqual(n3->name, BAD_CAST "lowerCorner"))
                    {
                        //std::cout << "Lower Corner N1 = " << n3->children->content << std::endl;
                        std::string str((const char*)(n3->children->content));

                        std::string X = str.substr(0, str.find_first_of(" "));
                        std::string Y = str.substr(str.find_first_of(" ") + 1, str.find_last_of(" ") - str.find_first_of(" "));
                        std::string Z = str.substr(str.find_last_of(" ") + 1);

                        Xmin = std::stod(X);
                        Ymin = std::stod(Y);
                        Zmin = std::stod(Z);
                    }
                    else if (xmlStrEqual(n3->name, BAD_CAST "upperCorner"))
                    {
                        //std::cout << "Upper Corner N1 = " << n3->children->content << std::endl;
                        std::string str((const char*)(n3->children->content));

                        std::string X = str.substr(0, str.find_first_of(" "));
                        std::string Y = str.substr(str.find_first_of(" ") + 1, str.find_last_of(" ") - str.find_first_of(" "));
                        std::string Z = str.substr(str.find_last_of(" ") + 1);

                        Xmax = std::stod(X);
                        Ymax = std::stod(Y);
                        Zmax = std::stod(Z);
                    }
                }
            }
        }
        //////////////////////////////
        for (n2 = n2->children; n2 != NULL; n2 = n2->next)
        {
            if (xmlStrEqual(n2->name, BAD_CAST "cityObjectMember") || xmlStrEqual(n2->name, BAD_CAST "appearanceMember"))
            {
                xmlNodePtr copy_node2 = xmlCopyNode(n2, 1);
                xmlAddChild(out_root_node, copy_node2);
            }
            else if (xmlStrEqual(n2->name, BAD_CAST "boundedBy"))
            {
                xmlNodePtr copy_node2 = xmlCopyNode(n2, 1);// ? Je ne comprends pas comment ça fonctionne, comment copy_node2 se met à jour avec les nouveaux corners ?
                xmlNodePtr n3;
                for (n3 = n2->children->children; n3 != NULL; n3 = n3->next)
                {
                    if (xmlStrEqual(n3->name, BAD_CAST "lowerCorner"))
                    {
                        std::cout << "Lower Corner N2 = " << n3->children->content << std::endl;
                        std::string str((const char*)(n3->children->content));

                        std::string X = str.substr(0, str.find_first_of(" "));
                        std::string Y = str.substr(str.find_first_of(" ") + 1, str.find_last_of(" ") - str.find_first_of(" "));
                        std::string Z = str.substr(str.find_last_of(" ") + 1);

                        double x = std::stod(X);
                        Xmin = std::min(Xmin, x);
                        double y = std::stod(Y);
                        Ymin = std::min(Ymin, y);
                        double z = std::stod(Z);
                        Zmin = std::min(Zmin, z);

                        std::stringstream ss;
                        ss << std::setprecision(10) << Xmin << " " << Ymin << " " << Zmin;

                        n3->children->content = BAD_CAST ss.str().c_str();
                        std::cout << "Lower Corner N2 = " << n3->children->content << std::endl;
                    }
                    else if (xmlStrEqual(n3->name, BAD_CAST "upperCorner"))
                    {
                        std::cout << "Upper Corner N2 = " << n3->children->content << std::endl;
                        std::string str((const char*)(n3->children->content));

                        std::string X = str.substr(0, str.find_first_of(" "));
                        std::string Y = str.substr(str.find_first_of(" ") + 1, str.find_last_of(" ") - str.find_first_of(" "));
                        std::string Z = str.substr(str.find_last_of(" ") + 1);

                        double x = std::stod(X);
                        Xmax = std::max(Xmax, x);
                        double y = std::stod(Y);
                        Ymax = std::max(Ymax, y);
                        double z = std::stod(Z);
                        Zmax = std::max(Zmax, z);

                        std::stringstream ss;
                        ss << std::setprecision(10) << Xmax << " " << Ymax << " " << Zmax; //Pour ne pas perdre des chiffres après la virgule

                        n3->children->content = BAD_CAST ss.str().c_str();
                        std::cout << "Lower Corner N2 = " << n3->children->content << std::endl;
                    }
                }
                xmlAddChild(out_root_node, copy_node2);
            }
        }
    }
    else
    {
        xmlFreeDoc(out_doc);
        return;
    }

    // dumping document to file
    xmlSaveFormatFileEnc(Path.c_str(), out_doc, "ISO-8859-1", 1);

    // free the documents
    xmlFreeDoc(out_doc);
}
/**
* @brief Fusion des fichiers CityGML contenus dans deux dossiers : sert à fusionner les tiles donc deux fichiers du même nom seront fusionnés en un fichier contenant tous leurs objets à la suite.
*/
void FusionTiles()
{
    ///// Ouverture des fichiers CityGML de la première zone
    QFileDialog w;
    w.setWindowTitle("Selectionner le dossier contenant les CityGML de la premiere zone.");
    w.setFileMode(QFileDialog::Directory);

    if(w.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::vector<std::string> ListGML1;

    QStringList fileGML1 = w.selectedFiles();
    foreach (QString s, fileGML1)
    {
        QFileInfo file(s);
        if(file.isDir())
        {
            QDir dir(s);
            QStringList files;

            QDirIterator iterator(dir.absolutePath(), QDirIterator::Subdirectories);
            while(iterator.hasNext())
            {
                iterator.next();
                if(!iterator.fileInfo().isDir())
                {
                    QString filename = iterator.filePath();
                    if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
                        files.append(filename);
                }
            }

            for(int i = 0; i < files.count(); ++i)
            {
                ListGML1.push_back(files[i].toStdString());
            }
        }
        else if(file.isFile())
        {
            ListGML1.push_back(s.toStdString());
        }
    }

    ///// Ouverture des fichiers CityGML de la seconde zone

    QFileDialog w1;
    w1.setWindowTitle("Selectionner le dossier contenant les CityGML de la seconde zone.");
    w1.setFileMode(QFileDialog::Directory);

    if(w1.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::vector<std::string> ListGML2;

    QStringList fileGML2 = w1.selectedFiles();
    foreach (QString s, fileGML2)
    {
        QFileInfo file(s);
        if(file.isDir())
        {
            QDir dir(s);
            QStringList files;

            QDirIterator iterator(dir.absolutePath(), QDirIterator::Subdirectories);
            while(iterator.hasNext())
            {
                iterator.next();
                if(!iterator.fileInfo().isDir())
                {
                    QString filename = iterator.filePath();
                    if(filename.endsWith(".citygml", Qt::CaseInsensitive) || filename.endsWith(".gml", Qt::CaseInsensitive))
                        files.append(filename);
                }
            }

            for(int i = 0; i < files.count(); ++i)
            {
                ListGML2.push_back(files[i].toStdString());
            }
        }
        else if(file.isFile())
        {
            ListGML2.push_back(s.toStdString());
        }
    }

    ///// Sélection du dossier de sortie

    QFileDialog w2;
    w2.setWindowTitle("Selectionner le dossier de sortie pour la fusion de CityGML.");
    w2.setFileMode(QFileDialog::Directory);

    if(w2.exec() == 0)
    {
        std::cout << "Annulation : Dossier non valide." << std::endl;
        return;
    }

    std::string FolderOut = w2.selectedFiles().at(0).toStdString();

    ///// Détection des fichiers CityGML à lier et copie dans le dossier de sortie

    foreach(std::string GML1, ListGML1)//On parcourt tous les fichiers CityGML de la première zone
    {
        std::string Tile1 = GML1.substr(GML1.find_last_of("/") + 1);
        Tile1 = Tile1.substr(Tile1.find_first_of("_") + 1, Tile1.find(".") - (Tile1.find_first_of("_") + 1));

        int i = 0;
        bool Erase = false;

        foreach(std::string GML2, ListGML2)//On parcourt tous les fichiers CityGML de la seconde zone
        {
            std::string Tile2 = GML2.substr(GML2.find_last_of("/") + 1);
            Tile2 = Tile2.substr(Tile2.find_first_of("_") + 1, Tile2.find(".") - (Tile2.find_first_of("_") + 1));

            if(Tile1 == Tile2)//Si deux fichiers ont le même nom, cela signifie qu'ils sont sur la même tuile et qu'il faut les fusionner
            {
                xmlDocPtr doc1;

                // opens document
                xmlKeepBlanksDefault(0); // ignore les noeuds texte composant la mise en forme
                doc1 = xmlParseFile(GML1.c_str());
                if (doc1 == NULL)
                {
                    fprintf(stderr, "Invalid XML file\n");
                    continue;
                }

                xmlNodePtr racine1 = xmlDocGetRootElement(doc1);

                if (racine1 == NULL)
                {
                    fprintf(stderr, "Empty XML file\n");
                    xmlFreeDoc(doc1);
                    continue;
                }

                xmlDocPtr doc2;
                doc2 = xmlParseFile(GML2.c_str());
                if (doc2 == NULL)
                {
                    fprintf(stderr, "Invalid XML file\n");
                    continue;
                }

                xmlNodePtr racine2 = xmlDocGetRootElement(doc2);

                if (racine2 == NULL)
                {
                    fprintf(stderr, "Empty XML file\n");
                    xmlFreeDoc(doc2);
                    continue;
                }

                std::cout << "Fusion : " << Tile1 << std::endl;

                MergeTiles(FolderOut + "/Tile_" + Tile1 + ".gml", racine1, racine2);

                Erase = true;
                //xmlFreeDoc(doc1);
                //xmlFreeDoc(doc2);
                break;
            }

            ++i;
        }
        if(Erase)
            ListGML2.erase(ListGML2.begin()+i); //On supprime le CityGML de la seconde zone qui a été fusionné pour ne conserver que les CityGML propres à la seconde zone
        else //Le fichier CityGML courant de la première zone ne se retrouve pas dans la seconde zone donc il faut le dupliquer tel quel
        {
            std::cout << "Copie de " << Tile1 << std::endl;

            QFile file(GML1.c_str());
            file.copy(QString::fromStdString(FolderOut + "/Tile_" + Tile1 + ".gml"));
            /*std::string x = Tile1.substr(0, Tile1.find("_"));
            std::string y = Tile1.substr(Tile1.find("_") + 1);
            int X = std::stoi(x) + 3689;
            int Y = std::stoi(y) + 10347;
            file.copy(QString::fromStdString(FolderOut + "/Dossier1/Tile_" + std::to_string(X) + "_" + std::to_string(Y) + ".gml"));*/
            file.close();
        }
    }

    foreach(std::string GML2, ListGML2) //On duplique les fichiers CityGML de la seconde zone qui n'ont pas été retrouvé dans la première
    {
        std::string Tile2 = GML2.substr(GML2.find_last_of("/")+1);
        Tile2 = Tile2.substr(Tile2.find_first_of("_") + 1, Tile2.find(".") - (Tile2.find_first_of("_") + 1));

        std::cout << "Copie de " << Tile2 << std::endl;

        QFile file(GML2.c_str());
        file.copy(QString::fromStdString(FolderOut + "/Tile_" + Tile2 + ".gml"));
        file.close();
    }
}

void FusionLODs(citygml::CityModel * City1, citygml::CityModel * City2) //// Prend deux fichiers modélisant les bâtiments avec deux lods différents et les fusionne en un seul
{
	citygml::CityObjects City1Obj = City1->getCityObjectsRoots();
	citygml::CityObjects City2Obj = City2->getCityObjectsRoots();

	for(citygml::CityObject * obj1 : City1Obj)
	{
		if(obj1)
		{
			bool test = false; //Passera à true si obj1 trouve son équivalent dans City2
			for(citygml::CityObject * obj2 : City2Obj)
			{
				if(obj2)
				{
					if(obj1->getId() == obj2->getId()) //Fusion des geometries de obj1 et obj2 dans obj2
					{
						const std::vector<citygml::CityObject *> Surfaces = obj1->getChildren();
						for(citygml::CityObject * Surface : Surfaces)
						{
							obj2->insertNode(Surface);
						}

						test = true;
						break;
					}
				}
			}
			if(!test) //Ajout de l'obj1 dans City2
			{
				City2->addCityObject(obj1);
			}
		}
	}
}
