#include "FloodARTools.hpp"

#include "osgDB/fstream"

#include <QFileDialog>
#include <QImageReader>
#include <QDebug>
#include <QMessageBox>

#include "gui/osg/osgMnt.hpp"
#include "import/importerASC.hpp"
#include "processes/ASCCut.hpp"

namespace FloodAR
{
	////////////////////////////////////////////////////////////////////////////////
	std::vector<TextureCityGML*> getTexturesList(citygml::CityModel* model, QFileInfo file, QFileInfo texturesPath)
	{
		std::vector<TextureCityGML*> TexturesList;

		for (citygml::CityObject* obj : model->getCityObjectsRoots())
		{
			for (citygml::Geometry* Geometry : obj->getGeometries())
			{
				for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
				{

					std::vector<TVec2f> TexUV;

					OGRLinearRing * OgrRing = new OGRLinearRing;
					for (TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
					{
						OgrRing->addPoint(Point.x, Point.y, Point.z);
						TexUV.push_back(TVec2f(Point.x, Point.y));
					}

					{
						//double A, B, C, D; //Voir fr.wikipedia.org/wiki/World_file : Taille pixel, rotation, retournement //Pour faire une conversion propre.
						double offset_x;
						double offset_y;

						//std::ifstream fichier(texturesPath.absolutePath().toStdString() + "/" + texturesPath.baseName().toStdString() + ".jgw", std::ios::in);

						//if (fichier)
						//{
						//	fichier >> A >> B >> C >> D >> offset_x >> offset_y;
						//	fichier.close();
						//}
						offset_x = model->getEnvelope().getLowerBound().x;
						offset_y = model->getEnvelope().getUpperBound().y;
						float tileSizeX = model->getEnvelope().getUpperBound().x - model->getEnvelope().getLowerBound().x; // taille de la zone en mètres
						float tileSizeY = model->getEnvelope().getUpperBound().y - model->getEnvelope().getLowerBound().y;
						int i = 0;
						for (TVec2f UV : TexUV)
						{
							UV.x = (UV.x - offset_x) / tileSizeX;
							UV.y = 1 + (UV.y - offset_y) / tileSizeY;//Car D est négatif
							TexUV.at(i) = UV;
							++i;
						}
					}
					//Remplissage de ListTextures
					QDir workdir = file.dir();
					std::string Url = workdir.relativeFilePath(texturesPath.filePath()).toStdString();
					citygml::Texture::WrapMode WrapMode = citygml::Texture::WM_NONE;

					TexturePolygonCityGML Poly;
					Poly.Id = PolygonCityGML->getId();
					Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
					Poly.TexUV = TexUV;

					bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
					for (TextureCityGML* Tex : TexturesList)
					{
						if (Tex->Url == Url)
						{
							URLTest = true;
							Tex->ListPolygons.push_back(Poly);
							break;
						}
					}
					if (!URLTest)
					{
						TextureCityGML* Texture = new TextureCityGML;
						Texture->Wrap = WrapMode;
						Texture->Url = Url;
						Texture->ListPolygons.push_back(Poly);
						TexturesList.push_back(Texture);
					}
				}
			}
		}
		return TexturesList;
	}
	////////////////////////////////////////////////////////////////////////////////
	void cutASC(std::string filePath, std::string outputDir, int tileSizeX, int tileSizeY)
	{
		QFileInfo file = QFileInfo(QString(filePath.c_str()));
		//reading file
		citygml::ImporterASC* importer = new citygml::ImporterASC();
		MNT* asc = new MNT();
		if (asc->charge(filePath.c_str(), "ASC"))
		{
			ASCCut(asc, tileSizeX, tileSizeY, outputDir, file.baseName().toStdString());
		}
		delete importer;
		delete asc;
	}
	////////////////////////////////////////////////////////////////////////////////
	void ASCtoWater(std::string filePath, bool polygonsImport, float prec, bool tempImport, std::string creaDate, std::string termDate)
	{
		QFileInfo file = QFileInfo(QString(filePath.c_str()));
		citygml::CityModel* model;
		std::cout << "CONVERTING FILE " << file.baseName().toStdString() << std::endl;
		QString ext = file.suffix().toLower();
		if (ext == "asc")
		{
			//lecture du fichier
			citygml::ImporterASC* importer = new citygml::ImporterASC();
			MNT* asc = new MNT();
			if (asc->charge(file.absoluteFilePath().toStdString().c_str(), "ASC"))
			{
				//conversion en structure CityGML
				if (polygonsImport)
				{
					model = new citygml::CityModel();
					citygml::CityObject* waterbody = importer->waterToCityGMLPolygons(asc, prec);
					model->addCityObject(waterbody);
					model->addCityObjectAsRoot(waterbody);
				}
				else
				{
					model = importer->waterToCityGML(asc);
				}
				delete importer;
				delete asc;
			}
		}
		//Add temporal info
		if (tempImport)
		{
			for (citygml::CityObject* obj : model->getCityObjectsRoots())
			{
				obj->setAttribute("creationDate", creaDate);
				obj->setAttribute("terminationDate", termDate);
			}
		}
		//export en CityGML
		std::cout << "Export ...";
		if (model->size() != 0)
		{
			citygml::ExporterCityGML exporter((file.path() + '/' + file.baseName() + ".gml").toStdString());
			exporter.exportCityModel(*model);
			std::cout << "OK!" << std::endl;
		}
		else std::cout << std::endl << "Export aborted: empty CityModel!" << std::endl;
		delete model;
	}
	////////////////////////////////////////////////////////////////////////////////
	void ASCtoTerrain(std::string filePath1, bool fusion, std::string filePath2, bool addTextures, std::string texturesPath)
	{
		citygml::CityModel* model;
		QFileInfo file = QFileInfo(QString(filePath1.c_str()));
		if (!fusion)
		{
			std::cout << "CONVERTING FILE " << file.baseName().toStdString() << std::endl;
			QString ext = file.suffix().toLower();
			if (ext == "asc")
			{
				//lecture du fichier
				citygml::ImporterASC* importer = new citygml::ImporterASC();
				MNT* asc = new MNT();
				if (asc->charge(file.absoluteFilePath().toStdString().c_str(), "ASC"))
				{
					//conversion en structure CityGML
					model = importer->reliefToCityGML(asc);
					delete importer;
					delete asc;
				}
			}
		}
		else
		{
			QFileInfo file2 = QFileInfo(QString(filePath2.c_str()));
			std::cout << "MERGING FILES " << file.baseName().toStdString() << " AND " << file2.baseName().toStdString() << std::endl;
			QString ext = file.suffix().toLower();
			if (ext == "asc")
			{
				//lecture du fichier
				citygml::ImporterASC* importer = new citygml::ImporterASC();
				MNT* asc1 = new MNT();
				MNT* asc2 = new MNT();
				if (asc1->charge(file.absoluteFilePath().toStdString().c_str(), "ASC") && (asc2->charge(file2.absoluteFilePath().toStdString().c_str(), "ASC")))
				{
					//Check which MNT is the more precise one
					MNT* morePrecise;
					MNT* lessPrecise;
					if (asc1->get_pas_x() >= asc2->get_pas_x())
					{
						lessPrecise = asc1;
						morePrecise = asc2;
					}
					else
					{
						lessPrecise = asc2;
						morePrecise = asc1;
					}
					//conversion en structure CityGML
					model = importer->fusionResolutions(lessPrecise, morePrecise);
					delete importer;
					delete asc1;
					delete asc2;
				}
			}
		}
		//add textures
		std::vector<TextureCityGML*> TexturesList;
		if (addTextures)
		{
			TexturesList = getTexturesList(model, file, QFileInfo(QString(texturesPath.c_str())));
		}
		//export en CityGML
		std::cout << "Export ...";
		if (model->size() != 0)
		{
			citygml::ExporterCityGML exporter((file.path() + '/' + file.baseName() + ".gml").toStdString());
			if (addTextures) exporter.exportCityModelWithListTextures(*model, &TexturesList);
			else exporter.exportCityModel(*model);
			std::cout << "OK!" << std::endl;
		}
		else std::cout << std::endl << "Export aborted: empty CityModel!" << std::endl;
		delete model;
		for (TextureCityGML* tex : TexturesList) delete tex;
	}
	////////////////////////////////////////////////////////////////////////////////
	void cutPicture(std::string filename, int tileSizeX, int tileSizeY)
	{

		float NW_x, NW_y;
		float pxSize_x, pxSize_y;
		float rx, ry;

		QFileInfo file(filename.c_str());

		//TODO : read info from world file or display a prompt?$

		std::string ext = file.suffix().toStdString();
		std::stringstream wext;
		wext << ext.at(0) << ext.at(ext.size() - 1) << "w";
		std::string wPath = file.absolutePath().toStdString() + "/" + file.baseName().toStdString() + ".";
		wPath = wPath + wext.str();
		std::cout << wPath << std::endl;
		QFileInfo wFile(wPath.c_str());
		if (wFile.exists())
		{
			std::ifstream fichier(wPath, std::ios::in);
			fichier >> pxSize_x >> ry >> rx >> pxSize_y >> NW_x >> NW_y;
			fichier.close();
			pxSize_y = abs(pxSize_y);
		}
		else
		{
			std::cerr << "world file not found!" << std::endl;
			QMessageBox msgBox;
			msgBox.setText("World file not found, tiling aborted!");
			msgBox.setIcon(QMessageBox::Critical);
			msgBox.exec();
			return;
		}

		QImageReader reader(file.absoluteFilePath());
		qDebug() << "Image size:" << reader.size();
		int origWidth = reader.size().rwidth();
		int origHeight = reader.size().rheight();

		QDir dir = file.absoluteDir();
		if (!dir.exists("_MNT"))
			dir.mkdir("_MNT");
		QDir MNTdir = dir.path() + "/_MNT";
		int x, y;
		x = 0;
		y = 0;

		while (x < origWidth && y < origHeight)
		{
			//get bounds of the current tile
			float cornerX = NW_x + (x*pxSize_x);
			float cornerY = NW_y - (y*pxSize_y);
			int dvX = cornerX / tileSizeX;
			int dvY = cornerY / tileSizeY;
			float tileXmin = dvX*tileSizeX;
			float tileXmax = (dvX + 1)*tileSizeY;
			float tileYmin = dvY*tileSizeX;
			float tileYmax = (dvY + 1)*tileSizeY;

			int width = ceil((tileXmax - NW_x) / pxSize_x) - x + 1;
			int height = ceil((NW_y - tileYmin) / pxSize_y) - y + 1;
			//crop and save pitcure
			std::string tilenumber = std::to_string(dvX) + "_" + std::to_string(dvY);

			if (!MNTdir.exists(tilenumber.c_str()))
				MNTdir.mkdir(QString(tilenumber.c_str()));
			QDir tileDir = MNTdir.path() + "/" + tilenumber.c_str(); 
			if (!tileDir.exists("Appearance"))
				tileDir.mkdir("Appearance");

			QFileInfo file(filename.c_str());
			std::string outputname = file.absolutePath().toStdString() + "/_MNT/" + tilenumber + "/Appearance/" + tilenumber + "_MNT.jpg";
			QImageReader reader(file.absoluteFilePath());
			reader.setClipRect(QRect(x, y, width, height));
			QImage croppedImage = reader.read();
			croppedImage.save(QString(outputname.c_str()), "JPG", -1);
			//progress
			std::cout << "Tiling texture... (" << y * 100 / origHeight << "%)\r";
			//tile is finished, set xy for next tile
			if ((x + width) < origWidth)
			{
				x = x + width; //same row, next column
			}
			else
			{
				x = 0; //next row, first column
				y = y + height;
			}
		}
		std::cout << "Tiling texture... (100%)" << std::endl;
	}
	////////////////////////////////////////////////////////////////////////////////
	/*
	 *Method for tiling shapefiles, stored here for now
	 */
	 //void slotCutShapeFile()
	 //{
	 //	const uint tilesize = 500;
	 //	m_osgView->setActive(false);
	 //	QStringList filenames = QFileDialog::getOpenFileNames(this, "Cut Shapefile", "", "Shapefiles (*.shp)");

	 //	QMessageBox msgBox;
	 //	msgBox.setText("Use centroid and don't cut shape features?");
	 //	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
	 //	int ret = msgBox.exec();
	 //	if (ret == QMessageBox::Cancel) { m_osgView->setActive(true); return; }


	 //	for (int i = 0; i < filenames.count(); ++i)
	 //	{
	 //		QFileInfo file(filenames[i]);
	 //		QDir dir = file.absoluteDir();
	 //		if (!dir.exists("tiles"))
	 //			dir.mkdir("tiles");

	 //		const char * DriverName = "ESRI Shapefile";
	 //		OGRSFDriver * Driver;

	 //		OGRRegisterAll();
	 //		Driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(DriverName);
	 //		if (Driver == NULL)
	 //		{
	 //			printf("%s driver not available.\n", DriverName);
	 //			return;
	 //		}
	 //		OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filenames[i].toStdString().c_str(), FALSE);

	 //		double Xmin = 100000000, Xmax = -100000000, Ymin = 100000000, Ymax = -100000000;

	 //		int nbLayers = poDS->GetLayerCount();
	 //		if (nbLayers > 0)
	 //		{
	 //			OGRLayer *poLayer = poDS->GetLayer(0);

	 //			OGRFeature *poFeature;
	 //			poLayer->ResetReading();

	 //			while ((poFeature = poLayer->GetNextFeature()) != NULL)
	 //			{
	 //				OGRGeometry* poGeometry = poFeature->GetGeometryRef();

	 //				if (poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
	 //				{
	 //					OGRPolygon* poPG = (OGRPolygon*)poGeometry;

	 //					OGRLinearRing* poLR = poPG->getExteriorRing();

	 //					int nbPoints = poLR->getNumPoints();

	 //					for (int i = 0; i<nbPoints; ++i)//Pour récupérer les points de l'exterior ring
	 //					{
	 //						OGRPoint p;
	 //						poLR->getPoint(i, &p);

	 //						if (p.getX() > Xmax)
	 //							Xmax = p.getX();
	 //						if (p.getX() < Xmin)
	 //							Xmin = p.getX();
	 //						if (p.getY() > Ymax)
	 //							Ymax = p.getY();
	 //						if (p.getY() < Ymin)
	 //							Ymin = p.getY();
	 //					}
	 //				}
	 //			}

	 //			Xmin = tilesize * ((int)Xmin / tilesize);
	 //			Ymin = tilesize * ((int)Ymin / tilesize);
	 //			Xmax = tilesize * ((int)Xmax / tilesize);
	 //			Ymax = tilesize * ((int)Ymax / tilesize);

	 //			std::cout << std::setprecision(10) << "Boite englobante creee : " << Xmin << " " << Xmax << " | " << Ymin << " " << Ymax << std::endl;

	 //			std::vector<OGRPolygon*> Tuiles;

	 //			for (int x = (int)Xmin; x <= (int)Xmax; x += tilesize)
	 //			{
	 //				for (int y = (int)Ymin; y <= (int)Ymax; y += tilesize)
	 //				{
	 //					OGRLinearRing* Ring = new OGRLinearRing;
	 //					Ring->addPoint(x, y);
	 //					Ring->addPoint(x + tilesize, y);
	 //					Ring->addPoint(x + tilesize, y + tilesize);
	 //					Ring->addPoint(x, y + tilesize);
	 //					Ring->addPoint(x, y);

	 //					OGRPolygon* Poly = new OGRPolygon;
	 //					Poly->addRingDirectly(Ring);

	 //					Tuiles.push_back(Poly);
	 //				}
	 //			}
	 //			std::cout << Tuiles.size() << " tuiles crees" << std::endl;

	 //			int cpt = -1;
	 //			for (int x = (int)Xmin; x <= (int)Xmax; x += tilesize)
	 //			{
	 //				for (int y = (int)Ymin; y <= (int)Ymax; y += tilesize)
	 //				{
	 //					++cpt;

	 //					OGRPolygon* Tuile = Tuiles.at(cpt);
	 //					QFileInfo file(filenames[i]);
	 //					//file.absoluteDir().mkdir(file.baseName());
	 //					std::string name = file.absoluteDir().absolutePath().toStdString() + "/tiles/" + "Tile_" + std::to_string((int)x / tilesize) + "_" + std::to_string((int)y / tilesize) + "_" + file.baseName().toStdString() + ".shp";

	 //					remove(name.c_str());
	 //					OGRDataSource * DS = Driver->CreateDataSource(name.c_str(), NULL);

	 //					OGRLayer * Layer = DS->CreateLayer("Layer1");

	 //					poLayer->ResetReading();
	 //					while ((poFeature = poLayer->GetNextFeature()) != NULL)
	 //					{
	 //						OGRGeometry* poGeometry = poFeature->GetGeometryRef();

	 //						if (poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
	 //						{
	 //							OGRPolygon* poPG = (OGRPolygon*)poGeometry;

	 //							if (!poPG->Intersects(Tuile))
	 //								continue;
	 //							OGRGeometry * Geometry;
	 //							if (ret == QMessageBox::No)
	 //							{
	 //								Geometry = poPG->Intersection(Tuile);
	 //								if (!Geometry->IsValid() || Geometry->IsEmpty() || Geometry->getGeometryType() != wkbPolygon25D && Geometry->getGeometryType() != wkbPolygon && Geometry->getGeometryType() != wkbMultiPolygon25D && Geometry->getGeometryType() != wkbMultiPolygon)
	 //									continue;
	 //							}
	 //							if (ret == QMessageBox::Yes)
	 //							{
	 //								OGRPoint* Centroid = new OGRPoint;
	 //								poPG->Centroid(Centroid);

	 //								if (!Tuile->Contains(Centroid))
	 //									continue;

	 //								Geometry = poPG;
	 //								delete Centroid;
	 //							}

	 //							for (int i = 0; i < poFeature->GetFieldCount(); ++i)//Ne servira que la première fois, pour la première poFeature
	 //							{
	 //								if (Layer->FindFieldIndex(poFeature->GetFieldDefnRef(i)->GetNameRef(), 1) == -1)
	 //									Layer->CreateField(new OGRFieldDefn(poFeature->GetFieldDefnRef(i)->GetNameRef(), poFeature->GetFieldDefnRef(i)->GetType()));
	 //							}

	 //							OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());

	 //							Feature->SetGeometry(Geometry);

	 //							//Ajout des données sémantiques du shapefile
	 //							for (int i = 0; i < poFeature->GetFieldCount(); ++i)
	 //								Feature->SetField(poFeature->GetFieldDefnRef(i)->GetNameRef(), poFeature->GetFieldAsString(i));

	 //							Layer->CreateFeature(Feature);

	 //							OGRFeature::DestroyFeature(Feature);
	 //						}
	 //					}
	 //					OGRDataSource::DestroyDataSource(DS);
	 //					delete Tuile;
	 //				}
	 //				std::cout << "\rTiling... (" << (int)100 * (cpt + 1) / Tuiles.size() << "%)" << std::flush;
	 //			}
	 //		}
	 //		std::cout << "\rTiling done     " << std::endl;
	 //	}
	 //	m_osgView->setActive(true);
	 //}
} //namespace FloodAR
