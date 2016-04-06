#include "FloodARTools.hpp"

#include <QFileDialog>

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
} //namespace FloodAR
