#include "FloodARTools.hpp"

#include "osgDB/fstream"

#include <QFileDialog>
#include <QImageReader>
#include <QDebug>
#include <QMessageBox>

#include "src/DataStructures/DEM/osgMnt.hpp"
#include "import/importerASC.hpp"
#include "libfilters/tiling/ASCCut.hpp"

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
  void cutASC(std::string filePath, std::string workingDir, int tileSizeX, int tileSizeY, bool isTerrain)
  {
    QDir wkDir(workingDir.c_str());
    QDir outDir;
    if (isTerrain)
    {
      wkDir.mkpath("tmp/_MNT");
      outDir = QDir((workingDir + "/tmp/_MNT").c_str());
    }
    else
    {
      wkDir.mkpath("tmp/_WATER");
      outDir = QDir((workingDir + "/tmp/_WATER").c_str());
    }

    //reading file
    citygml::ImporterASC* importer = new citygml::ImporterASC();
    MNT* asc = new MNT();
    if (asc->charge(filePath.c_str(), "ASC"))
    {
      int xMin = floor(asc->get_x_noeud_NO() / tileSizeX);
      int yMin = floor(asc->get_y_noeud_NO() / tileSizeY);
      int xMax = floor((asc->get_x_noeud_NO() + asc->get_dim_x()*asc->get_pas_x()) / tileSizeX) + 1;
      int yMax = floor((asc->get_y_noeud_NO() + asc->get_dim_y()*asc->get_pas_y()) / tileSizeY) + 1;
      for (int y = yMin; y < yMax; y++)
      {
        for (int x = xMin; x < xMax; x++)
        {
          MNT* tiledDEM = BuildTile(asc, tileSizeX, tileSizeY, x, y);
          if (tiledDEM == nullptr) continue;
          std::string baseName = QFileInfo(filePath.c_str()).baseName().toStdString();
          if (!outDir.exists((std::to_string(x) + "-" + std::to_string(y)).c_str())) outDir.mkdir((std::to_string(x) + "_" + std::to_string(y)).c_str());
          std::string outputname = outDir.absolutePath().toStdString() + "/" + std::to_string(x) + "_" + std::to_string(y) + "/" + std::to_string(x) + "_" + std::to_string(y) + "_" + baseName + ".asc";
          tiledDEM->write(outputname.c_str());
          delete tiledDEM;
        }
        std::cout << "Tiling (" << (int)((y - yMin)*100.0 / (yMax - yMin)) << "%)\r";
      }
      std::cout << "Tiling (100%)" << std::endl;
    }
    delete importer;
    delete asc;
  }
  ////////////////////////////////////////////////////////////////////////////////
  void ASCtoWaterAuto(std::string workingDir, float prec, std::string startingDate)
  {
    QDir wkDir(workingDir.c_str());
    QDir tmpDir((workingDir + "/tmp/_WATER").c_str());
    // check if tmp directory exists in working directory
    if (!tmpDir.exists()) { std::cerr << "Input file not found!" << std::endl; return; }
    // explore tmp directory for tile directories
    for (QFileInfo f : tmpDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot))
    {
      QDir tileDir(f.absoluteFilePath());
      std::string tilenumber = f.baseName().toStdString();
      citygml::CityModel* model = new citygml::CityModel();
      // TODO: explore the dir once to create the list of dates
      std::map<int, QFileInfo> fileList;
      for (QFileInfo ff : tileDir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot))
      {
        //TODO: detect the "_TX_" string in the filename
        std::string fname = ff.baseName().toStdString();
        std::size_t pos1 = fname.find_first_of("_T");
        std::size_t pos2 = fname.find_first_of("_", pos1 + 1);
        int startTime;
        try
        {
          startTime = std::stoi(fname.substr(pos1 + 2, pos2));
        }
        catch (const std::invalid_argument&)
        {
          std::cerr << "Temporal information could not be detected, file skipped!" << std::endl;
          continue;
        }
        //TODO: create a list or something taht contains the dates of start, end, and filenames
        fileList.insert(std::make_pair(startTime, ff));
      }
      // Do treatement (TODO: parse the lst created just before instead)
      for (std::map<int, QFileInfo>::iterator it = fileList.begin(); it != fileList.end(); it++)
      {
        QFileInfo ff = it->second;
        std::cout << "CONVERTING FILE " << ff.baseName().toStdString() << std::endl;
        QString ext = ff.suffix().toLower();
        if (ext == "asc")
        {
          //lecture du fichier
          citygml::ImporterASC* importer = new citygml::ImporterASC();
          MNT* asc = new MNT();
          if (asc->charge(ff.absoluteFilePath().toStdString().c_str(), "ASC"))
          {
            //conversion en structure CityGML
            citygml::CityObject* waterbody = importer->waterToCityGMLPolygons(asc, prec);
            if (waterbody->getGeometries().size() > 0)
            {
              //Add temporal info
              int startTime = it->first;
              QDateTime creaDate = QDateTime::fromString(startingDate.c_str(), Qt::ISODate).addSecs(startTime * 3600);
              waterbody->setAttribute("creationDate", creaDate.toString(Qt::ISODate).toStdString());

              std::map<int, QFileInfo>::iterator it2 = std::next(it, 1);
              if (it2 != fileList.end())
              {
                int endTime = it2->first;
                QDateTime termDate = QDateTime::fromString(startingDate.c_str(), Qt::ISODate).addSecs(endTime * 3600);
                waterbody->setAttribute("terminationDate", termDate.toString(Qt::ISODate).toStdString());
              }
              // Add Surface to CityModel
              model->addCityObject(waterbody);
              model->addCityObjectAsRoot(waterbody);
            }
            else
            {
              delete waterbody;
            }
            delete importer;
            delete asc;
          }
        }
      }
      //export en CityGML
      std::cout << "Export ...";
      if (model != nullptr && model->size() != 0)
      {
        wkDir.mkpath(("_WATER/" + tilenumber).c_str());
        citygml::ExporterCityGML exporter((wkDir.path().toStdString() + "/_WATER/" + tilenumber + "/" + tilenumber + "_WATER.gml"));
        exporter.exportCityModel(*model);
        std::cout << "OK!" << std::endl;
      }
      else std::cout << std::endl << "Export aborted: empty CityModel!" << std::endl;
      delete model;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////
  void ASCtoWaterManual(std::string workingDir, std::string inputfilepath, std::string savefilepath, float prec, std::string creaDate, std::string termDate)
  {
    QDir wkDir(workingDir.c_str());
    citygml::CityModel* model = new citygml::CityModel();
    QFileInfo ff(inputfilepath.c_str());
    std::string tilenumber;
    std::cout << "CONVERTING FILE " << ff.baseName().toStdString() << std::endl;
    QString ext = ff.suffix().toLower();
    if (ext == "asc")
    {
      //lecture du fichier
      citygml::ImporterASC* importer = new citygml::ImporterASC();
      MNT* asc = new MNT();
      if (asc->charge(ff.absoluteFilePath().toStdString().c_str(), "ASC"))
      {
        //conversion en structure CityGML
        citygml::CityObject* waterbody = importer->waterToCityGMLPolygons(asc, prec);
        if (waterbody->getGeometries().size() > 0)
        {
          model->addCityObject(waterbody);
          model->addCityObjectAsRoot(waterbody);
        }
        else
        {
          delete waterbody;
        }
        delete importer;
        delete asc;
      }
    }
    //Add temporal info
    for (citygml::CityObject* obj : model->getCityObjectsRoots())
    {
      obj->setAttribute("creationDate", creaDate);
      obj->setAttribute("terminationDate", termDate);
    }

    //export en CityGML
    std::cout << "Export ...";
    if (model != nullptr && model->size() != 0)
    {
      citygml::ExporterCityGML exporter(savefilepath);
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
  void cutPicture(std::string filename, std::string workingDir, int tileSizeX, int tileSizeY)
  {

    float NW_x, NW_y;
    float pxSize_x, pxSize_y;
    float rx, ry;

    QFileInfo file(filename.c_str());

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

    QDir wkDir(workingDir.c_str());
    QDir outDir;
    wkDir.mkdir("_MNT");
    QDir MNTdir = wkDir.path() + "/_MNT";

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
      std::string outputname = tileDir.path().toStdString() + "/Appearance/" + tilenumber + "_MNT.jpg";
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
} //namespace FloodAR
