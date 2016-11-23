#include <osgDB/fstream>
#include <QFileDialog>
#include <QImageReader>
#include <QDebug>
#include <QDateTime>
#include <QMessageBox>
#include <QProgressDialog>

#include "export/exportCityGML.hpp"
#include "FloodARTools.hpp"
#include "DataStructures/DEM/osgMnt.hpp"
#include "importerASC.hpp"
#include "filters/Tiling/ASCCut.hpp"
#include "filters/ShpExtrusion/ShpExtrusion.hpp"

namespace FloodAR
{
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<TextureCityGML*> getTexturesList(citygml::CityModel* model, QDir tiledir, QFileInfo texturesPath)
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
                        float tileSizeX = model->getEnvelope().getUpperBound().x - model->getEnvelope().getLowerBound().x; // taille de la zone en metres
                        float tileSizeY = model->getEnvelope().getUpperBound().y - model->getEnvelope().getLowerBound().y;
                        int i = 0;
                        for (TVec2f UV : TexUV)
                        {
                            UV.x = (UV.x - offset_x) / tileSizeX;
                            UV.y = 1 + (UV.y - offset_y) / tileSizeY;//Car D est negatif
                            TexUV.at(i) = UV;
                            ++i;
                        }
                    }
                    //Remplissage de ListTextures
                    std::string Url = tiledir.relativeFilePath(texturesPath.filePath()).toStdString();
                    citygml::Texture::WrapMode WrapMode = citygml::Texture::WM_NONE;

                    TexturePolygonCityGML Poly;
                    Poly.Id = PolygonCityGML->getId();
                    Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                    Poly.TexUV = TexUV;

                    bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
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
        //Display a progress window
        int numSteps = (tmpDir.count() - 2) * 3;
        QProgressDialog progress("Converting files...", QString(), 0, numSteps, nullptr);//QString() for no cancel button
        progress.setWindowModality(Qt::WindowModal);
        // explore tmp directory for tile directories
        for (QFileInfo f : tmpDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot))
        {
            QDir tileDir(f.absoluteFilePath());
            std::string tilenumber = f.baseName().toStdString();
            citygml::CityModel* model = new citygml::CityModel();
            // TODO: explore the dir once to create the list of dates
            std::map<int, QFileInfo> fileList;
            for (QFileInfo ff : tileDir.entryInfoList(QDir::AllEntries | QDir::NoDotAndDotDot))
            {
                //TODO: detect the "_TX_" string in the filename
                std::string fname = ff.baseName().toStdString();
                std::size_t pos1 = fname.find("_T");
                std::size_t pos2 = fname.find("_", pos1 + 1);
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
            progress.setValue(progress.value() + 1);
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
            model->computeEnvelope();
            progress.setValue(progress.value() + 1);
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
            progress.setValue(progress.value() + 1);
        }
        progress.setValue(progress.maximum());
    }
    ////////////////////////////////////////////////////////////////////////////////
    void ASCtoTerrain(std::string workingDir)
    {

        QDir wkDir(workingDir.c_str());
        QDir tmpDir((workingDir + "/tmp/_MNT").c_str());
        // check if tmp directory exists in working directory
        if (!tmpDir.exists()) { std::cerr << "Input file not found!" << std::endl; return; }
        //Display a progress window
        int numSteps = (tmpDir.count() - 2) * 4;
        QProgressDialog progress("Converting files...", QString(), 0, numSteps, nullptr);//QString() for no cancel button
        progress.setWindowModality(Qt::WindowModal);
        // explore tmp directory for tile directories
        for (QFileInfo f : tmpDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot))
        {
            QDir tileDir(f.absoluteFilePath());
            std::string tilenumber = f.baseName().toStdString();
            wkDir.mkpath(("_MNT/" + tilenumber).c_str());
            QDir outputTileDir((wkDir.absolutePath().toStdString() + "/_MNT/" + tilenumber).c_str());

            citygml::CityModel* model = new citygml::CityModel();
            std::map<float, MNT*> fileList; //List of ASC rasters ordered by precision
            for (QFileInfo ff : tileDir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot))
            {
                std::cout << "Reading file " << ff.baseName().toStdString() << std::endl;
                QString ext = ff.suffix().toLower();
                if (ext == "asc")
                {
                    //lecture du fichier
                    MNT* asc = new MNT();
                    if (asc->charge(ff.absoluteFilePath().toStdString().c_str(), "ASC"))
                    {
                        fileList.insert(std::make_pair(asc->get_pas_x(), asc));
                    }
                }
            }
            progress.setValue(progress.value() + 1);
            if (fileList.size() > 2) { std::cerr << "Tile " << tilenumber << ": more than two ASC raster found, conversion aborted!" << std::endl; continue; }
            //TODO: convert into CityGML
            citygml::CityObject* terrainSurface;
            citygml::ImporterASC* importer = new citygml::ImporterASC();
            switch (fileList.size())
            {
            case 2:
            {
                MNT* morePrecise = fileList.begin()->second;
                MNT* lessPrecise = std::next(fileList.begin(), 1)->second;
                terrainSurface = importer->fusionResolutions(lessPrecise, morePrecise);
                delete lessPrecise;
                delete morePrecise;
            }
            break;
            case 1:
                MNT* asc = fileList.begin()->second;
                terrainSurface = importer->reliefToCityGML(asc);
                delete asc;
                break;
            }
            delete importer;
            if (terrainSurface->getGeometries().size() > 0)
            {
                model->addCityObject(terrainSurface);
                model->addCityObjectAsRoot(terrainSurface);
                model->computeEnvelope();
            }
            else
            {
                delete terrainSurface;
                std::cout << std::endl << "Export aborted: empty CityModel!" << std::endl;
                continue;
            }
            progress.setValue(progress.value() + 1);
            //add textures
            std::vector<TextureCityGML*> TexturesList;
            QDir appearanceDir((outputTileDir.path().toStdString() + "/Appearance").c_str());
            if (appearanceDir.exists())
            {
                std::list<QFileInfo> texFileList;
                for (QFileInfo ff : appearanceDir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot))
                {
                    if (ff.suffix().toLower() == "jpg")
                    {
                        texFileList.push_back(ff);
                    }
                }
                if (texFileList.size() > 1) std::cerr << "Warning: Tile " << tilenumber << " - More than one appearance file found." << std::endl;
                TexturesList = getTexturesList(model, outputTileDir, *(texFileList.begin()));
            }
            progress.setValue(progress.value() + 1);
            //export CityGML
            std::cout << "Export ...";
            if (model != nullptr && model->size() != 0)
            {
                std::string outfilename = outputTileDir.path().toStdString() + "/" + tilenumber + "_MNT.gml";
                std::cout << outfilename << std::endl;
                citygml::ExporterCityGML exporter(outfilename.c_str());
                exporter.exportCityModelWithListTextures(*model, &TexturesList);
                std::cout << "OK!" << std::endl;
            }
            else std::cout << std::endl << "Export aborted: empty CityModel!" << std::endl;
            delete model;
            progress.setValue(progress.value() + 1);
        }
        progress.setValue(progress.maximum());
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
            pxSize_y = std::abs(pxSize_y);
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
        //progress
        std::cout << "Tiling texture... (0%)\r";
        while (x < origWidth && y < origHeight)
        {
            //get bounds of the current tile
            float cornerX = NW_x + (x*pxSize_x);
            float cornerY = NW_y - (y*pxSize_y);
            int dvX = cornerX / tileSizeX;
            int dvY = (cornerY - 1) / tileSizeY;
            float tileXmin = dvX*tileSizeX;
            float tileXmax = (dvX + 1)*tileSizeX;
            float tileYmin = dvY*tileSizeY;
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

            QFileInfo file2(filename.c_str());
            std::string outputname = tileDir.path().toStdString() + "/Appearance/" + tilenumber + "_MNT.jpg";
            QImageReader reader2(file2.absoluteFilePath());
            reader2.setClipRect(QRect(x, y, width, height));
            QImage croppedImage = reader2.read();
            croppedImage.save(QString(outputname.c_str()), "JPG", -1);
            //tile is finished, set xy for next tile
            if ((x + width) < origWidth)
            {
                x = x + width; //same row, next column
            }
            else
            {
                x = 0; //next row, first column
                y = y + height;
                //progress
                std::cout << "Tiling texture... (" << 100 * y / origHeight << "%)\r";
                //progress.setValue(y);
            }
        }
        std::cout << "Tiling texture... (100%)" << std::endl;
    }
    ////////////////////////////////////////////////////////////////////////////////
    void CutShapeFile(std::string workingDir, int tilesize_x, int tilesize_y, std::string filename)
    {
        //progress window
        QProgressDialog progress("Converting files...", QString(), 0, 1000, nullptr);//QString() for no cancel button, 1000 used as placeholer
        progress.setWindowModality(Qt::WindowModal);

        QFileInfo file(filename.c_str());
        QDir dir(workingDir.c_str());

        const char * DriverName = "ESRI Shapefile";
        OGRSFDriver * Driver;

        OGRRegisterAll();
        Driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(DriverName);
        if (Driver == NULL)
        {
            printf("%s driver not available.\n", DriverName);
            return;
        }
        OGRDataSource* poDS = OGRSFDriverRegistrar::Open(filename.c_str(), FALSE);

        double Xmin = 100000000, Xmax = -100000000, Ymin = 100000000, Ymax = -100000000;

        int nbLayers = poDS->GetLayerCount();
        if (nbLayers > 0)
        {
            OGRLayer *poLayer = poDS->GetLayer(0);

            OGRFeature *poFeature;
            poLayer->ResetReading();

            while ((poFeature = poLayer->GetNextFeature()) != NULL)
            {
                OGRGeometry* poGeometry = poFeature->GetGeometryRef();

                if (poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
                {
                    OGRPolygon* poPG = (OGRPolygon*)poGeometry;

                    OGRLinearRing* poLR = poPG->getExteriorRing();

                    int nbPoints = poLR->getNumPoints();

                    for (int i = 0; i < nbPoints; ++i)//Pour recuperer les points de l'exterior ring
                    {
                        OGRPoint p;
                        poLR->getPoint(i, &p);

                        if (p.getX() > Xmax)
                            Xmax = p.getX();
                        if (p.getX() < Xmin)
                            Xmin = p.getX();
                        if (p.getY() > Ymax)
                            Ymax = p.getY();
                        if (p.getY() < Ymin)
                            Ymin = p.getY();
                    }
                }
            }

            Xmin = tilesize_x * ((int)Xmin / tilesize_x);
            Ymin = tilesize_y * ((int)Ymin / tilesize_y);
            Xmax = tilesize_x * ((int)Xmax / tilesize_x);
            Ymax = tilesize_y * ((int)Ymax / tilesize_y);

            //std::cout << std::setprecision(10) << "Boite englobante creee : " << Xmin << " " << Xmax << " | " << Ymin << " " << Ymax << std::endl;

            std::vector<OGRPolygon*> Tuiles;

            for (int x = (int)Xmin; x <= (int)Xmax; x += tilesize_x)
            {
                for (int y = (int)Ymin; y <= (int)Ymax; y += tilesize_y)
                {
                    OGRLinearRing* Ring = new OGRLinearRing;
                    Ring->addPoint(x, y);
                    Ring->addPoint(x + tilesize_x, y);
                    Ring->addPoint(x + tilesize_x, y + tilesize_y);
                    Ring->addPoint(x, y + tilesize_y);
                    Ring->addPoint(x, y);

                    OGRPolygon* Poly = new OGRPolygon;
                    Poly->addRingDirectly(Ring);

                    Tuiles.push_back(Poly);
                }
            }
            std::cout << Tuiles.size() << " tuiles crees" << std::endl;
            progress.setMaximum(Tuiles.size());

            int cpt = -1;
            for (int x = (int)Xmin; x <= (int)Xmax; x += tilesize_x)
            {
                for (int y = (int)Ymin; y <= (int)Ymax; y += tilesize_y)
                {
                    ++cpt;

                    OGRPolygon* Tuile = Tuiles.at(cpt);
                    //file.absoluteDir().mkdir(file.baseName());
                    std::string tilenumber = std::to_string((int)x / tilesize_x) + "_" + std::to_string((int)y / tilesize_y);
                    dir.mkpath(("tmp/_BATI/" + tilenumber).c_str());
                    std::string name = dir.path().toStdString() + "/tmp/_BATI/" + tilenumber + "/" + tilenumber + "_" + file.baseName().toStdString() + ".shp";

                    remove(name.c_str());
                    OGRDataSource * DS = Driver->CreateDataSource(name.c_str(), NULL);

                    OGRLayer * Layer = DS->CreateLayer("Layer1");

                    poLayer->ResetReading();
                    while ((poFeature = poLayer->GetNextFeature()) != NULL)
                    {
                        OGRGeometry* poGeometry = poFeature->GetGeometryRef();

                        if (poGeometry != NULL && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon))
                        {
                            OGRPolygon* poPG = (OGRPolygon*)poGeometry;

                            if (!poPG->Intersects(Tuile))
                                continue;
                            OGRGeometry * Geometry;

                            OGRPoint* Centroid = new OGRPoint;
                            poPG->Centroid(Centroid);

                            if (!Tuile->Contains(Centroid))
                                continue;

                            Geometry = poPG;
                            //delete Centroid; //Memory access violation exception thrown when deleting
                            Centroid->empty();

                            for (int i = 0; i < poFeature->GetFieldCount(); ++i)//Ne servira que la premiere fois, pour la premiere poFeature
                            {
                                if (Layer->GetLayerDefn()->GetFieldIndex(poFeature->GetFieldDefnRef(i)->GetNameRef()) == -1)
                                    Layer->CreateField(new OGRFieldDefn(poFeature->GetFieldDefnRef(i)->GetNameRef(), poFeature->GetFieldDefnRef(i)->GetType()));
                            }

                            OGRFeature * Feature = OGRFeature::CreateFeature(Layer->GetLayerDefn());

                            Feature->SetGeometry(Geometry);

                            //Ajout des donnees semantiques du shapefile
                            for (int i = 0; i < poFeature->GetFieldCount(); ++i)
                                Feature->SetField(poFeature->GetFieldDefnRef(i)->GetNameRef(), poFeature->GetFieldAsString(i));

                            Layer->CreateFeature(Feature);

                            OGRFeature::DestroyFeature(Feature);
                        }
                    }
                    OGRDataSource::DestroyDataSource(DS);
                    //delete Tuile; //Memory access violation exception thrown when deleting
                    Tuile->empty();
                }
                std::cout << "\rTiling... (" << (int)100 * (cpt + 1) / Tuiles.size() << "%)" << std::flush;
                progress.setValue(cpt + 1);
            }
        }
        std::cout << "\rTiling done     " << std::endl;
    }
    ////////////////////////////////////////////////////////////////////////////////
    void ShapeExtrusion(std::string workingDir)
    {
        QDir wkDir(workingDir.c_str());
        QDir tmpDir((workingDir + "/tmp/_BATI").c_str());
        // check if tmp directory exists in working directory
        if (!tmpDir.exists()) { std::cerr << "Input file not found!" << std::endl; return; }
        //Display a progress window
        int numSteps = (tmpDir.count() - 2);
        QProgressDialog progress("Converting files...", QString(), 0, numSteps, nullptr);//QString() for no cancel button
        progress.setWindowModality(Qt::WindowModal);
        progress.setValue(0);
        // explore tmp directory for tile directories
        for (QFileInfo f : tmpDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot))
        {
            QDir tileDir(f.absoluteFilePath());
            std::string tilenumber = f.baseName().toStdString();
            wkDir.mkpath(("_BATI/" + tilenumber).c_str());
            QDir outputTileDir((wkDir.absolutePath().toStdString() + "/_BATI/" + tilenumber).c_str());

            std::list<QFileInfo> fileList;
            for (QFileInfo ff : tileDir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot))
            {
                QString ext = ff.suffix().toLower();
                if (ext == "shp")
                {
                    //lecture du fichier
                    std::cout << "Reading file " << ff.baseName().toStdString() << std::endl;
                    fileList.push_back(ff);
                }
            }
            if (fileList.size() != 1)continue;
            QFileInfo file = *fileList.begin();
            //Convert into CityGML
            std::string outputfile = outputTileDir.path().toStdString() + "/" + tilenumber + "_BATI.gml";

            QString ext = file.suffix().toLower();

            if (ext == "shp")
            {
                OGRDataSource* poDS = OGRSFDriverRegistrar::Open(file.absoluteFilePath().toStdString().c_str(), FALSE);
                std::cout << "Shp loaded" << std::endl;
                std::cout << "Processing..." << std::endl;

                citygml::CityModel* ModelOut = ShpExtrusion(poDS, workingDir + "/");

                delete poDS;

                citygml::ExporterCityGML exporter(outputfile);
                exporter.exportCityModel(*ModelOut);
                std::cout << "Done exporting" << std::endl;
            }

            progress.setValue(progress.value() + 1);
        }
        progress.setValue(progress.maximum());
    }
    ////////////////////////////////////////////////////////////////////////////////
} //namespace FloodAR
