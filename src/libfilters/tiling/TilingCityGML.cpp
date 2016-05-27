#include "TilingCityGML.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "libcitygml/utils/CityGMLtools.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtools.hpp"
////////////////////////////////////////////////////////////////////////////////
/**
* @brief Decoupe le fichier CityGML en un ensemble de tuiles dont la taille est definie en entree.
* @param Tile Contient les donnees du fichier CityGML ouvert : il doit contenir un ensemble de batiments LOD2 ou du terrain
* @param TexturesList : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie;
* @param MinTile : Coordonnee du coin bas gauche de la tuile
* @param MaxTile : Coordonnee du coin haut droit de la tuile
*/
citygml::CityModel* TileCityGML(vcity::Tile* Tile, std::vector<TextureCityGML*>* TexturesList, TVec2d MinTile, TVec2d MaxTile, std::string PathFolder)
{
    citygml::CityModel* Tuile = new citygml::CityModel;

    citygml::CityModel* Model = Tile->getCityModel();

    OGRPolygon* PolyTile = new OGRPolygon;
    OGRLinearRing* RingTile = new OGRLinearRing;
    RingTile->addPoint(MinTile.x, MinTile.y);
    RingTile->addPoint(MinTile.x, MaxTile.y);
    RingTile->addPoint(MaxTile.x, MaxTile.y);
    RingTile->addPoint(MaxTile.x, MinTile.y);
    RingTile->addPoint(MinTile.x, MinTile.y);
    PolyTile->addRingDirectly(RingTile);

    OGRLineString* WestLine = new OGRLineString;
    OGRLineString* EastLine = new OGRLineString;
    OGRLineString* NorthLine = new OGRLineString;
    OGRLineString* SouthLine = new OGRLineString;

    WestLine->addPoint(MinTile.x, MinTile.y);
    WestLine->addPoint(MinTile.x, MaxTile.y);
    EastLine->addPoint(MaxTile.x, MinTile.y);
    EastLine->addPoint(MaxTile.x, MaxTile.y);
    NorthLine->addPoint(MinTile.x, MaxTile.y);
    NorthLine->addPoint(MaxTile.x, MaxTile.y);
    SouthLine->addPoint(MinTile.x, MinTile.y);
    SouthLine->addPoint(MaxTile.x, MinTile.y);

    for (citygml::CityObject* obj : Model->getCityObjectsRoots())
    {
        if (obj->getType() == citygml::COT_TINRelief || obj->getType() == citygml::COT_WaterBody)
        {
            std::string Name = obj->getId();
            citygml::CityObject* TIN_CO;
            if (obj->getType() == citygml::COT_TINRelief)
                TIN_CO = new citygml::TINRelief(Name);
            else if (obj->getType() == citygml::COT_WaterBody)
                TIN_CO = new citygml::WaterBody(Name);

            citygml::Geometry* TIN = new citygml::Geometry(Name + "_TINGeometry", citygml::GT_Unknown, 2);

            int cptPolyTIN = 0;

            for (citygml::Geometry* Geometry : obj->getGeometries())
            {
                for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                {
                    OGRLinearRing * OgrRing = new OGRLinearRing;
                    for (TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
                        OgrRing->addPoint(Point.x, Point.y, Point.z);

                    std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

                    bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

                    if (HasTexture && PolygonCityGML->getTexture()->getType() == "GeoreferencedTexture") //Ce sont des coordonnees georeferences qu'il faut convertir en coordonnees de texture standard
                    {
                        /*double A, B, C ,D; //Voir fr.wikipedia.org/wiki/World_file : Taille pixel, rotation, retournement //Pour faire une conversion propre.
                        double offset_x;
                        double offset_y;

                        std::string path = PathFolder + "/" + PolygonCityGML->getTexture()->getUrl().substr(0, PolygonCityGML->getTexture()->getUrl().find_last_of('.'))+".jgw";
                        std::cout << path << std::endl;
                        std::ifstream fichier(path, std::ios::in);

                        if(fichier)
                        {
                        fichier >> A >> B >> C >> D >> offset_x >> offset_y;
                        fichier.close();
                        }
                        std::cout << A << " " << B << " " << C << " " << D << " " << offset_x << " " << offset_y << std::endl;*/


                        //////////////////////////////// MARCHE POUR DES TEXTURES 4096x4096 avec un D negatif (donnees de LYON)
                        int i = 0;
                        for (TVec2f UV : TexUV)
                        {
                            UV.x = UV.x / 4095;
                            UV.y = 1 + UV.y / 4095;//Car D est negatif
                            TexUV.at(i) = UV;
                            ++i;
                        }
                    }

                    OgrRing->closeRings();

                    if (OgrRing->getNumPoints() > 3)
                    {
                        OGRPolygon * OgrPoly = new OGRPolygon;
                        OgrPoly->addRingDirectly(OgrRing);
                        if (OgrPoly->IsValid() && OgrPoly->Intersects(PolyTile))
                        {
                            std::string Url;
                            citygml::Texture::WrapMode WrapMode;
                            std::vector<std::vector<TVec2f>> TexUVout;
                            if (HasTexture)
                            {
                                Url = PolygonCityGML->getTexture()->getUrl();
                                WrapMode = PolygonCityGML->getTexture()->getWrapMode();
                            }

                            OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, PolyTile, &TexUV, &TexUVout);

                            if (CutPoly != nullptr)
                            {
                                if (CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
                                {
                                    citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
                                    TIN->addPolygon(GMLPoly);
                                    if (HasTexture)
                                    {
                                        TexturePolygonCityGML Poly;

                                        Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
                                        Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
                                        Poly.TexUV = TexUVout.at(0);

                                        bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                        for (TextureCityGML* Tex : *TexturesList)
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
                                            TexturesList->push_back(Texture);
                                        }
                                    }
                                    ++cptPolyTIN;
                                }
                            }
                            else
                            {
                                OGRMultiPolygon* CutMultiPoly = dynamic_cast<OGRMultiPolygon*>(CutPoly);
                                if (CutMultiPoly != nullptr)
                                {
                                    for (int i = 0; i < CutMultiPoly->getNumGeometries(); ++i)
                                    {
                                        if (((OGRPolygon*)CutMultiPoly->getGeometryRef(i))->get_Area() < Precision_Vect)
                                            continue;

                                        citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutMultiPoly->getGeometryRef(i), Name + "_" + std::to_string(cptPolyTIN));
                                        TIN->addPolygon(GMLPoly);
                                        if (HasTexture)
                                        {
                                            TexturePolygonCityGML Poly;

                                            Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
                                            Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
                                            Poly.TexUV = TexUVout.at(i);

                                            bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                            for (TextureCityGML* Tex : *TexturesList)
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
                                                TexturesList->push_back(Texture);
                                            }
                                        }
                                        ++cptPolyTIN;
                                    }
                                }
                            }
                        }
                        delete OgrPoly;
                        // TODO : Re-trianguler les polygones du TIN decoupes.
                    }
                    else
                        delete OgrRing;
                }
            }

            if (TIN->getPolygons().size() > 0)
            {
                TIN_CO->addGeometry(TIN);
                Tuile->addCityObject(TIN_CO);
                Tuile->addCityObjectAsRoot(TIN_CO);
            }
        }
        else if (obj->getType() == citygml::COT_Building)
        {
            std::string Name = obj->getId();
            citygml::CityObject* BuildingCO = new citygml::Building(Name);
            citygml::CityObject* RoofCO = new citygml::RoofSurface(Name + "_Roof");
            citygml::Geometry* Roof = new citygml::Geometry(Name + "_RoofGeometry", citygml::GT_Roof, 2);
            citygml::CityObject* WallCO = new citygml::WallSurface(Name + "_Wall");
            citygml::Geometry* Wall = new citygml::Geometry(Name + "_WallGeometry", citygml::GT_Wall, 2);

            int cptPolyRoof = 0; //Compteur de polygones representant un Roof du batiment courant (pour avoir des noms differents)
            int cptPolyWall = 0; //Compteur de polygones representant un Wall du batiment courant (pour avoir des noms differents)

            for (citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du batiment
            {
                if (object->getType() == citygml::COT_RoofSurface)
                {
                    for (citygml::Geometry* Geometry : object->getGeometries()) //pour chaque geometrie
                    {
                        for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                        {
                            OGRLinearRing * OgrRing = new OGRLinearRing;
                            for (TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
                                OgrRing->addPoint(Point.x, Point.y, Point.z);

                            std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

                            OgrRing->closeRings();
                            if (OgrRing->getNumPoints() > 3)
                            {
                                OGRPolygon * OgrPoly = new OGRPolygon;
                                OgrPoly->addRingDirectly(OgrRing);
                                if (OgrPoly->IsValid() && OgrPoly->Intersects(PolyTile))
                                {
                                    bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

                                    std::string Url;
                                    citygml::Texture::WrapMode WrapMode;
                                    std::vector<std::vector<TVec2f>> TexUVout;
                                    if (HasTexture)
                                    {
                                        Url = PolygonCityGML->getTexture()->getUrl();
                                        WrapMode = PolygonCityGML->getTexture()->getWrapMode();
                                    }

                                    OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, PolyTile, &TexUV, &TexUVout);

                                    if (CutPoly != nullptr)
                                    {
                                        if (CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
                                        {
                                            if (((OGRPolygon*)CutPoly)->get_Area() < Precision_Vect) //Pour eliminer les polygons plats
                                            {
                                                delete CutPoly;
                                                continue;
                                            }

                                            citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_Roof_" + std::to_string(cptPolyRoof));
                                            Roof->addPolygon(GMLPoly);
                                            if (HasTexture)
                                            {
                                                TexturePolygonCityGML Poly;

                                                Poly.Id = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Poly";
                                                Poly.IdRing = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Ring";
                                                Poly.TexUV = TexUVout.at(0);

                                                bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                                for (TextureCityGML* Tex : *TexturesList)
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
                                                    TexturesList->push_back(Texture);
                                                }
                                            }
                                            ++cptPolyRoof;
                                        }
                                        else
                                        {
                                            OGRMultiPolygon* CutMultiPoly = dynamic_cast<OGRMultiPolygon*>(CutPoly);
                                            if (CutMultiPoly != nullptr)
                                            {
                                                for (int i = 0; i < CutMultiPoly->getNumGeometries(); ++i)
                                                {
                                                    if (((OGRPolygon*)CutMultiPoly->getGeometryRef(i))->get_Area() < Precision_Vect)
                                                        continue;

                                                    citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutMultiPoly->getGeometryRef(i), Name + "_Roof_" + std::to_string(cptPolyRoof));
                                                    Roof->addPolygon(GMLPoly);
                                                    if (HasTexture)
                                                    {
                                                        TexturePolygonCityGML Poly;

                                                        Poly.Id = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Poly";
                                                        Poly.IdRing = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Ring";
                                                        Poly.TexUV = TexUVout.at(i);

                                                        bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                                        for (TextureCityGML* Tex : *TexturesList)
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
                                                            TexturesList->push_back(Texture);
                                                        }
                                                    }
                                                    ++cptPolyRoof;
                                                }
                                            }
                                        }
                                    }
                                }
                                delete OgrPoly;
                            }
                            else
                                delete OgrRing;
                        }
                    }
                }
                else if (object->getType() == citygml::COT_WallSurface)
                {
                    for (citygml::Geometry* Geometry : object->getGeometries()) //pour chaque geometrie
                    {
                        for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                        {
                            OGRLinearRing * WallRing = new OGRLinearRing;

                            std::vector<TVec3d> PointsWall = PolygonCityGML->getExteriorRing()->getVertices();
                            for (int i = 0; i < PointsWall.size(); ++i) //Le premier point n'est pas repete a la fin
                            {
                                TVec3d Point = PointsWall.at(i);
                                WallRing->addPoint(Point.x, Point.y, Point.z);
                            }
                            WallRing->closeRings();

                            if (WallRing->getNumPoints() > 3)
                            {
                                OGRPolygon * WallPoly = new OGRPolygon;
                                WallPoly->addRingDirectly(WallRing);
                                if (WallPoly->Distance(PolyTile) < Precision_Vect) //Le polygon du Wall semble intersecter le Roof du Shape, on va donc l'ajouter a ce batiment.
                                {
                                    bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

                                    std::string Url;
                                    citygml::Texture::WrapMode WrapMode;
                                    std::vector<TVec2f> TexUV;

                                    if (HasTexture)
                                    {
                                        Url = PolygonCityGML->getTexture()->getUrl();
                                        WrapMode = PolygonCityGML->getTexture()->getWrapMode();
                                        TexUV = PolygonCityGML->getTexCoords();
                                        TexUV.push_back(PolygonCityGML->getTexCoords().at(0));
                                    }

                                    std::vector<TVec2f> TexUVWall;

                                    OGRPolygon* WallPolyRes = new OGRPolygon; //Contiendra le polygon du Wall que l'on aura limite a la tuile
                                    OGRLinearRing* WallRingRes = new OGRLinearRing;

                                    for (int i = 0; i < WallRing->getNumPoints() - 1; ++i)
                                    {
                                        OGRPoint* WallPoint1 = new OGRPoint(WallRing->getX(i), WallRing->getY(i), WallRing->getZ(i));
                                        OGRPoint* WallPoint2 = new OGRPoint(WallRing->getX(i + 1), WallRing->getY(i + 1), WallRing->getZ(i + 1));

                                        if (WallPoint1->Distance(PolyTile) < Precision_Vect && WallPoint2->Distance(PolyTile) < Precision_Vect) //Les deux points de la ligne sont dans la tuile, rien a faire
                                        {
                                            WallRingRes->addPoint(WallPoint1);
                                            WallRingRes->addPoint(WallPoint2);
                                            if (HasTexture)
                                            {
                                                TexUVWall.push_back(TexUV.at(i));
                                                TexUVWall.push_back(TexUV.at(i + 1));
                                            }
                                        }
                                        else
                                        {
                                            OGRLineString* Line = new OGRLineString;
                                            Line->addPoint(WallPoint1);
                                            Line->addPoint(WallPoint2);
                                            if (WallPoint1->Distance(PolyTile) < Precision_Vect) //Seul le premier point est dans la tuile, il faut donc calculer le second
                                            {
                                                WallRingRes->addPoint(WallPoint1);
                                                if (HasTexture)
                                                    TexUVWall.push_back(TexUV.at(i));
                                                if (Line->Intersects(EastLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(EastLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                else if (Line->Intersects(WestLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(WestLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                else if (Line->Intersects(NorthLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(NorthLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                else if (Line->Intersects(SouthLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(SouthLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                            }
                                            else if (WallPoint2->Distance(PolyTile) < Precision_Vect)
                                            {
                                                if (Line->Intersects(EastLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(EastLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                else if (Line->Intersects(WestLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(WestLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                else if (Line->Intersects(NorthLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(NorthLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                else if (Line->Intersects(SouthLine))
                                                {
                                                    OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(SouthLine));
                                                    if (Point != nullptr)
                                                    {
                                                        WallRingRes->addPoint(Point);
                                                        if (HasTexture)
                                                            TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
                                                    }
                                                    delete Point;
                                                }
                                                WallRingRes->addPoint(WallPoint2);
                                                if (HasTexture)
                                                    TexUVWall.push_back(TexUV.at(i + 1));
                                            }
                                            else if (Line->Intersects(PolyTile)) // Cas où la ligne est dans un coin, aucun des points n'est dans la tuile mais une partie de la ligne y est
                                            {
                                                OGRLineString* InterLine = dynamic_cast<OGRLineString*>(Line->Intersection(SouthLine));
                                                if (InterLine != nullptr)
                                                {
                                                    //double Z1 = WallPoint1->getZ() + (WallPoint2->getZ() - WallPoint1->getZ())*(InterLine->getX(0) - WallPoint1->getX())/(WallPoint2->getX() - WallPoint1->getX()); //Calcul de la coordonnee z car l'intersection de GDAL est 2D
                                                    //double Z2 = WallPoint1->getZ() + (WallPoint2->getZ() - WallPoint1->getZ())*(InterLine->getX(1) - WallPoint1->getX())/(WallPoint2->getX() - WallPoint1->getX()); //Calcul de la coordonnee z car l'intersection de GDAL est 2D

                                                    WallRingRes->addPoint(InterLine->getX(0), InterLine->getY(0), InterLine->getZ(0));
                                                    WallRingRes->addPoint(InterLine->getX(1), InterLine->getY(1), InterLine->getZ(1));
                                                    if (HasTexture)
                                                    {
                                                        TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(InterLine->getX(0), InterLine->getY(0), InterLine->getZ(0))));
                                                        TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(InterLine->getX(1), InterLine->getY(1), InterLine->getZ(1))));
                                                    }
                                                }
                                                delete InterLine;
                                            }
                                            delete Line;
                                        }
                                        delete WallPoint1;
                                        delete WallPoint2;
                                    }

                                    if (WallRingRes != nullptr && WallRingRes->getNumPoints() > 0)
                                    {
                                        WallRingRes->closeRings();
                                        WallPolyRes->addRingDirectly(WallRingRes);
                                        citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)WallPolyRes, Name + "_Wall_" + std::to_string(cptPolyWall));
                                        delete WallPolyRes;

                                        Wall->addPolygon(GMLPoly);

                                        if (HasTexture)
                                        {
                                            TexturePolygonCityGML Poly;

                                            Poly.Id = Name + "_Wall_" + std::to_string(cptPolyWall) + "_Poly";
                                            Poly.IdRing = Name + "_Wall_" + std::to_string(cptPolyWall) + "_Ring";
                                            Poly.TexUV = TexUVWall;

                                            bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                                            for (TextureCityGML* Tex : *TexturesList)
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
                                                TexturesList->push_back(Texture);
                                            }
                                        }
                                        ++cptPolyWall;
                                    }
                                }
                                delete WallPoly;
                            }
                            else
                                delete WallRing;
                        }
                    }
                }
            }

            bool test = false;
            if (Roof->getPolygons().size() > 0)
            {
                RoofCO->addGeometry(Roof);
                Tuile->addCityObject(RoofCO);
                BuildingCO->insertNode(RoofCO);
                test = true;
            }
            if (Wall->getPolygons().size() > 0)
            {
                WallCO->addGeometry(Wall);
                Tuile->addCityObject(WallCO);
                BuildingCO->insertNode(WallCO);
                test = true;
            }
            if (test)
            {
                Tuile->addCityObject(BuildingCO);
                Tuile->addCityObjectAsRoot(BuildingCO);
            }
        }
    }

    delete EastLine;
    delete WestLine;
    delete NorthLine;
    delete SouthLine;
    delete PolyTile;

    return Tuile;
}

/**
* @brief Fusionne deux CityModel representant la meme tuile mais generees a partir de deux fichiers differents
* @param OldTile Contient les donnees du fichier CityGML deja existant qui modelise la tuile : il doit contenir un ensemble de batiments LOD2 ou du terrain
* @param NewTile Contient les donnees du CityModel nouvellemeent cree que l'on souhaite enregistrer en le fusionnant avec OldTile
* @param TexturesList : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie, sachant qu'il est deja rempli avec les textures de NewTile
*/
void MergingTile(vcity::Tile* OldTile, citygml::CityModel* NewTile, std::vector<TextureCityGML*>* TexturesList)
{
    citygml::CityModel* Model = OldTile->getCityModel();

    for (citygml::CityObject* obj : Model->getCityObjectsRoots())
    {
        if (obj->getType() == citygml::COT_Building)
        {
            NewTile->addCityObject(obj);
            NewTile->addCityObjectAsRoot(obj);

            for (citygml::CityObject* object : obj->getChildren())
            {
                for (citygml::Geometry* Geometry : object->getGeometries())
                {
                    for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                    {
                        bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

                        if (!HasTexture)
                            continue;

                        std::string Url = PolygonCityGML->getTexture()->getUrl();
                        citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

                        TexturePolygonCityGML Poly;
                        Poly.Id = PolygonCityGML->getId();
                        Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                        Poly.TexUV = PolygonCityGML->getTexCoords();

                        bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                        for (TextureCityGML* Tex : *TexturesList)
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
                            TexturesList->push_back(Texture);
                        }
                    }
                }
            }
        }
        else if (obj->getType() == citygml::COT_TINRelief || obj->getType() == citygml::COT_WaterBody)
        {
            NewTile->addCityObject(obj);
            NewTile->addCityObjectAsRoot(obj);

            for (citygml::Geometry* Geometry : obj->getGeometries())
            {
                for (citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
                {
                    bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

                    if (!HasTexture)
                        continue;

                    std::string Url = PolygonCityGML->getTexture()->getUrl();
                    citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

                    TexturePolygonCityGML Poly;
                    Poly.Id = PolygonCityGML->getId();
                    Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
                    Poly.TexUV = PolygonCityGML->getTexCoords();

                    bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
                    for (TextureCityGML* Tex : *TexturesList)
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
                        TexturesList->push_back(Texture);
                    }
                }
            }
        }
    }
}

/**
* @brief Ouvre un fichier .dat contenant les coordonnees de BoundingBox
* @param Dir : Chemin du fichier .dat
*/
citygml::CityModel* CreateBoundingBox(std::string dir)
{
    QDir dt(dir.c_str());

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
