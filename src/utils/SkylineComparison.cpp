#include "SkylineComparison.hpp"

#include <QString>
#include <QDir>
#include <fstream>

#include <vector>
#include <map>
#include <utility>
#include <iostream>
#include "ogrsf_frmts.h"
#include "vecs.hpp"


///
/// \brief OGRLayerToPointsVector Convert a OGRLayer to a Vector of OGRPoint
/// \param LayerSkylinePoints OGR Layer representing Skyline Points
/// \return Vector of OGRPoint*
///
std::vector<OGRPoint*> OGRLayerToPointsVector(OGRLayer* LayerSkylinePoints)
{
    std::vector<OGRPoint*> vPoints;

    OGRFeature *poFeature;
    LayerSkylinePoints->ResetReading();

    while( (poFeature = LayerSkylinePoints->GetNextFeature()) != NULL )
    {
        OGRGeometry* poGeometry = poFeature->GetGeometryRef();

        if(poGeometry != NULL && (poGeometry->getGeometryType() == OGRwkbGeometryType::wkbPoint25D || poGeometry->getGeometryType() == OGRwkbGeometryType::wkbPoint))
        {
            OGRPoint* point = dynamic_cast<OGRPoint*>(poGeometry);
            vPoints.push_back(point);
        }
    }

    return vPoints;
}


std::vector<double> ComputeDist(OGRLayer* LayerSkylinePoints1, OGRLayer* LayerSkylinePoints2)
{
    std::vector<double> dist;

    std::vector<OGRPoint*> vPoints1 = OGRLayerToPointsVector(LayerSkylinePoints1);
    std::vector<OGRPoint*> vPoints2 = OGRLayerToPointsVector(LayerSkylinePoints2);

    if(vPoints1.size() != vPoints2.size())
    {
        std::cout << "Error : Curves are not the same size, consider discretisation" << std::cout;
        return dist;
    }

    for(unsigned int i = 0; i < vPoints1.size(); ++i)
    {
        //Compute euclidian norm
        TVec3d vec = TVec3d(vPoints1.at(i)->getX() - vPoints2.at(i)->getX(), vPoints1.at(i)->getY() - vPoints2.at(i)->getY(), vPoints1.at(i)->getZ() - vPoints2.at(i)->getZ());
        double norm = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);

        dist.push_back(norm);
    }

    return dist;
}

std::map<std::string,Compo> BuildToCompoMap()
{
    std::map<std::string,Compo> ToCompo;

    ToCompo["GenericCityObject"] = Compo::Other;
    ToCompo["Building"] = Compo::Building;
    ToCompo["Room"] = Compo::Building;
    ToCompo["BuildingInstallation"] = Compo::Building;
    ToCompo["BuildingFurniture"] = Compo::Building;
    ToCompo["Door"] = Compo::Building;
    ToCompo["Window"] = Compo::Building;
    ToCompo["CityFurniture"] =  Compo::Other;
    ToCompo["Track"] =  Compo::Other;
    ToCompo["Road"] =  Compo::Other;
    ToCompo["Railway"] =  Compo::Other;
    ToCompo["Square"] =  Compo::Veget;
    ToCompo["PlantCover"] =  Compo::Veget;
    ToCompo["SolidaryVegetationObject"] =  Compo::Veget;
    ToCompo["WaterBody"] =  Compo::Water;
    ToCompo["TINRelief"] =  Compo::TIN;
    ToCompo["LandUse"] =  Compo::TIN;
    ToCompo["Tunnel"] =  Compo::Other;
    ToCompo["Bridge"] =  Compo::Other;
    ToCompo["BridgeConstructionElement"] =  Compo::Other;
    ToCompo["BridgeInstallation"] =  Compo::Other;
    ToCompo["BridgePart"] =  Compo::Other;
    ToCompo["BuildingPart"] = Compo::Building;
    ToCompo["WallSurface"] = Compo::Building;
    ToCompo["RoofSurface"] = Compo::Building;
    ToCompo["GroundSurface"] =  Compo::TIN;
    ToCompo["ClosureSurface"] = Compo::Building;
    ToCompo["FloorSurface"] = Compo::Building;
    ToCompo["InteriorWallSurface"] = Compo::Building;
    ToCompo["CeilingSurface"] = Compo::Building;
    ToCompo["All"] = Compo::Other;

    return ToCompo;
}

std::vector<std::pair<Compo,Compo>> AnalyseSkylineCompo(OGRLayer* LayerSkylinePoints1, OGRLayer* LayerSkylinePoints2)
{
    std::vector<std::pair<Compo,Compo>> skylinesCompo;

    std::map<std::string,Compo> ToCompo = BuildToCompoMap();

    OGRFeature *poFeature1;
    LayerSkylinePoints1->ResetReading();

    OGRFeature *poFeature2;
    LayerSkylinePoints2->ResetReading();

    while( (poFeature1 = LayerSkylinePoints1->GetNextFeature()) != NULL &&
           (poFeature2 = LayerSkylinePoints2->GetNextFeature()) != NULL )
    {
        std::string type1 = poFeature1->GetFieldAsString("ObjectType");
        std::string type2 = poFeature2->GetFieldAsString("ObjectType");

        skylinesCompo.push_back(std::make_pair(ToCompo[type1],ToCompo[type2]));
    }

    return skylinesCompo;

}


std::string EnumToString(Compo c)
{
    std::string sCompo;

    switch(c)
    {
         case Building:
            sCompo = "Building";
            break;
         case TIN:
            sCompo = "TIN";
            break;
         case Veget:
            sCompo = "Vegetation";
            break;
         case Water:
            sCompo = "Water";
            break;
         default:
            sCompo = "";
            break;
    }

    return sCompo;
}

void exportToCSV(std::string path, std::string filename, std::vector<double> dist, std::vector<std::pair<Compo,Compo>> compo)
{
    //Test if directory exists
    QDir outputDir(QString::fromStdString(path));
    if(!outputDir.exists(QString::fromStdString(path)))
    {
        std::cout << "Error : " << path << " : No such file or directory" << std::endl;
        return;
    }

    //Create and open file
    std::ofstream ofs;
    ofs.open(path + filename);

    ofs << "Order" << ";";

    for(unsigned int i = 1 ; i <= dist.size() ; ++i)
    {
        ofs << i << ";";
    }

    ofs << "" << std::endl;

    ofs << "Angle" << ";";

    double angle = 0.0;

    ofs << angle << ";";

    for(unsigned int i = 1 ; i < dist.size() ; ++i)
    {
        angle += 60.0 / dist.size();
        ofs << angle << ";";
    }

    ofs << "" << std::endl;

    ofs << "Dist" << ";";

    for(double d : dist)
    {
        ofs << d << ";";
    }

    ofs << "" << std::endl;

    ofs << "Skyline 1 Compo;";

   int cpt_building1 = 0;
   int cpt_tin1 = 0;
   int cpt_veget1 = 0;
   int cpt_water1 = 0;

   for(std::pair<Compo,Compo> c : compo)
   {
        switch(c.first)
        {
            case Building:
                ++cpt_building1;
                break;
            case TIN:
                ++cpt_tin1;
                break;
            case Veget:
                ++cpt_veget1;
                break;
            case Water:
                ++cpt_water1;
                break;
            default:
                break;
        }

        ofs << EnumToString(c.first) << ";";
   }                                 

   ofs << "" << std::endl;

   ofs << "Skyline 2 Compo;";

    int cpt_building2 = 0;
    int cpt_tin2 = 0;
    int cpt_veget2 = 0;
    int cpt_water2 = 0;

   for(std::pair<Compo,Compo> c : compo)
   {
        switch(c.second)
        {
            case Building:
                ++cpt_building2;
                break;
            case TIN:
                ++cpt_tin2;
                break;
            case Veget:
                ++cpt_veget2;
                break;
            case Water:
                ++cpt_water2;
                break;
            default:
                break;
        }

        ofs << EnumToString(c.second) << ";";
   }

    int sum = cpt_building1 + cpt_water1 + cpt_tin1 + cpt_veget1;

    ofs << "" << std::endl;

    ofs << "Skyline 1 Overall Composition" << ";";
    ofs << "Building : " << cpt_building1 * 100.0 / sum << "%" << ";";
    ofs << "TIN : " << cpt_tin1 * 100.0 / sum << "%" << ";";
    ofs << "Vegetation : " << cpt_veget1 * 100.0 / sum << "%" << ";";
    ofs << "Water : " << cpt_water1 * 100.0 / sum << "%" << ";";

    ofs << "" << std::endl;

    ofs << "Skyline 2 Overall Composition" << ";";
    ofs << "Building : " << cpt_building2 * 100.0 / sum << "%" << ";";
    ofs << "TIN : " << cpt_tin2 * 100.0 / sum << "%" << ";";
    ofs << "Vegetation : " << cpt_veget2 * 100.0 / sum << "%" << ";";
    ofs << "Water : " << cpt_water2 * 100.0 / sum << "%" << ";";

   ofs.close();
}

void CompareSkylines()
{
    //*******************Comparison of 2 skylines

    //Skylines
    std::string name_skyline1 = "Skyline_Bellecour_Without_Tower";
    std::string name_skyline2 = "Skyline_Bellecour_With_Tower";

    //Load 2 shapefiles
    std::string filepath1 = "/home/vincent/Documents/VCity_Project/NewFonctionalities/Comparaison_skylines/Compart/2/" + name_skyline1 + "/SkylinePoints.shp";

    std::cout << "load shp file : " << filepath1 << std::endl;
    OGRDataSource* poDS1 = OGRSFDriverRegistrar::Open(filepath1.c_str(), /*TRUE*/FALSE); //False pour read only et TRUE pour pouvoir modifier

    //Convert to OGRLayer
    OGRLayer* LayerSkylinePoints1;
    if(poDS1->GetLayerCount() > 0)
        LayerSkylinePoints1 = poDS1->GetLayer(0);
    else
    {
        std::cout << "Error while loading skyline : " << filepath1 <<std::endl;
        return;
    }


    std::string filepath2 = "/home/vincent/Documents/VCity_Project/NewFonctionalities/Comparaison_skylines/Compart/2/" + name_skyline2 + "/SkylinePoints.shp";
    std::cout << "load shp file : " << filepath2 << std::endl;
    OGRDataSource* poDS2 = OGRSFDriverRegistrar::Open(filepath2.c_str(), /*TRUE*/FALSE); //False pour read only et TRUE pour pouvoir modifier

    //Convert to OGRLayer
    OGRLayer* LayerSkylinePoints2;
    if(poDS2->GetLayerCount() > 0)
        LayerSkylinePoints2 = poDS2->GetLayer(0);
    else
    {
        std::cout << "Error while loading skyline : " << filepath2 <<std::endl;
        return;
    }

    //Compute dist between skylines
    std::vector<double> dist = ComputeDist(LayerSkylinePoints1,LayerSkylinePoints2);

    //Compare composition information
    std::vector<std::pair<Compo,Compo>> compoSkylines = AnalyseSkylineCompo(LayerSkylinePoints1,LayerSkylinePoints2);

    //Export result to csv
    std::string path = "/home/vincent/Documents/VCity_Project/NewFonctionalities/Comparaison_skylines/Compart/2/";
    std::string filename = "SkylineComparison.csv";

    exportToCSV(path, filename, dist, compoSkylines);

    std::cout << "Computation done." << std::endl;
}
