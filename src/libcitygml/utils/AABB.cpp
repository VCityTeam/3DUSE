#include "AABB.hpp"

#include <QDir>
#include <QFile>

#include <iostream>
#include <fstream>
//FIXME #include <osgDB/fstream> // MT 09/03/2016
#include <map>

#include "Triangle.hpp"
#include "TiledFilesLayout.hpp"

//AABB

bool AABB::operator==(AABB const& other)
{
    return name == other.name;
}


//BoxOrder

bool operator<(const BoxOrder& a, const BoxOrder& b)
{
    return a.order < b.order;
}

bool operator<(const BoxwithRays& a, const BoxwithRays& b)
{
    return a.minDistance < b.minDistance;
}


// AABBCollection

/**
*	@brief Load a collection of box from a file
*	@param path Path to the file
*	@return A collection of box
*/
std::vector<AABB> DoLoadAABB(std::string path)
{
    std::vector<AABB> bSet;

    char line[256];

    std::ifstream ifs(path, std::ifstream::in);

    ifs.getline(line, 256);

    unsigned int count = atoi(line);

    for (unsigned int i = 0; i < count; i++)
    {
        AABB box;
        ifs.getline(line, 256);

        box.name = std::string(line);

        //To avoid problems with files from Windows used on UNIX (Windows uses '/r/n' as 'new line' escape sequence and Unix systems use '/n' only).
        //In order to erase '\r' character if present.
        if (!box.name.empty() && *box.name.rbegin() == '\r')
            box.name.erase(box.name.length() - 1, 1);

        double minx;
        double miny;
        double minz;
        double maxx;
        double maxy;
        double maxz;

        ifs.getline(line, 256); minx = atof(line);
        ifs.getline(line, 256); miny = atof(line);
        ifs.getline(line, 256); minz = atof(line);
        ifs.getline(line, 256); maxx = atof(line);
        ifs.getline(line, 256); maxy = atof(line);
        ifs.getline(line, 256); maxz = atof(line);

        if (minx < maxx && miny < maxy && minz <= maxz)
        {
            box.min = TVec3d(minx, miny, minz);
            box.max = TVec3d(maxx, maxy, maxz);
            bSet.push_back(box);
        }
    }

    ifs.close();

    return bSet;
}

AABBCollection LoadAABB(std::string dir)
{
    // In order to add a new data set, uncomment exemple and replace fillers <..> by your data
    bool foundBuild = false;
    QFileInfo bDat;
    bool foundTerrain = false;
    QFileInfo tDat;
    bool foundWater = false;
    QFileInfo wDat;
    bool foundVeget = false;
    QFileInfo vDat;
    // bool found<MyData> = false;
    // QFileInfo <myData>Dat;

    //Check if our bounding box files do exists
    QDir dt(dir.c_str());
    if (dt.exists())
    {
        for (QFileInfo f : dt.entryInfoList())
        {
            if (f.isFile())
            {
                if (f.fileName() == "_BATI_AABB.dat")
                {
                    bDat = f.absoluteFilePath();
                    foundBuild = true;
                }
                if (f.fileName() == "_MNT_AABB.dat")
                {
                    tDat = f.absoluteFilePath();
                    foundTerrain = true;
                }
                if (f.fileName() == "_WATER_AABB.dat")
                {
                    wDat = f.absoluteFilePath();
                    foundWater = true;
                }
                if (f.fileName() == "_VEGET_AABB.dat")
                {
                    vDat = f.absoluteFilePath();
                    foundVeget = true;
                }
                // if(f.fileName() == "_<MyDataSuffix>_AABB.dat")
                // {
                // <myData>Dat = f.absoluteFilePath();
                // found<MyData> = true;
                // }
            }
        }
    }
    else
    {
        std::cout << "Error, files does not exists." << std::endl;
    }

    std::vector<AABB> bSet;
    std::vector<AABB> tSet;
    std::vector<AABB> wSet;
    std::vector<AABB> vSet;
    // std::vector<AABB> <myData>Set;


    if (foundBuild)
        bSet = DoLoadAABB(dir + "_BATI_AABB.dat");

    if (foundTerrain)
        tSet = DoLoadAABB(dir + "_MNT_AABB.dat");

    if (foundWater)
        wSet = DoLoadAABB(dir + "_WATER_AABB.dat");

    if (foundVeget)
        vSet = DoLoadAABB(dir + "_VEGET_AABB.dat");

    // if(foundVeget)
    // <myData>Set = DoLoadAABB(dir+"_<MyDataSuffix>_AABB.dat");

    AABBCollection collection;
    collection.building = bSet;
    collection.terrain = tSet;
    collection.water = wSet;
    collection.veget = vSet;
    // collection.<myData> = <myData>Set;

    return collection;
}

/**
*	@brief Build a collection of boxes from a citygml file in a set of directory
*	@param dirs Directories where citygml files are located
*	@param offset 3D offset used by the application
*	@param type Type of cityobject to use
*	@return a collection of boxes, key = name of the box, value = <min of the box, max of the box>
*/
std::map<std::string, std::pair<TVec3d, TVec3d>> DoBuildAABB(std::string dir, TiledLayer L, citygml::CityObjectsType type)
{
    std::map<std::string, std::pair<TVec3d, TVec3d>> AABBs;

    for (int x = L.TuileMinX; x <= L.TuileMaxX; ++x)
    {
        for (int y = L.TuileMinY; y <= L.TuileMaxY; ++y)
        {
            std::string FileName = dir + L.Name + "/" + std::to_string(x) + "_" + std::to_string(y) + "/" + std::to_string(x) + "_" + std::to_string(y) + L.Name + ".gml";

            TVec3d min(FLT_MAX, FLT_MAX, FLT_MAX);
            TVec3d max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

            TriangleList* list = BuildTriangleList(FileName, type);

            for (Triangle* t : list->triangles) //Pour eliminer les points anormaux (qui ont des coordonnees z absurdes), on fait un petit filtre en verifiant ces valeurs de z.
            {
                min.x = std::min(t->a.x, min.x); min.y = std::min(t->a.y, min.y); if (t->a.z > -500) min.z = std::min(t->a.z, min.z);
                min.x = std::min(t->b.x, min.x); min.y = std::min(t->b.y, min.y); if (t->b.z > -500) min.z = std::min(t->b.z, min.z);
                min.x = std::min(t->c.x, min.x); min.y = std::min(t->c.y, min.y); if (t->b.z > -500) min.z = std::min(t->c.z, min.z);
                max.x = std::max(t->a.x, max.x); max.y = std::max(t->a.y, max.y); if (t->a.z < 1000) max.z = std::max(t->a.z, max.z);
                max.x = std::max(t->b.x, max.x); max.y = std::max(t->b.y, max.y); if (t->b.z < 1000) max.z = std::max(t->b.z, max.z);
                max.x = std::max(t->c.x, max.x); max.y = std::max(t->c.y, max.y); if (t->c.z < 1000) max.z = std::max(t->c.z, max.z);
            }

            AABBs.insert(std::make_pair(L.Name + "/" + std::to_string(x) + "_" + std::to_string(y) + "/" + std::to_string(x) + "_" + std::to_string(y) + L.Name + ".gml", std::make_pair(min, max)));
            std::cout << "File : " << L.Name + "/" + std::to_string(x) + "_" + std::to_string(y) + "/" + std::to_string(x) + "_" + std::to_string(y) + L.Name + ".gml" << std::endl;

            delete list;
        }
    }

    return AABBs;
}

/**
*	@brief Save a collection of boxes on disk
*	@param filePath where to save the collection
*	@param AABBs The collection of box
*/
void DoSaveAABB(std::string filePath, std::map<std::string, std::pair<TVec3d, TVec3d>> AABBs)
{
    std::filebuf fb;
    fb.open(filePath, std::ios::out);

    std::ostream file(&fb);

    file << AABBs.size() << "\n";

    for (std::pair<std::string, std::pair<TVec3d, TVec3d>> p : AABBs)
    {
        file << p.first << "\n";
        file << std::fixed << p.second.first.x << "\n"; //std::fixed -> important pour conserver tous les chiffres significatifs (ne pas avoir de 1e19)
        file << std::fixed << p.second.first.y << "\n";
        file << std::fixed << p.second.first.z << "\n";
        file << std::fixed << p.second.second.x << "\n";
        file << std::fixed << p.second.second.y << "\n";
        file << std::fixed << p.second.second.z << "\n";
    }

    fb.close();
}

void BuildAABB(std::string dir)
{
    TiledFiles Files(dir);
    Files.BuildListofLayers();

    for (TiledLayer L : Files.ListofLayers)
    {
        citygml::CityObjectsType type;
        if (L.Name.find("_BATI") != std::string::npos)
            type = citygml::CityObjectsType::COT_Building;
        else if (L.Name.find("_MNT") != std::string::npos)
            type = citygml::CityObjectsType::COT_TINRelief;
        else if (L.Name.find("_WATER") != std::string::npos)
            type = citygml::CityObjectsType::COT_WaterBody;
        else if (L.Name.find("_VEGET") != std::string::npos)
            type = citygml::CityObjectsType::COT_SolitaryVegetationObject;

        std::map<std::string, std::pair<TVec3d, TVec3d>> AABBs = DoBuildAABB(dir, L, type); //Pour chaque tuile "string", bounding box : min-max

        DoSaveAABB(dir + L.Name + "_AABB.dat", AABBs);
    }

    std::cout << "Done." << std::endl;
}
