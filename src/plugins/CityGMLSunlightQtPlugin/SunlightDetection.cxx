#include <osgDB/fstream>
#include <queue>

#include <fstream>
#include <string>

#include "SunlightDetection.h"
#include "AABB.hpp"
#include "Triangle.hpp"
#include "filters/raytracing/RayTracing.hpp"
#include "filters/raytracing/Hit.hpp"
#include "core/RayBox.hpp"
#include "core/dateTime.hpp"
#include "IO.h"
#include "FileInfo.h"


///
/// \brief SetupFileOrder create a std::queue queueing the files intersected by a given RayBoxCollection and sorted by intersection distance
/// \param boxes Vector of files' bounding boxes to check intersection with
/// \param rays Rays for intersection with bounding boxes testing
/// \return a std::queue of RayboxHit sorted by intersection distance (min first)
///
std::queue<RayBoxHit> SetupFileOrder(const std::vector<AABB>& boxes,RayBoxCollection* rays)
{
    std::map<std::string,int> boxToMaxOrder; //The order in which the box has been crossed : if a ray cross multiple AABB, we record the order of crossing in each one
    std::map<std::string,RayBoxHit> boxToRayBoxHit;//We keep record of the smallest rayboxhit of a box /// MultiResolution

    for(unsigned int i = 0; i < boxes.size(); i++)
    {
        boxToMaxOrder[boxes[i].name] = -1;
    }

    //For each ray
    for(unsigned int i = 0; i < rays->raysBB.size(); i++)
    {
        RayBox* rayBox = rays->raysBB[i];
        rayBox->boxes.clear();

        //For each box we check if the ray intersect it
        for(unsigned int j = 0; j < boxes.size(); j++)
        {
            float hit0,hit1;
            if(rayBox->Intersect(boxes[j],&hit0,&hit1))
            {
                RayBoxHit hTemp;
                hTemp.box = boxes[j];
                hTemp.minDistance = hit0;
                rayBox->boxes.push_back(hTemp); //Add this box to list of intersected boxes by this ray
            }
        }

        std::sort(rayBox->boxes.begin(),rayBox->boxes.end());//Sort the boxes depending on their intersection distance

        for(std::size_t j = 0; j < rayBox->boxes.size(); j++)//We update the order of each box
        {
            int current = boxToMaxOrder[rayBox->boxes[j].box.name];
            boxToMaxOrder[rayBox->boxes[j].box.name] = std::max(static_cast<int>(j),current);

            if(boxToRayBoxHit.find(rayBox->boxes[j].box.name) != boxToRayBoxHit.end())
            // if rayBox->boxes[j].box.name already exists in boxToRayBoxHit
            // MultiResolution
            {
                boxToRayBoxHit[rayBox->boxes[j].box.name] = std::min(rayBox->boxes[j],boxToRayBoxHit[rayBox->boxes[j].box.name]);
            }
            else
            {
                boxToRayBoxHit.insert(std::make_pair(rayBox->boxes[j].box.name,rayBox->boxes[j]));
            }
        }
    }

    //Sort the boxes depending on their max order
    std::vector<BoxOrder> boxesOrder;

    for(auto it = boxToMaxOrder.begin();it != boxToMaxOrder.end(); it++)
    {
        if(it->second >= 0)
        {
            BoxOrder boTemp;
            boTemp.box = it->first;
            boTemp.order = it->second;
            boxesOrder.push_back(boTemp);
        }
    }

    std::sort(boxesOrder.begin(),boxesOrder.end());

    //Setup the queue and return it
    std::queue<RayBoxHit> fileOrder;

    for(BoxOrder& bo : boxesOrder)
        fileOrder.push(boxToRayBoxHit[bo.box]);

    return fileOrder;
}

///
/// \brief CreateRayBoxCollection Keep all the rays that intersect with the current AABB and
///                               that do not already correspond to an hour when the current triangle is in the shadow.
/// \param currentBBName Name of the current AABB.
/// \param allRayBoxes All rays for a triangle (corresponding to all the times of sunlight computation)
/// \param datetimeSunInfo Map holding sun and shadow information for current triangle.
/// \return A collection of RayBox intersecting with the current BB and not already intersecting with other geometries.
///
RayBoxCollection* CreateRayBoxCollection(const std::string& currentBBName, const RayBoxCollection* allRayBoxes, std::map<int,bool>& datetimeSunInfo)
{
    RayBoxCollection* raysBoxes = new RayBoxCollection();//Not all rays intersect the box

    //We only get the rays that intersect with the box
    for(unsigned int i = 0; i < allRayBoxes->raysBB.size(); i++)
    {
        RayBox* raybox = new RayBox();

        *raybox = *(allRayBoxes->raysBB[i]);//Get the ray

        bool found = false;

        for(RayBoxHit& rbh : raybox->boxes)//Go through all the boxes that the ray intersect to see if the current box is one of them
            found = found || rbh.box.name == currentBBName;

        if(found && datetimeSunInfo[raybox->id] == true) //If triangle is already in the shadow at this datetime, no need throw ray again
        {
            raysBoxes->raysBB.push_back(raybox);
        }
    }

    return raysBoxes;
}

///
/// \brief CreateRays Keep all the rays that intersect with the current AABB and
///                   that do not already correspond to an hour when the current triangle is in the shadow.
/// \param currentBBName Name of the current AABB.
/// \param allRayBoxes All rays for a triangle (corresponding to all the times of sunlight computation)
/// \param datetimeSunInfo Map holding sun and shadow information for current triangle.
/// \return A collection of Rays intersecting with the current BB and not already intersecting with other geometries.
///
RayCollection CreateRays(const std::string& currentBBName, const RayBoxCollection* allRayBoxes, std::map<int,bool>& datetimeSunInfo)
{
    RayCollection rays;//Not all rays intersect the box

    //We only get the rays that intersect with the box
    for(unsigned int i = 0; i < allRayBoxes->raysBB.size(); i++)
    {
        RayBox* raybox = allRayBoxes->raysBB[i];//Get the ray

        bool found = false;

        for(RayBoxHit& rbh : raybox->boxes)//Go through all the boxes that the ray intersect to see if the current box is one of them
            found = found || rbh.box.name == currentBBName;

        if(found && datetimeSunInfo[raybox->id] == true) //If triangle is already in the shadow at this datetime, no need throw ray again
            rays.rays.push_back(raybox);
    }

    return rays;

}

///
/// \brief RayTraceTriangles Load triangle list, run raytracing algorithm and update sunshine information.
/// \param filepath Path to file to check intersection with (current tile)
/// \param fileType Type of the file to check intersection with (current tile)
/// \param cityObjId Id of cityObject to test intersection with (Multi-résolution for _BATI)
/// \param rayColl Collection of Rays.
/// \param datetimeSunInfo Map holding sun and shadow information for current triangle.
///
void RayTraceTriangles(const std::string& filepath, const citygml::CityObjectsType& fileType, const std::string& cityObjId,
                       RayCollection& rayColl, std::map<int,bool>& datetimeSunInfo)
{
    //Get the triangle list of files matching intersected AABB
    TriangleList* trianglesTemp;

    trianglesTemp = BuildTriangleList(filepath,fileType, cityObjId, rayColl.rays.at(0)->ori.z);

    //Perform raytracing
    std::vector<Hit*>* tmpHits = RayTracing(trianglesTemp,rayColl.rays, true);

    for(Hit* h : *tmpHits)
    {
        datetimeSunInfo[h->ray.id] = false;
    }

    //Delete triangles
    delete trianglesTemp;

    //Delete hits
    for(unsigned int i = 0 ; i < tmpHits->size() ; ++i)
        delete tmpHits->at(i);

    delete tmpHits;
}

///
/// \brief writeInLogFile Print a text in an output txt file.
/// \param filepath path to the output file.
/// \param text text to print.
///
void writeInLogFile(const std::string& filepath, const std::string& text )
{
    std::ofstream logfile;
    logfile.open(filepath, std::ofstream::app);

    logfile << text << std::endl;
}

void SunlightDetection(std::string fileDir, std::vector<FileInfo*> filenames, std::string sunpathFile, std::string startDate, std::string endDate, QString outputDir)
{

    QTime time;
    time.start();

    std::cout << "Sunlight Calculation started." << std::endl;

    //Create output folders
    createOutputFolders(outputDir);

    //Log file
    std::string logFilePath = outputDir.toStdString() + "/SunlightOutput/logFile.txt";

    //Convert dates to integer
    int iStartDate = encodeDateTime(startDate,0);
    int iEndDate = encodeDateTime(endDate,23);

    // *** Compute sun's beam direction from sunpathFile and associate them to an hour encoded as an int. *** //
    std::map<int,TVec3d> SunsBeamsDir = loadSunpathFile(sunpathFile, iStartDate, iEndDate);

    // *** Build datetime_sunnyMap : result map associating a datetime to sunny info *** //
    //This map is created once as the sun beams are always the same in one simulation and will be associated with each triangle
    std::map<int,bool> datetime_sunnyMap;

    for(auto const& beamdir : SunsBeamsDir) //For all sun beams
    {
        if(beamdir.second == TVec3d(0.0,0.0,0.0)) //If beam direction is nul, i.e. sun is down
            datetime_sunnyMap[beamdir.first] = false; //sunny = false
        else
            datetime_sunnyMap[beamdir.first] = true; // sunny = true
    }


    // *** Load AABB of all files *** //
    AABBCollection boxes = LoadLayersAABBs(fileDir);

    //Concatenate buildingAABB and terrainAABB
    std::vector<AABB> building_terrainBB;

    building_terrainBB.reserve(boxes.building.size() + boxes.terrain.size()); //preallocate memory
    building_terrainBB.insert( building_terrainBB.end(), boxes.building.begin(), boxes.building.end() ); // insert building AABB
    building_terrainBB.insert( building_terrainBB.end(), boxes.terrain.begin(), boxes.terrain.end() ); // insert terrain AABB


    // *** Load files to analyse *** //

    unsigned int cpt_files = 1;
    double time_tot = 0.0;

    for(FileInfo* f : filenames) //Loop through files
    {
        std::cout << "===================================================" << std::endl;
        std::cout << "Computation of file " << f->WithPrevFolderAndGMLExtension() << "..." << std::endl;
        std::cout << "===================================================" << std::endl;

        //Log file
        std::string text = "File " + f->WithPrevFolderAndGMLExtension();
        writeInLogFile(logFilePath, text);

        //Load TriangleList of file to compute sunlight for
        TriangleList* trianglesfile;

        if(f->m_type == fileType::_BATI)
            trianglesfile = BuildTriangleList(f->m_filepath,citygml::CityObjectsType::COT_Building);
        else if(f->m_type == fileType::_MNT)
            trianglesfile = BuildTriangleList(f->m_filepath,citygml::CityObjectsType::COT_TINRelief);
        else
            trianglesfile = new TriangleList();

        //Log file
        text = "Triangles Number : " + std::to_string(trianglesfile->triangles.size());
        writeInLogFile(logFilePath, text);

        //Create csv file where results will be written
        createFileFolder(f, outputDir);

        int cpt_tri = 1;//output print purpose

        for(Triangle* t : trianglesfile->triangles) //Loop through each triangle
        {
            std::cout << "Triangle " << cpt_tri << " of " << trianglesfile->triangles.size() << "..." << std::endl;

            //Initialize sunlight Info results
            std::map<int,bool> datetimeSunInfo = datetime_sunnyMap;

            //Compute Barycenter of triangle
            TVec3d barycenter = TVec3d();
            barycenter.x = (t->a.x + t->b.x + t->c.x) / 3;
            barycenter.y = (t->a.y + t->b.y + t->c.y) / 3;
            barycenter.z = (t->a.z + t->b.z + t->c.z) / 3;

            //Create rayBoxCollection (All the rays for this triangle)
            RayBoxCollection* raysboxes = new RayBoxCollection();

            for(auto const& beamdir : SunsBeamsDir)
            {
                //If direction is null (ie sun is too low) leave triangle in the shadow and go to next iteration
                if(beamdir.second == TVec3d(0.0,0.0,0.0))
                    continue;

                //if triangle is not oriented towards the sun, it is in the shadow
                if(t->GetNormal().dot(beamdir.second) < 0.0)
                {
                    datetimeSunInfo[beamdir.first] = false;
                    continue;
                }

                //Add an offset for raytracing. Without this offset, origin of the ray might be behind the barycenter,
                //which will result in a collision between the ray its origin triangle
                TVec3d tmpBarycenter = TVec3d(0.0,0.0,0.0);
                tmpBarycenter.x = barycenter.x + 0.01f*beamdir.second.x;
                tmpBarycenter.y = barycenter.y + 0.01f*beamdir.second.y;
                tmpBarycenter.z = barycenter.z + 0.01f*beamdir.second.z;

                //Add ray to list
                RayBox* raybox = new RayBox(tmpBarycenter,beamdir.second,beamdir.first);
                raysboxes->raysBB.push_back(raybox);
            }

            //Setup and get the file's boxes in the right intersection order
            std::queue<RayBoxHit> fileOrder = SetupFileOrder(building_terrainBB,raysboxes);

            //While we have boxes, files
            while(fileOrder.size() != 0)
            {
                //Get current AABB hit (starting from the closest)
                RayBoxHit myRayBoxHit = fileOrder.front();
                std::string fileName_boxhit = myRayBoxHit.box.name;
                fileOrder.pop();

                //Create file information variable
                std::string path_boxhit = fileDir + fileName_boxhit;
                FileInfo fBoxHit = FileInfo(path_boxhit);

                std::string cityObjId = "";

                if(fBoxHit.m_type == fileType::_BATI) //If _BATI, Multi-Résolution
                {
                    //Create RayBoxes
                    RayBoxCollection* rayboxBuilding = new RayBoxCollection();

                    rayboxBuilding = CreateRayBoxCollection(fileName_boxhit, raysboxes, datetimeSunInfo);

                    //Clear current boxes associated to rays because they correspond to AABB of tiles.
                    //We will now recompute the boxes associated to rays using one level deeper AABB (Building AABB)
                    for(RayBox* rb : rayboxBuilding->raysBB)
                        rb->boxes.clear();

                    //Load Building AABB (B_AABB)
                    int extensionPos = fBoxHit.m_filepath.find(".gml");
                    std::string path_B_AABB = fBoxHit.m_filepath.substr(0, extensionPos) + "_Building_AABB.dat";

                    std::vector<AABB> B_AABB = LoadAABBFile(path_B_AABB);

                    //Setup AABB (The sorting is not essential here, we could just go "inside" an AABB when an intersection is found).
                    std::queue<RayBoxHit> B_AABB_Order = SetupFileOrder(B_AABB, rayboxBuilding);

                    while(B_AABB_Order.size() != 0) //For each intersected AABB
                    {
                        //Get current AABB hit (starting from the closest)
                        RayBoxHit currentBuildRayBoxHit = B_AABB_Order.front();
                        std::string fileName_BuildBoxhit = currentBuildRayBoxHit.box.name;
                        B_AABB_Order.pop();

                        std::string cityObjId = fileName_BuildBoxhit; //CityObj is the name of the Building intersected

                        RayCollection raysTemp = CreateRays(fileName_BuildBoxhit, rayboxBuilding, datetimeSunInfo); //Create Rays

                        //If no rays, continue
                        if(raysTemp.rays.size() == 0)
                        {
                            raysTemp.rays.clear();
                            continue;
                        }

                        //Raytracing on current building (thanks to cityObjId)
                        RayTraceTriangles(fBoxHit.m_filepath, citygml::CityObjectsType::COT_Building, cityObjId, raysTemp, datetimeSunInfo);

                        raysTemp.rays.clear();

                    }

                    delete rayboxBuilding;

                }
                else if(fBoxHit.m_type == fileType::_MNT) //If _MNT, no multirésolution
                {
                    RayCollection raysTemp = CreateRays(fileName_boxhit, raysboxes, datetimeSunInfo);

                    if(raysTemp.rays.size() == 0)
                    {
                        raysTemp.rays.clear();
                        continue;
                    }

                    RayTraceTriangles(fBoxHit.m_filepath, citygml::CityObjectsType::COT_TINRelief, cityObjId, raysTemp, datetimeSunInfo);

                    //Clear rays
                    raysTemp.rays.clear();
                }


            }

            exportLightningToCSV(datetimeSunInfo,t,f, iStartDate, iEndDate, outputDir); //Export result for this triangle in csv files

            //Delete RayBoxes
            delete raysboxes;

            ++cpt_tri;
        }

        //Delete TriangleList
        delete trianglesfile;

        std::cout << "===================================================" << std::endl;
        std::cout << "file " << cpt_files << " of " << filenames.size() << " done in : " << static_cast<double>(time.elapsed())/1000.0 << "s" << std::endl;
        std::cout << "===================================================" << std::endl;

        //Log file
        text = "Computation time : " + std::to_string(static_cast<double>(time.elapsed())/1000.0) + " s";
        writeInLogFile(logFilePath, text);
        writeInLogFile(logFilePath, ""); //Skip one line

        time_tot += static_cast<double>(time.elapsed())/1000.0;
        time.restart();
        ++cpt_files;
    }

    //Delete files info
    for(unsigned int i = 0 ; i < filenames.size() ; ++i)
        delete filenames[i];

    std::cout << "Total time : " << time_tot << "s" << std::endl;

}
