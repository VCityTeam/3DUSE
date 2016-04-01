#include "SunlightDetection.hpp"

#include <fstream>
#include <queue>

#include "AABB.hpp"
#include "Triangle.hpp"
#include "raytracing/RayTracing.hpp"
#include "raytracing/Hit.hpp"
#include "src/core/RayBox.hpp"
#include "quaternion.hpp"
#include "core/dateTime.hpp"

#include <QDir>


//For displaySun Function, to be removed
//#include <osg/Texture2D>
//#include <osg/Billboard>
//#include <osgDB/Registry>
//#include "src/core/application.hpp"
//#include "src/gui/applicationGui.hpp"


////Partly taken from http://howardhinnant.github.io/date_algorithms.html
/////
///// \brief encodeDateTime Compute number of days since civil 1970-01-01.  Negative values indicate days prior to 1970-01-01.
///// \param y year
///// \param m month
///// \param d day
///// \param h hour
///// \return Number of days since civil 1970-01-01.  Negative values indicate days prior to 1970-01-01.
/////
//int encodeDateTime(int y, int m, int d, int h)
//{
//    //Compute Date value : Compute number of days since civil (1970-01-01)
//    y -= m <= 2;

//    int era = (y >= 0 ? y : y-399) / 400;
//    unsigned int yoe = static_cast<unsigned>(y - era * 400);      // [0, 399] yoe = years of era
//    unsigned int doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1;  // [0, 365] doy = days of year
//    unsigned int doe = yoe * 365 + yoe/4 - yoe/100 + doy;         // [0, 146096] doe = days of era

//    int date = era * 146097 + static_cast<int>(doe) - 719468;

//    //Add time
//    int datetime = date * 24 + h;

//    return datetime;
//}

////Partly taken from http://howardhinnant.github.io/date_algorithms.html
/////
///// \brief decodeDateTime
///// \param dDateTime
///// \return
/////
//std::string decodeDateTime(int dateTime)
//{
//    std::string sDateTime = "";

//    //Gets time
//    int time = dateTime % 24;

//    //"Extract" time info from dateTime
//    dateTime = (dateTime - time) / 24;

//    dateTime += 719468; // shift the epoch from 1970-01-01 to 0000-03-01
//    int era = (dateTime >= 0 ? dateTime : dateTime - 146096) / 146097;

//    unsigned int doe = static_cast<unsigned>(dateTime - era * 146097);          // [0, 146096] doe = days of year
//    unsigned int yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;  // [0, 399] yoe = years of era
//    int y = static_cast<int>(yoe) + era * 400; // y = year

//    unsigned int doy = doe - (365*yoe + yoe/4 - yoe/100);                // [0, 365] doy = days of year
//    unsigned int mp = (5*doy + 2)/153;                                   // [0, 11] mp = m' = month number starting from March

//    unsigned int d = doy - (153*mp+2)/5 + 1;                             // [1, 31] d = day
//    unsigned int m = mp + (mp < 10 ? 3 : -9);                            // [1, 12] m = month
//    y = y + (m <= 2);


//    //Convert to string
//    std::string sMonth = "";
//    if(m < 10)
//        sMonth = "0";
//    sMonth += std::to_string(m);


//    std::string sDay = "";
//    if(d < 10)
//        sDay = "0";
//    sDay += std::to_string(d);

//    std::string sHour = "";
//    if(time < 10)
//        sHour = "0";
//    sHour += std::to_string(time);

//    sDateTime = sDay + sMonth + std::to_string(y) + ":" + sHour + "00";

//    return sDateTime;
//}


///
/// \brief SetupTileOrder create a std::queue queueing the tiles intersected by a given RayBoxCollection and sorted by intersection distance
/// \param boxes Vector of tiles' bounding boxes to check intersection with
/// \param rays Rays to throw to see if they intersect with boxes
/// \return a std::queue of RayboxHit (struct which holds a bounding box (AABB) and minDistance intersection) sorted by intersection distance (min first)
///
std::queue<RayBoxHit> SetupTileOrder(std::vector<AABB> boxes,RayBoxCollection* rays)
{
    std::map<std::string,int> boxToMaxOrder;//We keep record the maximum order the box has be traverse
    std::map<std::string,RayBoxHit> boxToRayBoxHit;//We keep record of the smallest rayboxhit of a box /// MultiResolution

    for(unsigned int i = 0; i < boxes.size(); i++)
    {
        boxToMaxOrder[boxes[i].name] = -1;
    }
    //For each rays
    for(unsigned int i = 0; i < rays->raysBB.size(); i++)
    {
        RayBox* rayBox = rays->raysBB[i];
        rayBox->boxes.clear();

        //For each boxes we check if the ray intersect the box and store it in the box list of the array
        for(unsigned int j = 0; j < boxes.size(); j++)
        {
            float hit0,hit1;
            if(rayBox->Intersect(boxes[j],&hit0,&hit1))
            {
                RayBoxHit hTemp;
                hTemp.box = boxes[j];
                hTemp.minDistance = hit0;
                rayBox->boxes.push_back(hTemp); //On remplit la liste des boîtes intersectées par ce rayon
            }
        }

        std::sort(rayBox->boxes.begin(),rayBox->boxes.end());//Sort the boxes depending on the intersection distance

        for(int j = 0; j < rayBox->boxes.size(); j++)//We update the order of each boxes
        {
            int current = boxToMaxOrder[rayBox->boxes[j].box.name];
            boxToMaxOrder[rayBox->boxes[j].box.name] = std::max(j,current);

            if(boxToRayBoxHit.find(rayBox->boxes[j].box.name) != boxToRayBoxHit.end())//= Si rayBox->boxes[j].box.name existe déjà dans boxToRayBoxHit/// MultiResolution
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
    std::queue<RayBoxHit> tileOrder;

    for(BoxOrder& bo : boxesOrder)
        tileOrder.push(boxToRayBoxHit[bo.box]);

    return tileOrder;
}


///
/// \brief initExportFile
/// \param tilename
///
void initExportFile(std::string tilename)
{
    QDir outputDir("./SunlightOutput/");
    if(!outputDir.exists("./SunlightOutput/"))
        outputDir.mkpath(outputDir.absolutePath());


    QString sOutputDir = "./SunlightOutput/";

    if(tilename.find("_BATI") != std::string::npos)
        sOutputDir =  "./SunlightOutput/_BATI/";
    else if(tilename.find("_MNT") != std::string::npos)
        sOutputDir =  "./SunlightOutput/_MNT/";


    QDir outputDir2(sOutputDir);
    if(!outputDir2.exists(sOutputDir))
        outputDir2.mkpath(outputDir2.absolutePath());

    std::ofstream ofs;
    ofs.open (sOutputDir.toStdString() + tilename + ".csv", std::ofstream::app);

    ofs << "TileName : " << tilename << std::endl;
    ofs << "DateTime;PolygoneId;Sunny" << std::endl;

    ofs.close();
}

///
/// \brief exportLightningToCSV Export Sunlight informations for a given tile into a csv file.
/// \param vSunInfo vector holding informations about sunlight for a given triangle.
/// \param tilename Name of the tile.
///
void exportLightningToCSV(std::map<int,bool> sunInfo, Triangle* t, std::string tilename)
{
    std::ofstream ofs;
    ofs.open ("./SunlightOutput/" + tilename + ".csv", std::ofstream::app);

    for(auto ySI : sunInfo)
    {
        std::string dateTime = decodeDateTime(ySI.first);
        ofs << dateTime << ";" << t->polygonId << ";" << ySI.second << std::endl;
        // iterator->first = key
        // iterator->second = value
    }

    ofs.close();
}


void SunlightDetection()
{
    QTime time;
    time.start();

    std::cout << "Sunlight Calculation started." << std::endl;

    // ************* Lightning Measure ******************************//

    std::string date = "2016-08-10";

    //Date
    int year = 2016; int month = 8; int day = 10;

    std::vector<double> elevationAngle;
    std::vector<double> azimutAngle;

    bool found = false;

    // *** CSV Load
    std::ifstream file( "/home/vincent/Documents/VCity_Project/Data/AnnualSunPath_Lyon.csv" );
    std::string line;

    while(std::getline(file,line) && found == false) // For all lines of csv file
    {
        std::stringstream  lineStream(line);
        std::string        cell;

        std::getline(lineStream,cell,';'); //Get first cell of line

        if(cell.find(date) != std::string::npos) //If it's the right line, get all cells
        {
            found = true;
            std::getline(lineStream,cell,';'); //jump second cell of line

            while(std::getline(lineStream,cell,';'))
            {
                //Add azimuthAngle (corresponding to current hour)
                if(cell == "--" || cell == "")
                    azimutAngle.push_back(0.0);
                else
                    azimutAngle.push_back(std::stod(cell) * M_PI / 180); //Conversion to radian

                //Get next cell (azimuth angle of current hour)
                std::getline(lineStream,cell,';');

                //Add Elevation angle
                if(cell == "--" || cell == "")
                    elevationAngle.push_back(0.0);
                else
                    elevationAngle.push_back(std::stod(cell) * M_PI / 180);

            }
        }
    }

    //*** Computation of beam directions
    // North : y axis
    // South : -y axis
    // Est : x axis
    // Ouest : -x axis

    TVec3d origin = TVec3d(1843927.29, 5173886.65, 0.0); //Lambert 93 Coordinates
    TVec3d sunPos = origin + TVec3d(0.0,60000.0,0.0); //SunPos is the first position of the sun (north) from which the angles are expressed
    TVec3d newSunPos = origin + TVec3d(0.0,60000.0,0.0); //Lambert 93 coordinates

    TVec3d ARotAxis = TVec3d(0.0,0.0,1.0);
    TVec3d ERotAxis = TVec3d(-1.0,0.0,0.0);

    std::vector<TVec3d> beamsDirections;

    for(unsigned int i = 0 ; i < elevationAngle.size() ; ++i)
    {
        //Compute new position of sun
        if (elevationAngle.at(i) <= 0.01 || azimutAngle.at(i) <= 0.01) //if sun to low (angle < 1°), go to next iteration
        {
            beamsDirections.push_back(TVec3d(0.0,0.0,0.0)); //Add nul beam direction
            continue;
        }

        //Azimut rotation quaternion
        citygml::quaternion qA = citygml::quaternion();
        qA.set_axis_angle(ARotAxis,azimutAngle.at(i));

        //Elevation rotation quaternion
        citygml::quaternion qE = citygml::quaternion();
        qE.set_axis_angle(ERotAxis,elevationAngle.at(i));

        //Total rotation quaternion
        citygml::quaternion q = qE*qA;

        sunPos = sunPos - origin;
        newSunPos = q*sunPos;
        newSunPos = newSunPos + origin;
        sunPos = sunPos + origin;

        //Display Sun
        //DisplaySun(newSunPos);

        //Compute sun beams direction
        TVec3d tmpDirection = (newSunPos - origin);
        beamsDirections.push_back(tmpDirection.normal());
    }

    //********* Build year map ************//

    std::map<int,bool> yearMap;

    for(int hour = 0 ; hour < 24 ; ++hour)
    {
        int datetime = encodeDateTime(year,month,day,hour);

        if(beamsDirections.at(hour) == TVec3d(0.0,0.0,0.0))
            yearMap[datetime] = false;
        else
            yearMap[datetime] = true;
    }


    //***** MultiTileAnalysis

    //TODO: Load list of all tiles from specified folder and loop on this list

    //vcity::Tile* tile = new vcity::Tile("/home/vincent/Documents/VCity_Project/Data/Tuiles/_BATI/3670_10382.gml");

    //Load AABB
    std::string dirTile = "/home/vincent/Documents/VCity_Project/Data/Tuiles/";

    AABBCollection boxes = LoadAABB(dirTile);

    //Concatenate buildingAABB and terrainAABB
    std::vector<AABB> building_terrainBB;

    building_terrainBB.reserve(boxes.building.size() + boxes.terrain.size()); //preallocate memory
    building_terrainBB.insert( building_terrainBB.end(), boxes.building.begin(), boxes.building.end() ); // insert building AABB
    building_terrainBB.insert( building_terrainBB.end(), boxes.terrain.begin(), boxes.terrain.end() ); // insert terrain AABB

    //Create vector of tiles to analyse
    std::vector<std::string> tiles;

    //Test rapide
    tiles.push_back("_BATI/3670_10383");
    tiles.push_back("_BATI/3670_10382");
    tiles.push_back("_MNT/3670_10383");
    tiles.push_back("_MNT/3670_10382");

    //test fred
//    tiles.push_back("3674_10346");
//    tiles.push_back("3674_10347");
//    tiles.push_back("3674_10348");

//    tiles.push_back("3675_10346");
//    tiles.push_back("3675_10347");
//    tiles.push_back("3675_10348");

//    tiles.push_back("3676_10346");
//    tiles.push_back("3676_10347");
//    tiles.push_back("3676_10348");

    //test moi
//    tiles.push_back("3682_10350");
//    tiles.push_back("3682_10351");
//    tiles.push_back("3682_10352");

//    tiles.push_back("3683_10350");
//    tiles.push_back("3683_10351");
//    tiles.push_back("3683_10352");

//    tiles.push_back("3684_10350");
//    tiles.push_back("3684_10351");
//    tiles.push_back("3684_10352");


    //test fred
//    tiles.push_back("3666_10346");
//    tiles.push_back("3667_10346");
//    tiles.push_back("3666_10347");
//    tiles.push_back("3667_10347");

    unsigned int cpt_tiles = 1;
    int time_tot = 0;

    for(std::string tilename : tiles)
    {
        std::cout << "===================================================" << std::endl;
        std::cout << "Computation of tile " << tilename << "..." << std::endl;
        std::cout << "===================================================" << std::endl;

        //Load TriangleList of tile to treat
        std::string path = "/home/vincent/Documents/VCity_Project/Data/Tuiles/" + tilename + ".gml";

        TriangleList* trianglesTile;

        if(tilename.find("_BATI") != std::string::npos)
        {
            trianglesTile = BuildTriangleList(path,citygml::CityObjectsType::COT_Building);
        }
        else if(tilename.find("_MNT") != std::string::npos)
        {
            trianglesTile = BuildTriangleList(path,citygml::CityObjectsType::COT_TINRelief);
        }
        else
        {
            trianglesTile = new TriangleList();
        }

        int cpt_tri = 1;

        //Create csv file where results are exported
        initExportFile(tilename);

        for(Triangle* t : trianglesTile->triangles) //Loop through each triangle
        {
            std::cout << "Triangle " << cpt_tri << " of " << trianglesTile->triangles.size() << "..." << std::endl;

            //Initialize sunInfo
            std::map<int,bool> yearSunInfo = yearMap;

            //Compute Barycenter
            TVec3d barycenter = TVec3d();
            barycenter.x = (t->a.x + t->b.x + t->c.x) / 3;
            barycenter.y = (t->a.y + t->b.y + t->c.y) / 3;
            barycenter.z = (t->a.z + t->b.z + t->c.z) / 3;

            //Create rayCollection (All the rays for this triangle)
            RayBoxCollection* raysboxes = new RayBoxCollection();

            int hour = 0;
            for(TVec3d beamDir : beamsDirections)
            {
                //If direction is null (ie sun is too low) make every triangle in the shadow and goes to next iteration
                if(beamDir == TVec3d(0.0,0.0,0.0))
                {
                    hour++;
                    continue;
                }

                //Id
                int id = encodeDateTime(year,month,day,hour);

                //Add an offset for raytracing
                TVec3d tmpBarycenter = TVec3d(0.0,0.0,0.0);
                tmpBarycenter.x = barycenter.x + 0.01f*beamDir.x;
                tmpBarycenter.y = barycenter.y + 0.01f*beamDir.y;
                tmpBarycenter.z = barycenter.z + 0.01f*beamDir.z;

                RayBox* raybox = new RayBox(tmpBarycenter,beamDir,id);
                raysboxes->raysBB.push_back(raybox);

                hour++;
            }

            //Setup and get the tile's boxes in the right intersection order
            std::queue<RayBoxHit> tileOrder = SetupTileOrder(building_terrainBB,raysboxes);

            //While we have boxes, tiles
            while(tileOrder.size() != 0)
            {
                RayBoxHit myRayBoxHit = tileOrder.front();
                std::string tileName = myRayBoxHit.box.name;
                tileOrder.pop();

                RayCollection raysTemp;//Not all rays intersect the box

                //We get only the rays that intersect the box
                for(unsigned int i = 0; i < raysboxes->raysBB.size(); i++)
                {
                    RayBox* raybox = raysboxes->raysBB[i];//Get the ray

                    bool found = false;

                    for(RayBoxHit& rbh : raybox->boxes)//Go through all the box that the ray intersect to see if the current box is one of them
                        found = found || rbh.box.name == tileName;

                    if(found)
                    {
                        raysTemp.rays.push_back(raybox);
                    }
                }

                if(raysTemp.rays.size() == 0)
                {
                    raysTemp.rays.clear();
                    continue;
                }

                //Load triangles and perform analysis
                std::string path = dirTile + tileName;

                //Get the triangle list
                TriangleList* trianglesTemp;

                if(tileName.find("_BATI") != std::string::npos)
                {
                    trianglesTemp = BuildTriangleList(path,citygml::CityObjectsType::COT_Building);
                }
                else if(tileName.find("_MNT") != std::string::npos)
                {
                    trianglesTemp = BuildTriangleList(path,citygml::CityObjectsType::COT_TINRelief);
                }

                std::vector<Hit*>* tmpHits = RayTracing(trianglesTemp,raysTemp.rays);

                for(Hit* h : *tmpHits)
                {
                    yearSunInfo[h->ray.id] = false;
                }

                //Delete triangles
                delete trianglesTemp;

                //Clear rays
                raysTemp.rays.clear();

                //Delete hits
                for(unsigned int i = 0 ; i < tmpHits->size() ; ++i)
                    delete tmpHits->at(i);

                delete tmpHits;
            }

            exportLightningToCSV(yearSunInfo,t,tilename);

            //Delete RayBoxes
            delete raysboxes;

            ++cpt_tri;
        }

        //Delete TriangleList
        delete trianglesTile;

        std::cout << "===================================================" << std::endl;
        std::cout << "Tile " << cpt_tiles << " of " << tiles.size() << " done in : " << (double)time.elapsed()/60000.0 << " min." << std::endl;
        std::cout << "===================================================" << std::endl;

        time_tot += time.elapsed();
        time.restart();
        ++cpt_tiles;

    }

    std::cout << "Total time : " << (double)time_tot/60000.0 << " min" << std::endl;

}



//Function to visualize sun in 3D-USE to verify results, to be removed
//void DisplaySun(TVec3d sunPos)
//{
//    //** Display sun

//   osg::Group* rootNode = new osg::Group();
//   osg::Texture2D *OldLyonTexture = new osg::Texture2D;
//   OldLyonTexture->setImage(osgDB::readImageFile("/home/vincent/Documents/VCity_Project/Data/soleil.jpg"));


//   osg::StateSet* bbState = new osg::StateSet;
//   bbState->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
//   bbState->setTextureAttributeAndModes(0, OldLyonTexture, osg::StateAttribute::ON );


//   float width = 3000.0f;
//   float height = 1500.0f;


//   osg::Geometry* bbQuad = new osg::Geometry;

//   osg::Vec3Array* bbVerts = new osg::Vec3Array(4);
//   (*bbVerts)[0] = osg::Vec3(-width/2.0f, 0, 0);
//   (*bbVerts)[1] = osg::Vec3( width/2.0f, 0, 0);
//   (*bbVerts)[2] = osg::Vec3( width/2.0f, 0, height);
//   (*bbVerts)[3] = osg::Vec3(-width/2.0f, 0, height);

//   bbQuad->setVertexArray(bbVerts);

//   osg::Vec2Array* bbTexCoords = new osg::Vec2Array(4);
//   (*bbTexCoords)[0].set(0.0f,0.0f);
//   (*bbTexCoords)[1].set(1.0f,0.0f);
//   (*bbTexCoords)[2].set(1.0f,1.0f);
//   (*bbTexCoords)[3].set(0.0f,1.0f);

//   bbQuad->setTexCoordArray(0,bbTexCoords);

//   bbQuad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

//   osg::Vec4Array* colorArray = new osg::Vec4Array;
//   colorArray->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); // white, fully opaque

//   // Use the index array to associate the first entry in our index array with all
//   // of the vertices.
//   bbQuad->setColorArray( colorArray);

//   bbQuad->setColorBinding(osg::Geometry::BIND_OVERALL);

//   bbQuad->setStateSet(bbState);


//   osg::Billboard* TestBillBoard = new osg::Billboard();
//   rootNode->addChild(TestBillBoard);

//   TestBillBoard->setMode(osg::Billboard::AXIAL_ROT);
//   TestBillBoard->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
//   TestBillBoard->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));


//   osg::Drawable* bbDrawable = bbQuad;

//   sunPos = sunPos - vcity::app().getSettings().getDataProfile().m_offset;

//   osg::Vec3d pos = osg::Vec3d(sunPos.x,sunPos.y,sunPos.z);

//   TestBillBoard->addDrawable(bbDrawable , pos);

//   vcity::URI uriLayer = vcity::app().getScene().getDefaultLayer("LayerShp")->getURI();

//   appGui().getOsgScene()->addShpNode(uriLayer, rootNode);
//}

