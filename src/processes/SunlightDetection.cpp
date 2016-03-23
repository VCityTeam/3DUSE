#include "SunlightDetection.hpp"

#include <fstream>
#include <queue>

#include "AABB.hpp"
#include "Triangle.hpp"
#include "raytracing/RayTracing.hpp"
#include "raytracing/Hit.hpp"
#include "src/core/RayBox.hpp"
#include "quaternion.hpp"

//For displaySun Function, to be removed
#include <osg/Texture2D>
#include <osg/Billboard>
#include <osgDB/Registry>
#include "src/core/application.hpp"
#include "src/gui/applicationGui.hpp"

///
/// \brief buildYearMap builds a std::map mapping every datetime of a year to a boolean valeur representing sunlight.
/// \param year The year for which the map should be build
/// \return A std::map mapping datetime (ddmmyyyy:hhmm) for a given year to a boolean value representing sunlight at this datetime (true = sunny, false = shadowed)
///
std::map<std::string,bool> buildYearMap(int year)
{
    std::map<std::string,bool> yearMap;

    std::string year_str = std::to_string(year);

    for(int month = 1; month <= 12 ; ++month)
    {
        std::string month_str;
        if (month < 10)
            month_str = "0" + std::to_string(month);
        else
            month_str = std::to_string(month);

        for(int day = 1; day <= 31 ; ++day)
        {
            //Special case for February
            if (month == 2 && day == 29)
            {
                //If year is not bissextile, exit loop
                if(!((year % 4 == 0 && year % 100 != 0) || year % 400 == 0))
                    break;
            }
            if (month == 2 && day == 30)
                break;

            //Month with 30 days
            if((month == 4 || month == 6 || month == 9 || month == 11) && day == 31)
                break;

            std::string day_str;
            if (day < 10)
                day_str = "0" + std::to_string(day);
            else
                day_str = std::to_string(day);

            for(int hour = 0 ; hour < 24 ; ++hour)
            {
                std::string hour_str;
                if (hour < 10)
                    hour_str = "0" + std::to_string(hour) + "00";
                else
                    hour_str = std::to_string(hour) + "00";

                std::string code_str = day_str + month_str + year_str + ":" + hour_str;
                yearMap[code_str] = true;
            }
        }
    }

    return yearMap;
}


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
/// \brief exportLightningToCSV Export Sunlight informations for a given tile into a csv file.
/// \param vSunInfo vector holding informations about sunlight for a given triangle.
/// \param tilename Name of the tile.
///
void exportLightningToCSV(std::vector<TriangleLightInfo*> vSunInfo, std::string tilename)
{
    //To create directory, use QDir.mkdir("name")
    //Delete file and recreate it or clear it
    //Add URI

    std::ofstream ofs;
    ofs.open ("./" + tilename + "_sunlight.csv", std::ofstream::out);



    ofs << "TileName : " << tilename << std::endl;
    ofs << "DateTime;PolygoneId;Sunny" << std::endl;

    for(TriangleLightInfo* tli : vSunInfo)
    {
        for(auto ySI : tli->yearSunInfo)
        {
            ofs << ySI.first << ";" << tli->triangle->polygonId << ";" << ySI.second << std::endl;
            // iterator->first = key
            // iterator->second = value
        }
    }

    ofs.close();

    std::cout << "file created" << std::endl;

}

//Function to visualize sun in 3D-USE to verify results, to be removed
void DisplaySun(TVec3d sunPos)
{
    //** Display sun

   osg::Group* rootNode = new osg::Group();
   osg::Texture2D *OldLyonTexture = new osg::Texture2D;
   OldLyonTexture->setImage(osgDB::readImageFile("/home/vincent/Documents/VCity_Project/Data/soleil.jpg"));


   osg::StateSet* bbState = new osg::StateSet;
   bbState->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
   bbState->setTextureAttributeAndModes(0, OldLyonTexture, osg::StateAttribute::ON );


   float width = 3000.0f;
   float height = 1500.0f;


   osg::Geometry* bbQuad = new osg::Geometry;

   osg::Vec3Array* bbVerts = new osg::Vec3Array(4);
   (*bbVerts)[0] = osg::Vec3(-width/2.0f, 0, 0);
   (*bbVerts)[1] = osg::Vec3( width/2.0f, 0, 0);
   (*bbVerts)[2] = osg::Vec3( width/2.0f, 0, height);
   (*bbVerts)[3] = osg::Vec3(-width/2.0f, 0, height);

   bbQuad->setVertexArray(bbVerts);

   osg::Vec2Array* bbTexCoords = new osg::Vec2Array(4);
   (*bbTexCoords)[0].set(0.0f,0.0f);
   (*bbTexCoords)[1].set(1.0f,0.0f);
   (*bbTexCoords)[2].set(1.0f,1.0f);
   (*bbTexCoords)[3].set(0.0f,1.0f);

   bbQuad->setTexCoordArray(0,bbTexCoords);

   bbQuad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

   osg::Vec4Array* colorArray = new osg::Vec4Array;
   colorArray->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); // white, fully opaque

   // Use the index array to associate the first entry in our index array with all
   // of the vertices.
   bbQuad->setColorArray( colorArray);

   bbQuad->setColorBinding(osg::Geometry::BIND_OVERALL);

   bbQuad->setStateSet(bbState);


   osg::Billboard* TestBillBoard = new osg::Billboard();
   rootNode->addChild(TestBillBoard);

   TestBillBoard->setMode(osg::Billboard::AXIAL_ROT);
   TestBillBoard->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
   TestBillBoard->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));


   osg::Drawable* bbDrawable = bbQuad;

   sunPos = sunPos - vcity::app().getSettings().getDataProfile().m_offset;

   osg::Vec3d pos = osg::Vec3d(sunPos.x,sunPos.y,sunPos.z);

   TestBillBoard->addDrawable(bbDrawable , pos);

   vcity::URI uriLayer = vcity::app().getScene().getDefaultLayer("LayerShp")->getURI();

   appGui().getOsgScene()->addShpNode(uriLayer, rootNode);
}


void SunlightDetection()
{
    QTime time;
    time.start();

    // ************* Lightning Measure ******************************//

    std::string date = "2016-08-10";
    std::string tilename = "3670_10382";

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

    for(int i = 0 ; i < elevationAngle.size() ; ++i)
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
        DisplaySun(newSunPos);

        //Compute sun's beams direction
        TVec3d tmpDirection = (newSunPos - origin);
        beamsDirections.push_back(tmpDirection.normal());
    }

//    int cpt = 0;
//    for(TVec3d beamDir : beamsDirections)
//    {
//        std::cout << "BeamDir at Hour " << cpt << " : " << beamDir << std::endl ;
//        cpt++;
//    }


    //***** MultiTileAnalysis

    //TODO: Load list of all tiles from specified folder and loop on this list

    //vcity::Tile* tile = new vcity::Tile("/home/vincent/Documents/VCity_Project/Data/Tuiles/_BATI/3670_10382.gml");

    std::string path = "/home/vincent/Documents/VCity_Project/Data/Tuiles/_BATI/" + tilename + ".gml";
    std::string dirTile = "/home/vincent/Documents/VCity_Project/Data/Tuiles/";
    citygml::CityObjectsType objectType = citygml::CityObjectsType::COT_Building; //TODO:ADD Terrain

    TriangleList* trianglesTile1;
    trianglesTile1 = BuildTriangleList(path,objectType);

    AABBCollection boxes = LoadAABB(dirTile);
    std::vector<AABB> buildingBB = boxes.building; //TODO:ADD Terrain

    //Build year map
    //std::map<std::string,bool> yearMap = buildYearMap(2016);
    std::map<std::string,bool> yearMap;

    for(int hour = 0 ; hour < 24 ; ++hour)
    {
        std::string hour_str;
        if (hour < 10)
            hour_str = "0" + std::to_string(hour) + "00";
        else
            hour_str = std::to_string(hour) + "00";

        std::string code_str = "10082016:" + hour_str;

        if(beamsDirections.at(hour) == TVec3d(0.0,0.0,0.0))
        {
            yearMap[code_str] = false;
        }
        else
        {
            yearMap[code_str] = true;
        }
    }


    std::vector<TriangleLightInfo*> vSunInfoTriangle;

    for(Triangle* t : trianglesTile1->triangles) //Loop through each triangle
    {
        //Initialize sunInfo
        //t->sunInfo = yearMap;
        TriangleLightInfo* sunInfoTri = new TriangleLightInfo();
        sunInfoTri->triangle = t;
        sunInfoTri->yearSunInfo = yearMap;

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
            //Construct id
            std::string hour_str;
            if(hour<10)
                hour_str = "0"+ std::to_string(hour) + "00";
            else
                hour_str = std::to_string(hour) + "00";

            std::string id = "10082016:" + hour_str;

            //If direction is null (ie sun is too low) make every triangle in the shadow and goes to next iteration
            if(beamDir == TVec3d(0.0,0.0,0.0))
            {
                hour++;
                continue;
            }

            //Addition of an offset for raytracing
            TVec3d tmpBarycenter = TVec3d(0.0,0.0,0.0);
            tmpBarycenter.x = barycenter.x + 0.01f*beamDir.x;
            tmpBarycenter.y = barycenter.y + 0.01f*beamDir.y;
            tmpBarycenter.z = barycenter.z + 0.01f*beamDir.z;

            RayBox* raybox = new RayBox(tmpBarycenter,beamDir,id);
            raysboxes->raysBB.push_back(raybox);

            hour++;
        }

        //Setup and get the tile's boxes in the right intersection order
        std::queue<RayBoxHit> tileOrder = SetupTileOrder(buildingBB,raysboxes); //TODO: Add terrain

        //While we have boxes, tiles
        while(tileOrder.size() != 0)
        {
            RayBoxHit myRayBoxHit = tileOrder.front();
            std::string tileName = myRayBoxHit.box.name;
            tileOrder.pop();

//            tileName = tileName.substr(0, tileName.size() - 1);
//            std::cout<< "tilename : " << tileName.size() << std::endl;


//            myRayBoxHit.box.name = myRayBoxHit.box.name.substr(0, myRayBoxHit.box.name.size() - 1);
//            std::cout<< "box name : " << myRayBoxHit.box.name.size() << std::endl;

            RayCollection raysTemp;//Not all rays intersect the box
            //raysTemp.viewpoint = viewpoint;

            //We get only the rays that intersect the box
            for(unsigned int i = 0; i < raysboxes->raysBB.size(); i++)
            {
                RayBox* raybox = raysboxes->raysBB[i];//Get the ray

                //Check if triangle is sunny for this ray (i.e. this hour)
                bool sunny = sunInfoTri->yearSunInfo[raybox->id];
                //bool sunny = t->sunInfo[ray->id];
                bool found = false;

                for(RayBoxHit& rbh : raybox->boxes)//Go through all the box that the ray intersect to see if the current box is one of them
                    found = found || rbh.box.name == tileName;

                if(found && sunny)
                {
                    raysTemp.rays.push_back(raybox);
                }
            }

            if(raysTemp.rays.size() == 0)
            {
                std::cout << "Skipping." << std::endl;
                raysTemp.rays.clear();
                continue;
            }

            //Load triangles and perform analysis
            std::string path = dirTile + tileName;
            //std::string pathWithPrefix = dirTile + GetTilePrefixFromDistance(myRayBoxHit.minDistance) + tileName; /// MultiResolution

            //Get the triangle list
            TriangleList* trianglesTemp;

            trianglesTemp = BuildTriangleList(path,objectType);

            std::vector<Hit*>* tmpHits = RayTracing(trianglesTemp,raysTemp.rays);

            for(Hit* h : *tmpHits)
            {
                sunInfoTri->yearSunInfo[h->ray.id] = false;
            }

            raysTemp.rays.clear();
            delete tmpHits;

        }

        vSunInfoTriangle.push_back(sunInfoTri);

    }

    //Export to csv (one per tile)
    exportLightningToCSV(vSunInfoTriangle, tilename);

     std::cout << "Time : " << time.elapsed()/60000 << " min" << std::endl;

}

