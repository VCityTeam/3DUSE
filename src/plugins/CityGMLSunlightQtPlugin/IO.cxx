#include "IO.h"

#include "Triangle.hpp"
#include "core/dateTime.hpp"
#include "libcitygml/quaternion.hpp"

#include <QDir>
#include <fstream>


void initExportFile(FileInfo* file)
{
    //Open stream and write headers
    std::ofstream ofs;
    ofs.open ("./SunlightOutput/" + file->WithPrevFolder() + ".csv", std::ofstream::app);

    ofs << "FileName : " << file->WithPrevFolder() << std::endl;
    ofs << "DateTime;PolygoneId;Sunny" << std::endl;

    ofs.close();
}



void exportLightningToCSV(std::map<int,bool> sunInfo, Triangle* t, FileInfo* file)
{
    std::ofstream ofs;
    ofs.open ("./SunlightOutput/" + file->WithPrevFolder() + ".csv", std::ofstream::app);

    for(auto ySI : sunInfo)
    {
        std::string dateTime = decodeDateTime(ySI.first);
        ofs << dateTime << ";" << t->polygonId << ";" << ySI.second << std::endl;
        // iterator->first = key
        // iterator->second = value
    }

    ofs.close();
}


TVec3d computeBeamDir(double azimutAngle, double elevationAngle)
{

    TVec3d origin = TVec3d(1843927.29, 5173886.65, 0.0); //Lambert 93 Coordinates
    TVec3d sunPos = origin + TVec3d(0.0,60000.0,0.0); //SunPos is the first position of the sun (north) from which the angles are expressed
    TVec3d newSunPos = origin + TVec3d(0.0,60000.0,0.0); //Lambert 93 coordinates

    TVec3d ARotAxis = TVec3d(0.0,0.0,1.0);
    TVec3d ERotAxis = TVec3d(-1.0,0.0,0.0);

    //if sun to low (angle < 1°), return nul beam direction
    if (elevationAngle <= 0.01 || azimutAngle <= 0.01)
        return TVec3d(0.0,0.0,0.0);

    //Else, compute direction

    //Azimut rotation quaternion
    citygml::quaternion qA = citygml::quaternion();
    qA.set_axis_angle(ARotAxis,azimutAngle);

    //Elevation rotation quaternion
    citygml::quaternion qE = citygml::quaternion();
    qE.set_axis_angle(ERotAxis,elevationAngle);

    //Total rotation quaternion
    citygml::quaternion q = qE*qA;

    sunPos = sunPos - origin;
    newSunPos = q*sunPos;
    newSunPos = newSunPos + origin;
    sunPos = sunPos + origin;

    //Compute sun beams direction
    TVec3d tmpDirection = (newSunPos - origin);

    return tmpDirection.normal();
}



std::map<int,TVec3d> loadSunpathFile(std::string sunpathFile, std::string startDate, std::string endDate)
{
    std::map<int,TVec3d> SunBeamDir;

    std::ifstream file(sunpathFile); //Load file
    std::string line;

    int iStartDate = encodeDateTime(startDate,0);
    int iEndDate = encodeDateTime(endDate,23);

    bool found = false; //Date found
    bool exit_loop = false;

    //Skip header line
    std::getline(file,line);

    while(std::getline(file,line) && exit_loop == false) // For all lines of csv file
    {
        std::stringstream  lineStream(line);
        std::string        cell;

        std::getline(lineStream,cell,';'); //Get first cell of row

        std::string sCurrentDate = cell;
        int iCurrentDate = encodeDateTime(cell,0);

        if(iCurrentDate >= iStartDate && iCurrentDate < iEndDate)
        {
            found = true;
            std::getline(lineStream,cell,';'); //jump second cell of line

            int hour = 0;
            while(std::getline(lineStream,cell,';'))
            {
                //Add azimuthAngle
                double azimutAngle;

                if(cell == "--" || cell == "")
                    azimutAngle = 0.0;
                else
                    azimutAngle = std::stod(cell) * M_PI / 180; //Conversion to radian

                //Get next cell (elevation angle)
                std::getline(lineStream,cell,';');

                double elevationAngle;

                //Add Elevation angle
                if(cell == "--" || cell == "")
                    elevationAngle = 0.0;
                else
                    elevationAngle = std::stod(cell) * M_PI / 180;

                //Encode datetime
                int dateTime = encodeDateTime(sCurrentDate,hour);

                //Compute Sun's beam Direction
                SunBeamDir[dateTime] = computeBeamDir(azimutAngle, elevationAngle);

                ++hour;
            }
        }
        else if(iCurrentDate > iEndDate)
            exit_loop = true;
    }

    if(found == false)
        std::cerr << "Error while loading Annual SunPath file." << std::endl;

    return SunBeamDir;

}
