#include "IO.h"

#include "Triangle.hpp"
#include "core/dateTime.hpp"
#include "DataStructures/quaternion.hpp"

#include <QDir>
//This is not a real dependency towards osgDB/fstream, but towards fstream.
//This is a kludge to be compatible with other files including osgDB/fstream
//in order to avoid multiple definitions of stream related symbols on Windows.
#include <osgDB/fstream>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


void createOutputFolders(const QString& sOutputDir)
{
    //*** Create output folders
    QDir outputDirSunlight(sOutputDir + "SunlightOutput/");
    if(!outputDirSunlight.exists(sOutputDir + "SunlightOutput/"))
        outputDirSunlight.mkpath(outputDirSunlight.absolutePath());

    QDir outputDirBati(sOutputDir + "SunlightOutput/_BATI");
    if(!outputDirBati.exists(sOutputDir + "SunlightOutput/_BATI"))
        outputDirBati.mkpath(outputDirBati.absolutePath());

    QDir outputDirMnt(sOutputDir + "SunlightOutput/_MNT");
    if(!outputDirMnt.exists(sOutputDir + "SunlightOutput/_MNT"))
        outputDirMnt.mkpath(outputDirMnt.absolutePath());
}

void createFileFolder(FileInfo* file, const QString& sOutputDir)
{
    //Create folder corresponding to file
    QString path = sOutputDir + "SunlightOutput/" + QString::fromStdString(file->WithPrevFolder()) + "/";
    QDir outputDir(path);
    if(!outputDir.exists(path))
        outputDir.mkpath(outputDir.absolutePath());
}

void exportLightningToCSV(std::map<int,bool>& sunInfo, Triangle* t, FileInfo* file, int iStartDate, int iEndDate, QString &outputDir)
{
    int nb_days = (iEndDate - iStartDate + 1) / 24;

    for(int day_nb = 0 ; day_nb < nb_days ; ++day_nb)
    {
        //Get day as string
        int iDate = iStartDate + (day_nb*24);
        std::string datetime = decodeDateTime(iDate);
        std::string day = datetime.substr(0,datetime.find(":"));

        //Create and open file
        std::ofstream ofs;
        ofs.open (outputDir.toStdString() + "/SunlightOutput/" + file->WithPrevFolder() + "/" + day + ".csv", std::ofstream::app);

        for(int i = 0 ; i < 24 ; ++i) //For each hour
        {
            ofs << decodeDateTime(iDate + i) << ";" << t->polygonId << ";" << std::to_string(sunInfo[iDate + i]) << std::endl;
        }

        ofs.close();
    }
}

///
/// \brief computeBeamDir Computes sun's beam direction depending on its position
/// \param azimutAngle Azimut angle of the sun
/// \param elevationAngle Elevation angle of the sun
/// \return Beam normalized direction vector
///
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

    //Azimut rotation quaternion
    quaternion qA = quaternion(ARotAxis,azimutAngle);

    //Elevation rotation quaternion
    quaternion qE = quaternion(ERotAxis,elevationAngle);

    //Total rotation quaternion
    quaternion q = qE*qA;

    sunPos = sunPos - origin;
    newSunPos = q*sunPos;
    newSunPos = newSunPos + origin;
    sunPos = sunPos + origin;

    //Compute sun beam direction
    TVec3d tmpDirection = (newSunPos - origin);

    return tmpDirection.normal();
}

std::map<int,TVec3d> loadSunpathFile(std::string sunpathFile, int iStartDate, int iEndDate)
{
    std::map<int,TVec3d> SunBeamDir;

    std::ifstream file(sunpathFile); //Load file
    std::string line;

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

            //*** The following lines needs to be commented out if input sun position values
            //*** already consider time changes of the 27 March and the 30 November

            //Encode 27 march and 30 november (dates when time changes : summer hour <-> winter hour)
            int twentySevenMarch = encodeDateTime(2016,3,27,0);
            int thirtyNovember = encodeDateTime(2016,11,30,0);
            bool timeshift = false;

            if(iCurrentDate < twentySevenMarch || iCurrentDate >= thirtyNovember)
            {
                //Skip first two cells -> shift time back from hour
                std::getline(lineStream,cell,';');
                std::getline(lineStream,cell,';');
                timeshift = true;
            }

            //*** End of the part which needs to be commented out

            int hour = 0;
            while(std::getline(lineStream,cell,';'))
            {
                //Get azimutAngle
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


            //*** The following lines needs to be commented out if input sun position values
            //*** already consider time changes of the 27 March and the 30 November

            if(timeshift)
            {
                //Add nul beam direction for last hour of the day
                int dateTime = encodeDateTime(sCurrentDate,hour);
                SunBeamDir[dateTime] = TVec3d(0.0,0.0,0.0);
            }

            //*** End of the part which needs to be commented out

        }
        else if(iCurrentDate > iEndDate)
            exit_loop = true;
    }

    if(found == false)
        std::cerr << "Error while loading Annual SunPath file." << std::endl;

    return SunBeamDir;

}
