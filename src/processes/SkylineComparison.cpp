#include "SkylineComparison.hpp"

#include <QString>
#include <vector>
#include <iostream>
#include "ogrsf_frmts.h"
#include "vecs.hpp"

////////////////////////////////////////////////////////////////////////////////
/// \brief dataSourceToPointList Gets all the OGRPoints from the OGRDataSource variable and add them into a list of OGRPoint*.
/// \param poDS OGRDataSource to get the Points from.
/// \return a list of OGRPoint*.
///
std::vector<OGRPoint*> dataSourceToPointList(OGRDataSource* poDS)
{
    std::vector<OGRPoint*> vPoints;

    OGRLayer *poLayer;
    int nbLayers = poDS->GetLayerCount();
    if(nbLayers > 0)
    {
        poLayer = poDS->GetLayer(0);
        OGRFeature *poFeature;
        poLayer->ResetReading();

        while( (poFeature = poLayer->GetNextFeature()) != NULL )
        {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();

            if(poGeometry != NULL && (poGeometry->getGeometryType() == OGRwkbGeometryType::wkbPoint25D || poGeometry->getGeometryType() == OGRwkbGeometryType::wkbPoint))
            {
                OGRPoint* point = dynamic_cast<OGRPoint*>(poGeometry);
                vPoints.push_back(point);
            }
        }
    }

    return vPoints;
}

std::vector<double> ComputeDist(std::vector<OGRPoint*> vPoints1, std::vector<OGRPoint*> vPoints2)
{
    std::vector<double> dist;

    if(vPoints1.size() != vPoints2.size())
    {
        std::cout << "Error : Curves are not the same size, consider discretisation" << std::cout;
        return dist;
    }

    for(unsigned int i = 0; i < vPoints1.size(); ++i)
    {
        //Compute euclidian norm
        //Pour le moment on stocke que la dist mais on pourrait aussi stocker la sémantique associée,...
        TVec3d vec = TVec3d(vPoints1.at(i)->getX() - vPoints2.at(i)->getX(), vPoints1.at(i)->getY() - vPoints2.at(i)->getY(), vPoints1.at(i)->getZ() - vPoints2.at(i)->getZ());
        double norm = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);

        dist.push_back(norm);
    }

    return dist;
}

void exportToCSV(std::vector<double> dist, std::string path, std::string filename)
{

}

void CompareSkylines()
{
    //*******************Comparison of 2 skylines

    //Load 2 shapefiles
    QString filepath1 = "/home/vincent/Documents/VCity_Project/NewFonctionalities/Comparaison_skylines/SkylinesNewAlgo/SkylineOutputBellecour3/SkylinePoints.shp";
    QString filepath2 = "/home/vincent/Documents/VCity_Project/NewFonctionalities/Comparaison_skylines/SkylinesNewAlgo/SkylineOutputBellecour3/SkylinePoints.shp";

    std::cout << "load shp file : " << filepath1.toStdString() << std::endl;
    OGRDataSource* poDS1 = OGRSFDriverRegistrar::Open(filepath1.toStdString().c_str(), /*TRUE*/FALSE); //False pour read only et TRUE pour pouvoir modifier

    std::cout << "load shp file : " << filepath2.toStdString() << std::endl;
    OGRDataSource* poDS2 = OGRSFDriverRegistrar::Open(filepath2.toStdString().c_str(), /*TRUE*/FALSE); //False pour read only et TRUE pour pouvoir modifier

    //Get vector of OGRPoints from OGRDataSource
    std::vector<OGRPoint*> vPoints1 = dataSourceToPointList(poDS1);
    std::vector<OGRPoint*> vPoints2 = dataSourceToPointList(poDS2);

    //Load point of view
    QString filepath_pov = "/home/vincent/Documents/VCity_Project/NewFonctionalities/Comparaison_skylines/SkylinesNewAlgo/SkylineOutputBellecour1/Viewpoint.shp";
    std::cout << "load shp file : " << filepath_pov.toStdString() << std::endl;
    OGRDataSource* OGRpov = OGRSFDriverRegistrar::Open(filepath_pov.toStdString().c_str(), /*TRUE*/FALSE); //False pour read only et TRUE pour pouvoir modifier

    std::vector<OGRPoint*> vPov = dataSourceToPointList(OGRpov);

    OGRPoint pov = *(vPov.at(0));

    //Compute discrete Fréchet Distance
    //std::cout << "Discrete Frechet dist = " << ComputeDiscreteFrechetDist(vPoints1,vPoints2) << std::endl;

    std::vector<double> dist = ComputeDist(vPoints1,vPoints2);

//    double max = dist.at(0);

//    for(double d : dist)
//    {
//        if(d > max)
//            max = d;
//    }

//    std::cout << "Discrete Frechet dist = " << max << std::endl;


    //Check if points lies in the same plane
//    for(unsigned int i = 0; i < vPoints1.size() - 1; ++i)
//    {
//        bool test = PointsLiesInSamePlane(*(vPoints1.at(i)), *(vPoints1.at(i+1)), *(vPoints2.at(i)), *(vPoints2.at(i+1)));

//        if (test == false)
//        {
//            std::cout << "points " << i << ", " << i+1 << " don't lie in the same plane" << std::endl;
//        }
//    }

//    double summedDist = 0.0;

//    for(unsigned int i = 0; i < vPoints1.size() - 1; ++i)
//    {
//        double area = ComputeAreaOfQuads(*(vPoints1.at(i)), *(vPoints1.at(i+1)), *(vPoints2.at(i)), *(vPoints2.at(i+1)), pov);

//        std::cout << "Area between points " << i << ", " << i+1 << " : " << area << std::endl;

//        summedDist += area;
//    }

//    std::cout << "Summed Distance : " << summedDist << std::endl;
}


#if 0

double ComputeDiscreteFrechetDist(std::vector<OGRPoint*> vPoints1, std::vector<OGRPoint*> vPoints2)
{
    if(vPoints1.size() != vPoints2.size())
    {
        std::cout << "Error : Curves are not the same size, consider discretisation" << std::cout;
        return 0.0;
    }

    double dist = 0.0;
    int index_distmax = 0;
    int cpt = 0;

    //Iterate through two lists simultaneously
    std::vector<OGRPoint*>::iterator itCurve1 = vPoints1.begin();
    std::vector<OGRPoint*>::iterator itCurve2 = vPoints2.begin();

    for(; itCurve1 != vPoints1.end() && itCurve2 != vPoints2.end(); ++itCurve1, ++itCurve2)
    {
        //Compute euclidian norm
        //Pour le moment on stocke que la dist mais on pourrait aussi stocker la sémantique associée,...
        TVec3d vec = TVec3d((*itCurve1)->getX() - (*itCurve2)->getX(), (*itCurve1)->getY() - (*itCurve2)->getY(), (*itCurve1)->getZ() - (*itCurve2)->getZ());
        double norm = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);

        if(norm > dist)
        {
            dist = norm;
            index_distmax = cpt;
        }

        ++cpt;
    }

    std::cout << "index dist max : " << index_distmax << std::endl;

    return dist;
}


bool PointsLiesInSamePlane(OGRPoint p1, OGRPoint p2, OGRPoint p3, OGRPoint p4)
{
    double epsilon = 0.001;

    //Build three vectors
    TVec3d v1 = TVec3d(p2.getX() - p1.getX(), p2.getY() - p1.getY(), p2.getZ() - p1.getZ());
    TVec3d v2 = TVec3d(p3.getX() - p1.getX(), p3.getY() - p1.getY(), p3.getZ() - p1.getZ());
    TVec3d v3 = TVec3d(p4.getX() - p1.getX(), p4.getY() - p1.getY(), p4.getZ() - p1.getZ());

    //compute normal vector of supposed plane
    TVec3d n = v1.cross(v2);

    if(fabs(v3.dot(n)) <= epsilon)
        return true;
    else
        return false;
}


double ComputeAreaOfQuads(OGRPoint p1, OGRPoint p2, OGRPoint p3, OGRPoint p4, OGRPoint pov)
{
    //Compute angle
//    TVec3d lineCenter = TVec3d((p1.getX() + p2.getX()) / 2.0, (p1.getY() + p2.getY()) / 2.0, (p1.getZ() + p2.getZ()) / 2.0);
//    TVec3d oppositeSide = TVec3d (lineCenter.x - p1.getX(), lineCenter.y - p1.getY(), lineCenter.z - p1.getZ());
//    TVec3d hypotenuse = TVec3d(p1.getX() - pov.getX(), p1.getY() - pov.getY(), p1.getZ() - pov.getZ());

//    double angle = asin(oppositeSide / hypotenuse) * 2.0;

    //Compute area of first triangle using Heron formula

    //Compute sides length
    TVec3d a1 = TVec3d(p1.getX() - pov.getX(), p1.getY() - pov.getY(), p1.getZ() - pov.getZ());
    TVec3d b1 = TVec3d(p2.getX() - p1.getX(), p2.getY() - p1.getY(), p2.getZ() - p1.getZ());
    TVec3d c1 = TVec3d(p2.getX() - pov.getX(), p2.getY() - pov.getY(), p2.getZ() - pov.getZ());

    double sp1 = (a1.length() + b1.length() + c1.length()) / 2.0; //semi perimeter

    double S1 = sqrt(sp1 * (sp1 - a1.length()) * (sp1 - b1.length()) * (sp1 - c1.length()));

    //Compute area of second triangle
    TVec3d a2 = TVec3d(p3.getX() - pov.getX(), p3.getY() - pov.getY(), p3.getZ() - pov.getZ());
    TVec3d b2 = TVec3d(p4.getX() - p3.getX(), p4.getY() - p3.getY(), p4.getZ() - p3.getZ());
    TVec3d c2 = TVec3d(p4.getX() - pov.getX(), p4.getY() - pov.getY(), p4.getZ() - pov.getZ());

    double sp2 = (a2.length() + b2.length() + c2.length()) / 2.0; //semi perimeter

    double S2 = sqrt(sp2 * (sp2 - a2.length()) * (sp2 - b2.length()) * (sp2 - c2.length()));

    //Compute area of trapezium
    double area = fabs(S1 - S2);

    return area;
}


double LinearInterp(double z1, double z2, double mu)
{
    return (z1 * (1 - mu) + z2 * mu);
}

/***********
Compute intersection between two segments. Returns true if the two lines intersect and fill pIntersect with the intersection coordinates.
Only one point from the second skyline will match the other one so we compute the intersection in 2D and then add the z coordinates (by interpolation).
p1, p2 : points of the line
p3, p4 : points of the segment line
*************/
TVec3d LineLineIntersection(TVec3d p1, TVec3d p2, TVec3d p3, TVec3d p4)
{
    TVec3d pIntersec(0.0, 0.0, 0.0);

    // Peut etre faire une fonction recursive qui teste d'abord entre droite 1 et le segment, puis si on trouve pas, on teste avec les segments à dte et à gauche,..
    //Est ce que c'est utile ? Sinon faire une fonction qui teste avec toutes les polylignes -> permet de gérer les cas ou il y a des renfoncements avec les arbres (on garde la plus haute intersection)
    double epsilon = 0.001;

    double denominator = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

    if(fabs(denominator) < epsilon) //if lines are parallel TODO: differenciate coincident and parallel)
    {
        //take the point with the max y (the highest in terms of skylines)
        //maybe to be changed by the farest point fom pov in direction of pov to point of skyline
        if(p3.y >= p4.y)
            pIntersec = p3;
        else
            pIntersec = p4;

        std::cout << "parallels " << std::endl;
    }

    // else, compute intersection point
    pIntersec.x = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / denominator;
    pIntersec.y = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / denominator;

    return pIntersec;

//    std::cout << "" << std::endl;
//    std::cout << "p3 : " << p3.x << " ; " << p3.y << std::endl;
//    std::cout <<  "intersection point : " << (*pIntersec).x /*<< " ; " << (*pIntersec).y */<< std::endl;
//    std::cout << "p4 : " << p4.x << " ; " << p4.y << std::endl;

    //check if its inside the line segment interval or not
    //Supposing that p3 <= p4, is that always true ?
//    if((pIntersec.x >= p3.x && (*pIntersec).x <= p4.x) ||
//            ((*pIntersec).y >= p3.y && (*pIntersec).y <= p4.y))
//    {
//        //Interpolate z value
//        double mu = ((*pIntersec).x - p3.x) / (p4.x - p3.x);
//        (*pIntersec).z = LinearInterp(p3.z, p4.z, mu);
//        return true;
//    }
//    else
//        return false;

}

TVec3d OGRPointToTVec3d(OGRPoint OGRP)
{
    return TVec3d(OGRP.getX(), OGRP.getY(), OGRP.getZ());
}

OGRPoint TVec3dToOGRPoint(TVec3d vec3dP)
{
    return OGRPoint(vec3dP.x, vec3dP.y, vec3dP.z);
}
#endif
