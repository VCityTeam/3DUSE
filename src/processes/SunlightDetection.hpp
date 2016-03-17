#ifndef SUNLIGHTDETECTION_HPP
#define SUNLIGHTDETECTION_HPP

#include "vecs.hpp"
#include <stdlib.h>
#include <map>

class Triangle;

///
/// \brief The TriangleLightInfo struct holds informations about sunlight for a year on a given triangle
///
struct TriangleLightInfo
{
    Triangle* triangle;
    std::map<std::string,bool> yearSunInfo;
};


void SunlightDetection();

#endif // SUNLIGHTDETECTION_HPP
