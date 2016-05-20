#ifndef SKYLINECOMPARISON_HPP
#define SKYLINECOMPARISON_HPP

#include <vector>

enum Compo
{
    Building,
    TIN,
    Veget,
    Water,
    Other
};

//struct CompResult
//{
//    int order;
//    double angle;
//    double dist;
//    std::pair<Compo,Compo> compoSkylines;
//};

void CompareSkylines();


#endif // SKYLINECOMPARISON_HPP
