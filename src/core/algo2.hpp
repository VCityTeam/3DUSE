// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __ALGO2_HPP__
#define __ALGO2_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "URI.hpp"
#include "scene.hpp"
#include "dataprofile.hpp"
#include "settings.hpp"
#include "tools/log.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    class Algo2
    {
    public:
        void fixBuilding(const std::vector<URI>& uris);
        void stockeForme(citygml::CityObject *, int val);
        void recupTerrainBat(citygml::CityObject* node, int s);
        float sign(TVec3d p1, TVec3d p2, TVec3d p3);
        bool pointDansTriangle(TVec3d pt, TVec3d v1, TVec3d v2, TVec3d v3);
        void recup(citygml::CityObject *, int);

    private:
        citygml::LinearRing * contour;
        citygml::LinearRing * sol;
        std::vector<int> hauteurTerrainSousBat;
        std::vector<int> solBat;
    };
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __ALGO2_HPP__
