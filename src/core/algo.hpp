#ifndef __ALGO_HPP__
#define __ALGO_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "URI.hpp"
#include "geos/geom/GeometryFactory.h"
#include "libcitygml/citygml.hpp"

#include "BatimentShape.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
class Algo
{
public:
	void generateLOD0(const URI& uri);
    void generateLOD0Scene(geos::geom::Geometry * ShapeGeo, std::vector<BatimentShape> BatimentsInfo);
	void generateLOD1(geos::geom::Geometry * ShapeGeo, std::vector<std::pair<double, double>> Hauteurs);
	void CompareTiles();

    citygml::CityModel* getCitymodel();

private:
    citygml::CityModel* m_model; ///< Generation result
};
// entry points for LODs ?
// split algos in multiple files ?
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __ALGO_HPP__

