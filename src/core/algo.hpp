// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __ALGO_HPP__
#define __ALGO_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "URI.hpp"
#include "geos/geom/GeometryFactory.h"
#include "libcitygml/citygml.hpp"

#include "BatimentShape.hpp"
#include "src/gui/osg/osgGDAL.hpp"
#include "osg/Geode"
#include "osg/Geometry"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
class Algo
{
public:
	Algo();
	~Algo();
	void generateLOD0(const URI& uri);
	void generateLOD0(citygml::CityObject* obj);
	void DecoupeCityGML(geos::geom::Geometry * ShapeGeo, std::vector<BatimentShape> BatimentsInfo);
	void generateLOD1(geos::geom::Geometry * ShapeGeo, std::vector<std::pair<double, double>> Hauteurs);
	void generateLOD0(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin);
	void generateLOD0(citygml::CityObject* obj, geos::geom::Geometry ** Enveloppe, double * heightmax, double * heightmin);
	citygml::Geometry* ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin);
	citygml::Geometry* ConvertLOD0ToCityGML(std::string name, geos::geom::Geometry * Geometry, double Zmin);
	citygml::CityObject* ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin);
	citygml::CityObject* ConvertLOD1ToCityGML(std::string name, geos::geom::Geometry * Geometry, double * heightmax, double * heightmin);
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

