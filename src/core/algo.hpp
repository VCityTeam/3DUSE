// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __ALGO_HPP__
#define __ALGO_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "URI.hpp"
#include "geos/geom/GeometryFactory.h"
#include "libcitygml/citygml.hpp"

#include "src/gui/osg/osgGDAL.hpp"
#include "osg/Geode"
#include "osg/Geometry"
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
class Algo
{
public:
	Algo();
	~Algo();

	void DecoupeCityGML(geos::geom::Geometry * ShapeGeo, std::vector<BatimentShape> BatimentsInfo);

	void generateLOD0(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin);

	citygml::Geometry* ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin);
	citygml::CityObject* ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin);

	void CompareTiles(citygml::CityModel *City1, citygml::CityModel *City2);

    void SaveGeometrytoShape(std::string name, const OGRMultiPolygon* G);

	std::string Folder;
private:
};
// entry points for LODs ?
// split algos in multiple files ?
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __ALGO_HPP__

