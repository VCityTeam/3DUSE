// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __CITYGMLTOOLS_HPP__
#define __CITYGMLTOOLS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "libcitygml/citygml.hpp"
#include "ogrsf_frmts.h"
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
void generateLOD0fromLOD2(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin);
citygml::Geometry* ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin);
citygml::CityObject* ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin);

citygml::Polygon * ConvertOGRPolytoGMLPoly(OGRPolygon* OGRPoly, std::string Name);
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGMLTOOLS_HPP__
