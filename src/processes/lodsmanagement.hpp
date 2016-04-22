// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __LODSMANAGEMENT_HPP__
#define __LODSMANAGEMENT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "libcitygml/URI.hpp"
#include "libcitygml/citygml.hpp"
#include "src/gui/osg/osgGDAL.hpp"
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////
OGRMultiPolygon * GetEnveloppe(OGRMultiPolygon * MP);

void generateLOD0fromLOD2(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin);
citygml::Geometry* ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin);
citygml::CityObject* ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin);
////////////////////////////////////////////////////////////////////////////////
#endif // __LODSMANAGEMENT_HPP__
