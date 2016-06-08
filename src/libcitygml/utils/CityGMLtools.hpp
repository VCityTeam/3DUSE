// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __CITYGMLTOOLS_HPP__
#define __CITYGMLTOOLS_HPP__

#include <stdlib.h>
#include "libcitygml/citygml.hpp"
#include "ogrsf_frmts.h"
#include "citygmlutils_export.h"

CITYGMLUTILS_EXPORT void generateLOD0fromLOD2(
  citygml::CityObject* obj,
  OGRMultiPolygon ** Enveloppe,
  double * heightmax,
  double * heightmin);

CITYGMLUTILS_EXPORT citygml::Geometry* ConvertLOD0ToCityGML(
  std::string name,
  OGRMultiPolygon * Geometry,
  double * heightmin);

CITYGMLUTILS_EXPORT citygml::CityObject* ConvertLOD1ToCityGML(
  std::string name,
  OGRMultiPolygon * Enveloppe,
  double * heightmax,
  double * heightmin);

CITYGMLUTILS_EXPORT citygml::Polygon * ConvertOGRPolytoGMLPoly(
  OGRPolygon* OGRPoly,
  std::string Name);

#endif // __CITYGMLTOOLS_HPP__
