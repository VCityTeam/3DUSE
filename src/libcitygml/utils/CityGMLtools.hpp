// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
