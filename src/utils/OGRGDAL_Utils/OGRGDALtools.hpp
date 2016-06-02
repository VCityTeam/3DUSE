// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __OGRGDALTOOLS_HPP__
#define __OGRGDALTOOLS_HPP__

#include <vector>
#include <ogrsf_frmts.h>
#include "vcitycore_export.h"
#include "vecs.hpp"

/// Précision pour les points des Wall qui ne se superposent pas parfaitement
/// aux emprises au sol issue du Roff
static double Precision_Vect = 0.00001;

VCITYCORE_EXPORT OGRPoint* ProjectPointOnPolygon3D( OGRPoint* Point,
                                                    OGRPolygon* Polygon);

VCITYCORE_EXPORT OGRMultiPolygon* GetEnveloppe(OGRMultiPolygon * MP);

// FIXME: the following declarations don't seem to be part of the API.
// Remove them (the declarations not the defenitions within the cpp of course).



OGRPolygon* ProjectPolyOn3DPlane(OGRPolygon* Poly2D, OGRPolygon * Poly3D);

OGRMultiPolygon* ProjectPolyOn3DPlane(OGRMultiPolygon* Poly2D, OGRPolygon * Poly3D);

double Hausdorff(OGRMultiPolygon * GeoPoints, OGRMultiPolygon * Geo);

double DistanceHausdorff(OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2);

void ChangePointsOrderForNormal(OGRLinearRing* Ring, std::vector<TVec2f>* Tex);

OGRGeometry* CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp, std::vector<TVec2f> *TexUV, std::vector<std::vector<TVec2f>>* TexUVout);

TVec2f CalculUV(std::vector<TVec3d>* Poly, std::vector<TVec2f>* UVs, TVec3d Point);

std::vector<OGRMultiLineString*> GetLineStringsFromPolygon(OGRPolygon* Polygon);

std::vector<OGRMultiLineString*> GetLineStringsFromMultiPolygon(OGRGeometryCollection* MultiPolygon);

std::vector<OGRMultiPoint*> GetPointsFromPolygon(OGRPolygon* Polygon);
////////////////////////////////////////////////////////////////////////////////
#endif // __OGRGDALTOOLS_HPP__
