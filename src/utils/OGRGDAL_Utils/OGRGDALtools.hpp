// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __OGRGDALTOOLS_HPP__
#define __OGRGDALTOOLS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "vecs.hpp"
#include "ogrsf_frmts.h"
////////////////////////////////////////////////////////////////////////////////
static double Precision_Vect = 0.00001; //Précision pour les points des Wall qui ne se superposent pas parfaitement aux emprises au sol issue du Roff

OGRMultiPolygon * GetEnveloppe(OGRMultiPolygon * MP);

OGRPoint* ProjectPointOnPolygon3D(OGRPoint* Point, OGRPolygon* Polygon);
OGRPolygon* ProjectPolyOn3DPlane(OGRPolygon* Poly2D, OGRPolygon * Poly3D);
OGRMultiPolygon* ProjectPolyOn3DPlane(OGRMultiPolygon* Poly2D, OGRPolygon * Poly3D);

double Hausdorff(OGRMultiPolygon * GeoPoints, OGRMultiPolygon * Geo);
double DistanceHausdorff(OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2);

void ChangePointsOrderForNormal(OGRLinearRing* Ring, std::vector<TVec2f>* Tex);
OGRGeometry * CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp, std::vector<TVec2f> *TexUV, std::vector<std::vector<TVec2f>>* TexUVout);
TVec2f CalculUV(std::vector<TVec3d>* Poly, std::vector<TVec2f>* UVs, TVec3d Point);
std::vector<OGRMultiLineString*> GetLineStringsFromPolygon(OGRPolygon* Polygon);
std::vector<OGRMultiLineString*> GetLineStringsFromMultiPolygon(OGRGeometryCollection* MultiPolygon);
std::vector<OGRMultiPoint*> GetPointsFromPolygon(OGRPolygon* Polygon);
////////////////////////////////////////////////////////////////////////////////
#endif // __OGRGDALTOOLS_HPP__
