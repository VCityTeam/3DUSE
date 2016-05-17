// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __TOOLALGOCUT_HPP__
#define __TOOLALGOCUT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "libcitygml/citygml.hpp"
#include "ogrsf_frmts.h"
////////////////////////////////////////////////////////////////////////////////
static double Precision_Vect = 0.00001; //Précision pour les points des Wall qui ne se superposent pas parfaitement aux emprises au sol issue du Roff

citygml::Polygon * ConvertOGRPolytoGMLPoly(OGRPolygon* OGRPoly, std::string Name);
OGRPoint* ProjectPointOnPolygon3D(OGRPoint* Point, OGRPolygon* Polygon);
void ChangePointsOrderForNormal(OGRLinearRing* Ring, std::vector<TVec2f>* Tex);
OGRGeometry * CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp, std::vector<TVec2f> *TexUV, std::vector<std::vector<TVec2f>>* TexUVout);
TVec2f CalculUV(std::vector<TVec3d>* Poly, std::vector<TVec2f>* UVs, TVec3d Point);
std::vector<OGRMultiLineString*> GetLineStringsFromPolygon(OGRPolygon* Polygon);
std::vector<OGRMultiLineString*> GetLineStringsFromMultiPolygon(OGRGeometryCollection* MultiPolygon);
std::vector<OGRMultiPoint*> GetPointsFromPolygon(OGRPolygon* Polygon);
////////////////////////////////////////////////////////////////////////////////
#endif // __TOOLALGOCUT_HPP__
