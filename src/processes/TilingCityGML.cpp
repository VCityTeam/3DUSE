#include "TilingCityGML.hpp"
#include "lodsmanagement.hpp"
#include "ExportToShape.hpp"
#include <iomanip>
#include <math.h>

double Precision_Vect = 0.00001; //Précision pour les points des Wall qui ne se superposent pas parfaitement aux emprises au sol issue du Roff
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>

#include "export/exportCityGML.hpp"
//#include "src/processes/LinkCityGMLShape.cpp"

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Convertit un OGRPolygon* en citygml::Polygon*
* @param OGRPoly OGRPolygon* à convertir.
* @param Name Nom du Polygon CityGML à retourner.
*/
citygml::Polygon * ConvertOGRPolytoGMLPoly(OGRPolygon* OGRPoly, std::string Name)
{
	citygml::Polygon * Poly = new citygml::Polygon(Name + "_Poly");
	citygml::LinearRing * Ring = new citygml::LinearRing(Name + "_Ring", true);

	const OGRLinearRing * ExtRing = OGRPoly->getExteriorRing();

	for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
	{
		OGRPoint * point = new OGRPoint;
		ExtRing->getPoint(j, point);
		double x = point->getX();
		double y = point->getY();
		double z = point->getZ();
		delete point;

		Ring->addVertex(TVec3d(x, y, z));
	}
	Poly->addRing(Ring);

	return Poly;
}

/**
* @brief ProjectPointOnPolygon3D : prend un point 2D Point et calcule sa coordonnée Z en partant du principe qu'il est coplanaire à Polygon
* @param Point : point que l'on veut extruder en 3D
* @param Polygon : polygon qui définit le plan sur lequel vient se poser Point
* @return le point 3D correspondant
*/
OGRPoint* ProjectPointOnPolygon3D(OGRPoint* Point, OGRPolygon* Polygon)
{
	OGRLinearRing* Ring = Polygon->getExteriorRing();

	TVec3d A;
	TVec3d B;
	TVec3d C;

	A.x = Ring->getX(0);
	A.y = Ring->getY(0);
	A.z = Ring->getZ(0);

	TVec3d AB;
	TVec3d AC;

	int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
	for(int i = 1; i < Ring->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une répétition du premier
	{
		if(test == 0)
		{
			B.x = Ring->getX(i);
			B.y = Ring->getY(i);
			B.z = Ring->getZ(i);

			if(A.x != B.x || A.y != B.y)
			{
				++test;// A est bien différent de B
				AB = B - A;
			}
		}
		else if(test == 1)
		{
			C.x = Ring->getX(i);
			C.y = Ring->getY(i);
			C.z = Ring->getZ(i);

			if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y))
			{
				++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
				AC = C - A;
				break;
			}
		}
	}

	if(test != 2)
	{
		std::cout << "Erreur lors de la creation du plan. \n";
		return nullptr;
	}

	TVec3d M; // <=> Point
	M.x = Point->getX();
	M.y = Point->getY();

	double s, t;

	t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
	s = (M.x - A.x - t * AC.x) / AB.x;

	M.z = A.z + s * AB.z + t * AC.z;

	return new OGRPoint(M.x, M.y, M.z);
}

/**
* @brief Découpe un polygon 3D GMLPoly suivant un second polygon 2D BuildingShp. Il faut que le polygon découpé soit encore en 3d.
* @param GMLPoly représente le premier polygon 3D (issu du fichier CityGML)
* @param BuildingShp représente le second polygon 2D (issu du fichier Shape et modifié pour coller aux emprises du CityGML)
* @param TexUV représente les coordonnées de texture en entrée correspondant au GMLPoly
* @param TexUVout représente les coordonnées de texture en sortie pour la Geometry résultat.
*/
OGRGeometry * CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp, std::vector<TVec2f> *TexUV, std::vector<std::vector<TVec2f>>* TexUVout)
{
	//SaveGeometrytoShape("A_GMLPoly.shp", GMLPoly);
	//SaveGeometrytoShape("A_BuildingShp.shp", BuildingShp);

	OGRGeometry* Inter = GMLPoly->Intersection(BuildingShp);

	//On va parcourir chaque point P de Inter et calculer sa position dans GMLPoly afin de calculer sa coordonnée z
	//On commence par récupérer trois points A, B et C de GMLPoly non alignés pour obtenir l'équation paramétrique du plan formé par ce polygon
	OGRLinearRing* GMLRing = GMLPoly->getExteriorRing();

	TVec3d A;
	TVec3d B;
	TVec3d C;
	TVec2f uvA;
	TVec2f uvB;
	TVec2f uvC;
	TVec2f uvAB;
	TVec2f uvAC;

	A.x = GMLRing->getX(0);
	A.y = GMLRing->getY(0);
	A.z = GMLRing->getZ(0);

	uvA = TexUV->at(0);

	TVec3d AB;
	TVec3d AC;

	int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
	for(int i = 1; i < GMLRing->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une répétition du premier
	{
		if(test == 0)
		{
			B.x = GMLRing->getX(i);
			B.y = GMLRing->getY(i);
			B.z = GMLRing->getZ(i);

			if(A.x != B.x || A.y != B.y)
			{
				++test;// A est bien différent de B
				AB = B - A;
				uvB = TexUV->at(i);
				uvAB = uvB - uvA;
			}
		}
		else if(test == 1)
		{
			C.x = GMLRing->getX(i);
			C.y = GMLRing->getY(i);
			C.z = GMLRing->getZ(i);

			if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y))
			{
				++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
				AC = C - A;
				uvC = TexUV->at(i);
				uvAC = uvC - uvA;
				break;
			}
		}
	}

	if(test != 2)
	{
		std::cout << "Erreur lors de la creation du plan. \n";
		delete Inter;
		return nullptr;
	}

	OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(Inter);
	if(PolyInter != nullptr)
	{
		//SaveGeometrytoShape("A_PolyInter.shp", PolyInter);
		if(PolyInter->get_Area() == GMLPoly->get_Area())
		{
			delete Inter;
			TexUVout->push_back(*TexUV);
			return GMLPoly->clone(); //GMLPoly est inclu dans BuildingShp, il n'y a pas besoin de le modifier
		}
		OGRPolygon* ResPoly = new OGRPolygon;
		OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
		OGRLinearRing* ResExtRing = new OGRLinearRing;

		std::vector<TVec2f> uvPolyInter;

		for(int i = 0; i < InterExtRing->getNumPoints(); ++i)
		{
			OGRPoint* P = new OGRPoint;
			InterExtRing->getPoint(i, P);

			TVec3d M; // <=> P
			M.x = P->getX();
			M.y = P->getY();

			double s, t;

			t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
			s = (M.x - A.x - t * AC.x) / AB.x;

			M.z = A.z + s * AB.z + t * AC.z;

			ResExtRing->addPoint(M.x, M.y, M.z);

			uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquées sans déformation.

			delete P;
		}

		ResPoly->addRingDirectly(ResExtRing);

		TexUVout->push_back(uvPolyInter);

		return ResPoly;
	}
	else //Si l'intersection ne représente pas un simple Polygon, il faut rechercher si c'est une GeometryCollection qui en contient, afin de pouvoir ensuite les récupérer.
	{
		//std::cout << "Pas Polygon !! " << Inter->getGeometryName() << std::endl;

		OGRGeometryCollection* GC_Inter = dynamic_cast<OGRGeometryCollection*>(Inter);
		if(GC_Inter != nullptr)
		{
			OGRMultiPolygon* ResMultiPoly = new OGRMultiPolygon;
			for(int j = 0; j < GC_Inter->getNumGeometries(); ++j)
			{
				OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(GC_Inter->getGeometryRef(j));
				if(PolyInter != nullptr)
				{
					OGRPolygon* ResPoly = new OGRPolygon;
					OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
					OGRLinearRing* ResExtRing = new OGRLinearRing;

					std::vector<TVec2f> uvPolyInter;

					for(int i = 0; i < InterExtRing->getNumPoints(); ++i)
					{
						OGRPoint* P = new OGRPoint;
						InterExtRing->getPoint(i, P);

						TVec3d M; // <=> P
						M.x = P->getX();
						M.y = P->getY();

						double s, t;

						t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
						s = (M.x - A.x - t * AC.x) / AB.x;

						M.z = A.z + s * AB.z + t * AC.z;

						ResExtRing->addPoint(M.x, M.y, M.z);

						uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquées sans déformation.

						delete P;
					}

					ResPoly->addRingDirectly(ResExtRing);

					ResMultiPoly->addGeometryDirectly(ResPoly);

					TexUVout->push_back(uvPolyInter);
				}
			}
			return ResMultiPoly;
		}
		return nullptr;
	}
}

/**
* @brief Découpe un polygon 3D GMLPoly suivant un second polygon 2D BuildingShp. Il faut que le polygon découpé soit encore en 3d.
* @param GMLPoly représente le premier polygon 3D (issu du fichier CityGML)
* @param BuildingShp représente le second polygon 2D (issu du fichier Shape et modifié pour coller aux emprises du CityGML)
*/
OGRGeometry * CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp)
{
	OGRGeometry* Inter = GMLPoly->Intersection(BuildingShp);

	//On va parcourir chaque point P de Inter et calculer sa position dans GMLPoly afin de calculer sa coordonnée z
	//On commence par récupérer trois points A, B et C de GMLPoly non alignés pour obtenir l'équation paramétrique du plan formé par ce polygon
	OGRLinearRing* GMLRing = GMLPoly->getExteriorRing();

	TVec3d A;
	TVec3d B;
	TVec3d C;

	A.x = GMLRing->getX(0);
	A.y = GMLRing->getY(0);
	A.z = GMLRing->getZ(0);

	TVec3d AB;
	TVec3d AC;

	int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
	for(int i = 1; i < GMLRing->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une répétition du premier
	{
		if(test == 0)
		{
			B.x = GMLRing->getX(i);
			B.y = GMLRing->getY(i);
			B.z = GMLRing->getZ(i);

			if(A.x != B.x || A.y != B.y)
			{
				++test;// A est bien différent de B
				AB = B - A;
			}
		}
		else if(test == 1)
		{
			C.x = GMLRing->getX(i);
			C.y = GMLRing->getY(i);
			C.z = GMLRing->getZ(i);

			if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y))
			{
				++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
				AC = C - A;
				break;
			}
		}
	}

	if(test != 2)
	{
		std::cout << "Erreur lors de la creation du plan. \n";
		delete Inter;
		return nullptr;
	}

	OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(Inter);
	if(PolyInter != nullptr)
	{
		if(PolyInter->get_Area() == GMLPoly->get_Area())
		{
			delete Inter;
			return GMLPoly->clone(); //GMLPoly est inclu dans BuildingShp, il n'y a pas besoin de le modifier
		}
		OGRPolygon* ResPoly = new OGRPolygon;
		OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
		OGRLinearRing* ResExtRing = new OGRLinearRing;

		for(int i = 0; i < InterExtRing->getNumPoints(); ++i)
		{
			OGRPoint* P = new OGRPoint;
			InterExtRing->getPoint(i, P);

			TVec3d M; // <=> P
			M.x = P->getX();
			M.y = P->getY();

			double s, t;

			t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
			s = (M.x - A.x - t * AC.x) / AB.x;

			M.z = A.z + s * AB.z + t * AC.z;

			ResExtRing->addPoint(M.x, M.y, M.z);

			delete P;
		}

		ResPoly->addRingDirectly(ResExtRing);

		return ResPoly;
	}
	else //Si l'intersection ne représente pas un simple Polygon, il faut rechercher si c'est une GeometryCollection qui en contient, afin de pouvoir ensuite les récupérer.
	{
		OGRGeometryCollection* GC_Inter = dynamic_cast<OGRGeometryCollection*>(Inter);
		if(GC_Inter != nullptr)
		{
			OGRMultiPolygon* ResMultiPoly = new OGRMultiPolygon;
			for(int j = 0; j < GC_Inter->getNumGeometries(); ++j)
			{
				OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(GC_Inter->getGeometryRef(j));
				if(PolyInter != nullptr)
				{
					OGRPolygon* ResPoly = new OGRPolygon;
					OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
					OGRLinearRing* ResExtRing = new OGRLinearRing;

					for(int i = 0; i < InterExtRing->getNumPoints(); ++i)
					{
						OGRPoint* P = new OGRPoint;
						InterExtRing->getPoint(i, P);

						TVec3d M; // <=> P
						M.x = P->getX();
						M.y = P->getY();

						double s, t;

						t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
						s = (M.x - A.x - t * AC.x) / AB.x;

						M.z = A.z + s * AB.z + t * AC.z;

						ResExtRing->addPoint(M.x, M.y, M.z);

						delete P;
					}

					ResPoly->addRingDirectly(ResExtRing);

					ResMultiPoly->addGeometryDirectly(ResPoly);
				}
			}
			return ResMultiPoly;
		}
		return nullptr;
	}
}

/**
* @brief Calcul les coordonnées de texture UV d'un point par rapport à un polygone qui lui est coplanaire.
* @param Poly Contient les coordonnées des points du polygone
* @param UVs Contient les coordonnées de texture des points du polygone
* @param Point Contient le point dont on désire calculer les coordonnées de texture
*/
TVec2f CalculUV(std::vector<TVec3d>* Poly, std::vector<TVec2f>* UVs, TVec3d Point)
{
	TVec3d A;
	TVec3d B;
	TVec3d C;
	TVec2f uvA;
	TVec2f uvB;
	TVec2f uvC;
	TVec2f uvAB;
	TVec2f uvAC;

	A = Poly->at(0);

	uvA = UVs->at(0);

	TVec3d AB;
	TVec3d AC;

	int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
	for(int i = 1; i < Poly->size(); ++i) //Le premier point n'est pas répété à la fin
	{
		if(test == 0)
		{
			B = Poly->at(i);

			if(A != B)
			{
				++test;// A est bien différent de B
				AB = B - A;
				uvB = UVs->at(i);
				uvAB = uvB - uvA;
			}
		}
		else if(test == 1)
		{
			C = Poly->at(i);

			if((C.x - A.x)/(B.x - A.x) != (C.y - A.y)/(B.y - A.y) || (C.z - A.z)/(B.z - A.z) != (C.y - A.y)/(B.y - A.y) || (C.x - A.x)/(B.x - A.x) != (C.z - A.z)/(B.z - A.z))
			{
				++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
				AC = C - A;
				uvC = UVs->at(i);
				uvAC = uvC - uvA;
				break;
			}
		}
	}

	if(AB.x == 0 && AB.y == 0 && AC.x == 0 && AC.y == 0) // Pour les polygones de murs tenant sur un seul point (x,y) (ces polygones sont inutiles ?)
		return TVec2f(0, 0);

	double s, t;

	if(AB.x != 0)
	{
		t = (A.z * AB.x - A.x * AB.z + AB.z * Point.x - AB.x * Point.z) / (AB.z * AC.x - AB.x * AC.z);
		s = (Point.x - A.x - t * AC.x) / AB.x;
	}
	else if(AB.y != 0)
	{
		t = (A.z * AB.y - A.y * AB.z + AB.z * Point.y - AB.y * Point.z) / (AB.z * AC.y - AB.y * AC.z);
		s = (Point.y - A.y - t * AC.y) / AB.y;
	}
	else if(AB.z != 0)
	{
		t = (A.x * AB.z - A.z * AB.x + AB.x * Point.z - AB.z * Point.x) / (AB.x * AC.z - AB.z * AC.x);
		s = (Point.z - A.z - t * AC.z) / AB.z;
	}
	else
	{
		t = (A.y * AB.z - A.z * AB.y + AB.y * Point.z - AB.z * Point.y) / (AB.y * AC.z - AB.z * AC.y);
		s = (Point.z - A.z - t * AC.z) / AB.z;
	}

	if(AB.z == 0 && AC.z == 0)//Pour les bâtiments remarquables qui ne sont pas en LOD2 et qui ont des murs horizontaux, pour éviter les -1.#IND
	{
		t = (A.y * AB.x - A.x * AB.y + AB.y * Point.x - AB.x * Point.y) / (AB.y * AC.x - AB.x * AC.y);
		s = (Point.x - A.x - t * AC.x) / AB.x;
	}

	TVec2f Res = TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y);

	/*if(Res.x != Res.x || Res.y != Res.y)
	{
	std::cout << std::setprecision(15) << "s - t : " << s << " - " << t << std::endl;
	std::cout << Res << std::endl;
	std::cout << std::endl << "A : " << A << std::endl;
	std::cout << "uvA : " << uvA << std::endl;
	std::cout << "B : " << B << std::endl;
	std::cout << "uvB : " << uvB << std::endl;
	std::cout << "C : " << C << std::endl;
	std::cout << "uvC : " << uvC << std::endl;
	std::cout << "Point : " << Point << std::endl;

	for(TVec3d Vec: *Poly)
	std::cout << "Poly : " << Vec << std::endl;

	std::cout << std::endl << "AB : " << AB << std::endl;
	std::cout << "AC : " << AC << std::endl;
	std::cout << "uvAB : " << uvAB << std::endl;
	std::cout << "uvAC : " << uvAC << std::endl;

	int a;
	std::cin >> a;
	}*/

	return Res;
}

/**
* @brief GetLineStringsFromPolygon lit un ou plusieurs polygons et extraits les arêtes (deux points) regroupés par Ring
* @param Polygon
* @return Retourne un OGRMultiLineString par Ring : un LineString est composé de seulement deux points
*/
std::vector<OGRMultiLineString*> GetLineStringsFromPolygon(OGRPolygon* Polygon)
{
	std::vector<OGRMultiLineString*> ListRing;

	OGRLinearRing* Ring = Polygon->getExteriorRing();
	OGRMultiLineString* MultiLS = new OGRMultiLineString;

	for(int j = 0; j < Ring->getNumPoints() - 1; ++j)
	{
		OGRLineString* LS = new OGRLineString();
		LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
		LS->addPoint(Ring->getX(j+1), Ring->getY(j+1), Ring->getZ(j+1));
		MultiLS->addGeometryDirectly(LS);
	}
	ListRing.push_back(MultiLS);

	for(int k = 0; k < Polygon->getNumInteriorRings(); ++k)
	{
		Ring = Polygon->getInteriorRing(k);
		delete MultiLS;
		MultiLS = new OGRMultiLineString;

		for(int j = 0; j < Ring->getNumPoints() - 1; ++j)
		{
			OGRLineString* LS = new OGRLineString();
			LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
			LS->addPoint(Ring->getX(j+1), Ring->getY(j+1), Ring->getZ(j+1));
			MultiLS->addGeometryDirectly(LS);
		}
		ListRing.push_back(MultiLS);
	}

	return ListRing;
}
std::vector<OGRMultiLineString*> GetLineStringsFromMultiPolygon(OGRGeometryCollection* MultiPolygon)
{
	std::vector<OGRMultiLineString*> ListRing;

	for(int i = 0; i < MultiPolygon->getNumGeometries(); ++i)
	{
		OGRPolygon* Polygon = dynamic_cast<OGRPolygon*>(MultiPolygon->getGeometryRef(i));
		if(Polygon == nullptr)
			continue;

		OGRLinearRing* Ring = Polygon->getExteriorRing();
		OGRMultiLineString* MultiLS = new OGRMultiLineString;

		for(int j = 0; j < Ring->getNumPoints() - 1; ++j)
		{
			OGRLineString* LS = new OGRLineString();
			LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
			LS->addPoint(Ring->getX(j+1), Ring->getY(j+1), Ring->getZ(j+1));
			MultiLS->addGeometryDirectly(LS);
		}
		ListRing.push_back(MultiLS);

		for(int k = 0; k < Polygon->getNumInteriorRings(); ++k)
		{
			Ring = Polygon->getInteriorRing(k);
			delete MultiLS;
			MultiLS = new OGRMultiLineString;

			for(int j = 0; j < Ring->getNumPoints() - 1; ++j)
			{
				OGRLineString* LS = new OGRLineString();
				LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
				LS->addPoint(Ring->getX(j+1), Ring->getY(j+1), Ring->getZ(j+1));
				MultiLS->addGeometryDirectly(LS);
			}
			ListRing.push_back(MultiLS);
		}
	}

	return ListRing;
}
std::vector<OGRMultiPoint*> GetPointsFromPolygon(OGRPolygon* Polygon)
{
	std::vector<OGRMultiPoint*> ListPoints;
	if(Polygon == nullptr)
		return  ListPoints;

	OGRLinearRing* ExtRing = Polygon->getExteriorRing();

	OGRMultiPoint* PointsExtRing = new OGRMultiPoint;

	for(int i = 0; i < ExtRing->getNumPoints(); ++i)
	{
		OGRPoint* P = new OGRPoint;
		ExtRing->getPoint(i, P);
		PointsExtRing->addGeometryDirectly(P);
	}
	ListPoints.push_back(PointsExtRing);

	for(int j = 0; j < Polygon->getNumInteriorRings(); ++j)
	{
		OGRMultiPoint* PointsIntRing = new OGRMultiPoint;

		OGRLinearRing* IntRing = Polygon->getInteriorRing(j);
		for(int i = 0; i < IntRing->getNumPoints(); ++i)
		{
			OGRPoint* P = new OGRPoint;
			ExtRing->getPoint(i, P);
			PointsIntRing->addGeometryDirectly(P);
		}
		ListPoints.push_back(PointsIntRing);
	}

	return ListPoints;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Découpe le fichier CityGML en un ensemble de tuiles dont la taille est définie en entrée.
* @param Tile Contient les données du fichier CityGML ouvert : il doit contenir un ensemble de bâtiments LOD2 ou du terrain
* @param TexturesList : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie;
* @param MinTile : Coordonnée du coin bas gauche de la tuile
* @param MaxTile : Coordonnée du coin haut droit de la tuile
*/
citygml::CityModel* TileCityGML(vcity::Tile* Tile, std::vector<TextureCityGML*>* TexturesList, TVec2d MinTile, TVec2d MaxTile)
{
	citygml::CityModel* Tuile = new citygml::CityModel;

	citygml::CityModel* Model = Tile->getCityModel();

	OGRPolygon* PolyTile = new OGRPolygon;
	OGRLinearRing* RingTile = new OGRLinearRing;
	RingTile->addPoint(MinTile.x, MinTile.y);
	RingTile->addPoint(MinTile.x, MaxTile.y);
	RingTile->addPoint(MaxTile.x, MaxTile.y);
	RingTile->addPoint(MaxTile.x, MinTile.y);
	RingTile->addPoint(MinTile.x, MinTile.y);
	PolyTile->addRingDirectly(RingTile);

	OGRLineString* WestLine = new OGRLineString;
	OGRLineString* EastLine = new OGRLineString;
	OGRLineString* NorthLine = new OGRLineString;
	OGRLineString* SouthLine = new OGRLineString;

	WestLine->addPoint(MinTile.x, MinTile.y);
	WestLine->addPoint(MinTile.x, MaxTile.y);
	EastLine->addPoint(MaxTile.x, MinTile.y);
	EastLine->addPoint(MaxTile.x, MaxTile.y);
	NorthLine->addPoint(MinTile.x, MaxTile.y);
	NorthLine->addPoint(MaxTile.x, MaxTile.y);
	SouthLine->addPoint(MinTile.x, MinTile.y);
	SouthLine->addPoint(MaxTile.x, MinTile.y);

	for(citygml::CityObject* obj : Model->getCityObjectsRoots())
	{
		if(obj->getType() == citygml::COT_Building)
		{
			std::string Name = obj->getId();
			citygml::CityObject* BuildingCO = new citygml::Building(Name);
			citygml::CityObject* RoofCO = new citygml::RoofSurface(Name + "_Roof");
			citygml::Geometry* Roof = new citygml::Geometry(Name + "_RoofGeometry", citygml::GT_Roof, 2);
			citygml::CityObject* WallCO = new citygml::WallSurface(Name + "_Wall");
			citygml::Geometry* Wall = new citygml::Geometry(Name + "_WallGeometry", citygml::GT_Wall, 2);

			int cptPolyRoof = 0; //Compteur de polygones représentant un Roof du bâtiment courant (pour avoir des noms différents)
			int cptPolyWall = 0; //Compteur de polygones représentant un Wall du bâtiment courant (pour avoir des noms différents)

			for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
			{
				if(object->getType() == citygml::COT_RoofSurface)
				{
					for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
						{
							OGRLinearRing * OgrRing = new OGRLinearRing;
							for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								OgrRing->addPoint(Point.x, Point.y, Point.z);

							std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

							OgrRing->closeRings();
							if(OgrRing->getNumPoints() > 3)
							{
								OGRPolygon * OgrPoly = new OGRPolygon;
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid() && OgrPoly->Intersects(PolyTile))
								{
									bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

									std::string Url;
									citygml::Texture::WrapMode WrapMode;
									std::vector<std::vector<TVec2f>> TexUVout;
									if(HasTexture)
									{
										Url = PolygonCityGML->getTexture()->getUrl();
										WrapMode = PolygonCityGML->getTexture()->getWrapMode();
									}

									OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, PolyTile, &TexUV, &TexUVout);

									if(CutPoly != nullptr)
									{
										if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
										{
											if(((OGRPolygon*)CutPoly)->get_Area() < Precision_Vect) //Pour éliminer les polygons plats
											{
												delete OgrPoly;
												delete CutPoly;
												continue;
											}

											citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_Roof_" + std::to_string(cptPolyRoof));
											Roof->addPolygon(GMLPoly);
											if(HasTexture)
											{
												TexturePolygonCityGML Poly;

												Poly.Id = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Poly";
												Poly.IdRing = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Ring";
												Poly.TexUV = TexUVout.at(0);

												bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList)
												{
													if(Tex->Url == Url)
													{
														URLTest = true;
														Tex->ListPolygons.push_back(Poly);
														break;
													}
												}
												if(!URLTest)
												{
													TextureCityGML* Texture = new TextureCityGML;
													Texture->Wrap = WrapMode;
													Texture->Url = Url;
													Texture->ListPolygons.push_back(Poly);
													TexturesList->push_back(Texture);
												}
											}
											++cptPolyRoof;
										}
										else
										{
											OGRMultiPolygon* CutMultiPoly = dynamic_cast<OGRMultiPolygon*>(CutPoly);
											if(CutMultiPoly != nullptr)
											{
												for(int i = 0; i < CutMultiPoly->getNumGeometries(); ++i)
												{
													if(((OGRPolygon*)CutMultiPoly->getGeometryRef(i))->get_Area() < Precision_Vect)
														continue;

													citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutMultiPoly->getGeometryRef(i), Name + "_Roof_" + std::to_string(cptPolyRoof));
													Roof->addPolygon(GMLPoly);
													if(HasTexture)
													{
														TexturePolygonCityGML Poly;

														Poly.Id = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Poly";
														Poly.IdRing = Name + "_Roof_" + std::to_string(cptPolyRoof) + "_Ring";
														Poly.TexUV = TexUVout.at(i);

														bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
														for(TextureCityGML* Tex: *TexturesList)
														{
															if(Tex->Url == Url)
															{
																URLTest = true;
																Tex->ListPolygons.push_back(Poly);
																break;
															}
														}
														if(!URLTest)
														{
															TextureCityGML* Texture = new TextureCityGML;
															Texture->Wrap = WrapMode;
															Texture->Url = Url;
															Texture->ListPolygons.push_back(Poly);
															TexturesList->push_back(Texture);
														}
													}
													++cptPolyRoof;
												}
											}
										}
									}
								}
							}
						}
					}
				}
				else if(object->getType() == citygml::COT_WallSurface)
				{
					for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
						{
							OGRLinearRing * WallRing = new OGRLinearRing;

							std::vector<TVec3d> PointsWall = PolygonCityGML->getExteriorRing()->getVertices();
							for(int i = 0; i < PointsWall.size(); ++i) //Le premier point n'est pas répété à la fin
							{
								TVec3d Point = PointsWall.at(i);
								WallRing->addPoint(Point.x, Point.y, Point.z);
							}
							WallRing->closeRings();

							if(WallRing->getNumPoints() > 3)
							{
								OGRPolygon * WallPoly = new OGRPolygon;
								WallPoly->addRingDirectly(WallRing);
								if(WallPoly->Distance(PolyTile) < Precision_Vect) //Le polygon du Wall semble intersecter le Roof du Shape, on va donc l'ajouter à ce bâtiment.
								{
									bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

									std::string Url;
									citygml::Texture::WrapMode WrapMode;
									std::vector<TVec2f> TexUV;

									if(HasTexture)
									{
										Url = PolygonCityGML->getTexture()->getUrl();
										WrapMode = PolygonCityGML->getTexture()->getWrapMode();
										TexUV = PolygonCityGML->getTexCoords();
										TexUV.push_back(PolygonCityGML->getTexCoords().at(0));
									}

									std::vector<TVec2f> TexUVWall;

									OGRPolygon* WallPolyRes = new OGRPolygon; //Contiendra le polygon du Wall que l'on aura limité à la tuile
									OGRLinearRing* WallRingRes = new OGRLinearRing;

									for(int i = 0; i < WallRing->getNumPoints() - 1; ++i)
									{
										OGRPoint* WallPoint1 = new OGRPoint(WallRing->getX(i), WallRing->getY(i), WallRing->getZ(i));
										OGRPoint* WallPoint2 = new OGRPoint(WallRing->getX(i+1), WallRing->getY(i+1), WallRing->getZ(i+1));

										if(WallPoint1->Distance(PolyTile) < Precision_Vect && WallPoint2->Distance(PolyTile) < Precision_Vect) //Les deux points de la ligne sont dans la tuile, rien à faire
										{
											WallRingRes->addPoint(WallPoint1);
											WallRingRes->addPoint(WallPoint2);
											if(HasTexture)
											{
												TexUVWall.push_back(TexUV.at(i));
												TexUVWall.push_back(TexUV.at(i+1));
											}
										}
										else
										{
											OGRLineString* Line = new OGRLineString;
											Line->addPoint(WallPoint1);
											Line->addPoint(WallPoint2);
											if(WallPoint1->Distance(PolyTile) < Precision_Vect) //Seul le premier point est dans la tuile, il faut donc calculer le second
											{
												WallRingRes->addPoint(WallPoint1);
												if(HasTexture)
													TexUVWall.push_back(TexUV.at(i));
												if(Line->Intersects(EastLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(EastLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												else if(Line->Intersects(WestLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(WestLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												else if(Line->Intersects(NorthLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(NorthLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												else if(Line->Intersects(SouthLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(SouthLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
											}
											else if(WallPoint2->Distance(PolyTile) < Precision_Vect)
											{
												if(Line->Intersects(EastLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(EastLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												else if(Line->Intersects(WestLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(WestLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												else if(Line->Intersects(NorthLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(NorthLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												else if(Line->Intersects(SouthLine))
												{
													OGRPoint* Point = dynamic_cast<OGRPoint*>(Line->Intersection(SouthLine));
													if(Point != nullptr)
													{
														WallRingRes->addPoint(Point);
														if(HasTexture)
															TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(Point->getX(), Point->getY(), Point->getZ())));
													}
													delete Point;
												}
												WallRingRes->addPoint(WallPoint2);
												if(HasTexture)
													TexUVWall.push_back(TexUV.at(i+1));
											}
											else if(Line->Intersects(PolyTile)) // Cas où la ligne est dans un coin, aucun des points n'est dans la tuile mais une partie de la ligne y est
											{
												OGRLineString* InterLine = dynamic_cast<OGRLineString*>(Line->Intersection(SouthLine));
												if(InterLine != nullptr)
												{
													//double Z1 = WallPoint1->getZ() + (WallPoint2->getZ() - WallPoint1->getZ())*(InterLine->getX(0) - WallPoint1->getX())/(WallPoint2->getX() - WallPoint1->getX()); //Calcul de la coordonnée z car l'intersection de GDAL est 2D
													//double Z2 = WallPoint1->getZ() + (WallPoint2->getZ() - WallPoint1->getZ())*(InterLine->getX(1) - WallPoint1->getX())/(WallPoint2->getX() - WallPoint1->getX()); //Calcul de la coordonnée z car l'intersection de GDAL est 2D

													WallRingRes->addPoint(InterLine->getX(0), InterLine->getY(0), InterLine->getZ(0));
													WallRingRes->addPoint(InterLine->getX(1), InterLine->getY(1), InterLine->getZ(1));
													if(HasTexture)
													{
														TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(InterLine->getX(0), InterLine->getY(0), InterLine->getZ(0))));
														TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(InterLine->getX(1), InterLine->getY(1), InterLine->getZ(1))));
													}
												}
											}
										}
										delete WallPoint1;
										delete WallPoint2;
									}

									if(WallRingRes != nullptr && WallRingRes->getNumPoints() > 0)
									{
										WallRingRes->closeRings();
										WallPolyRes->addRingDirectly(WallRingRes);
										citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)WallPolyRes, Name + "_Wall_" + std::to_string(cptPolyWall));
										delete WallPolyRes;

										Wall->addPolygon(GMLPoly);

										if(HasTexture)
										{
											TexturePolygonCityGML Poly;

											Poly.Id = Name + "_Wall_" + std::to_string(cptPolyWall) + "_Poly";
											Poly.IdRing = Name + "_Wall_" + std::to_string(cptPolyWall) + "_Ring";
											Poly.TexUV = TexUVWall;

											bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
											for(TextureCityGML* Tex: *TexturesList)
											{
												if(Tex->Url == Url)
												{
													URLTest = true;
													Tex->ListPolygons.push_back(Poly);
													break;
												}
											}
											if(!URLTest)
											{
												TextureCityGML* Texture = new TextureCityGML;
												Texture->Wrap = WrapMode;
												Texture->Url = Url;
												Texture->ListPolygons.push_back(Poly);
												TexturesList->push_back(Texture);
											}
										}
										++cptPolyWall;
									}
								}
								delete WallPoly;
							}
							else
								delete WallRing;
						}
					}
				}
			}

			bool test = false;
			if(Roof->getPolygons().size() > 0)
			{
				RoofCO->addGeometry(Roof);
				Tuile->addCityObject(RoofCO);
				BuildingCO->insertNode(RoofCO);
				test = true;
			}
			if(Wall->getPolygons().size() > 0)
			{
				WallCO->addGeometry(Wall);
				Tuile->addCityObject(WallCO);
				BuildingCO->insertNode(WallCO);
				test = true;
			}
			if(test)
			{
				Tuile->addCityObject(BuildingCO);
				Tuile->addCityObjectAsRoot(BuildingCO);
			}
		}
	}

	delete EastLine;
	delete WestLine;
	delete NorthLine;
	delete SouthLine;
	delete PolyTile;

	return Tuile;
}

////////////////////////////////////////////////////////////////////////////////