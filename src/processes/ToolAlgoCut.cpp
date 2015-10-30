#include "ToolAlgoCut.hpp"

/**
* @brief Convertit un OGRPolygon* en citygml::Polygon*
* @param OGRPoly OGRPolygon* à convertir.
* @param Name Nom du Polygon CityGML à retourner.
*/
citygml::Polygon * ConvertOGRPolytoGMLPoly(OGRPolygon* OGRPoly, std::string Name)
{
	citygml::Polygon * Poly = new citygml::Polygon(Name + "_Poly");
	citygml::LinearRing * Ring = new citygml::LinearRing(Name + "_Ring", true);

	OGRLinearRing * ExtRing = OGRPoly->getExteriorRing();

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

	for(int i = 0; i < OGRPoly->getNumInteriorRings(); ++i)
	{
		citygml::LinearRing * IntRingGML = new citygml::LinearRing(Name + "_IntRing_" + std::to_string(i), false);

		OGRLinearRing * IntRing = OGRPoly->getInteriorRing(i);

		for(int j = 0; j < IntRing->getNumPoints() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
		{
			OGRPoint * point = new OGRPoint;
			IntRing->getPoint(j, point);
			double x = point->getX();
			double y = point->getY();
			double z = point->getZ();
			delete point;

			IntRingGML->addVertex(TVec3d(x, y, z));
		}

		Poly->addRing(IntRingGML);
	}

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
* @brief ChangePointsOrderForNormal : Modifie l'orientation du polygone pour que sa normale soit orientée vers le haut (pour MNT et Roof).
* @param Ring : Contient les points formant le polygone
* @param Tex : Contient les coordonnées de textures liées aux points de Ring, il faut également modifier leur ordre si on veut conserver l'information de texture
*/
void ChangePointsOrderForNormal(OGRLinearRing* Ring, std::vector<TVec2f>* Tex)
{
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

	double NormZ = AB.x * AC.y - AB.y * AC.x;

	if(NormZ >= 0) //Le polygone est bien orienté pour que sa normale soit vers le haut, il n'y a donc rien à changer.
		return;

	OGRLinearRing* RingTmp = (OGRLinearRing*) Ring->clone();
	std::vector<TVec2f> TexTmp;// = *Tex;

	TexTmp.insert(TexTmp.end(),Tex->begin(),Tex->end());

	Tex->clear();
	delete Ring;
	Ring = new OGRLinearRing;

	for(int i = 0; i < TexTmp.size(); ++i)
	{
		Tex->push_back(TexTmp.at(TexTmp.size() - 1 - i));
		Ring->addPoint(RingTmp->getX(TexTmp.size() - 1 - i), RingTmp->getY(TexTmp.size() - 1 - i), RingTmp->getZ(TexTmp.size() - 1 - i));
	}

	delete RingTmp;
}

/**
* @brief Découpe un polygon 3D GMLPoly suivant un second polygon 2D BuildingShp. Il faut que le polygon découpé soit encore en 3d.
* @param GMLPoly représente le premier polygon 3D (issu du fichier CityGML)
* @param BuildingShp représente le second polygon 2D qui va découper GMLPoly
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

	if(AB.x == 0) //Pour le calcul de s et t, cela pose problème donc on intervertit B et C pour avoir un AB.x != 0. En effet, AB.x et AC.x ne peuvent tous deux être égaux à 0 sinon le triangle serait plat.
	{
		TVec2f uvTemp = uvAB;
		TVec3d VecTemp = AB;

		uvAB = uvAC;
		AB = AC;
		uvAC = uvTemp;
		AC = VecTemp;

		uvTemp = uvB;
		VecTemp = B;
		uvB = uvC;
		B = C;
		uvC = uvTemp;
		C = VecTemp;
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

		ChangePointsOrderForNormal(ResExtRing, &uvPolyInter);

		ResPoly->addRingDirectly(ResExtRing);

		for(int r = 0; r < PolyInter->getNumInteriorRings(); ++r)
		{
			//std::cout << "INTERIOR RING" << std::endl;

			OGRLinearRing* InterIntRing = PolyInter->getInteriorRing(r);
			OGRLinearRing* ResIntRing = new OGRLinearRing;

			for(int i = 0; i < InterIntRing->getNumPoints(); ++i)
			{
				OGRPoint* P = new OGRPoint;
				InterIntRing->getPoint(i, P);

				TVec3d M;
				M.x = P->getX();
				M.y = P->getY();

				double s, t;

				t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
				s = (M.x - A.x - t * AC.x) / AB.x;

				M.z = A.z + s * AB.z + t * AC.z;

				ResIntRing->addPoint(M.x, M.y, M.z);

				uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y));

				delete P;
			}
			ResPoly->addRingDirectly(ResIntRing);
		}

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

					ChangePointsOrderForNormal(ResExtRing, &uvPolyInter);

					ResPoly->addRingDirectly(ResExtRing);

					for(int r = 0; r < PolyInter->getNumInteriorRings(); ++r)
					{
						//std::cout << "INTERIOR RING2" << std::endl;

						OGRLinearRing* InterIntRing = PolyInter->getInteriorRing(r);
						OGRLinearRing* ResIntRing = new OGRLinearRing;

						for(int i = 0; i < InterIntRing->getNumPoints(); ++i)
						{
							OGRPoint* P = new OGRPoint;
							InterIntRing->getPoint(i, P);

							TVec3d M;
							M.x = P->getX();
							M.y = P->getY();

							double s, t;

							t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
							s = (M.x - A.x - t * AC.x) / AB.x;

							M.z = A.z + s * AB.z + t * AC.z;

							ResIntRing->addPoint(M.x, M.y, M.z);

							uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquées sans déformation.

							delete P;
						}
						ResPoly->addRingDirectly(ResIntRing);
					}
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