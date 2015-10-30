#include "ChangeDetection.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
#include "ExportToShape.hpp"
#include "lodsmanagement.hpp"
#include <iomanip>

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Calcule la distance de Hausdorff unidirectionnelle entre un nuage de points et un ensemble de triangles
* @param GeoPoints Correspond à la géométrie contenant le nuage de points et que l'on va projeter sur la seconde géométrie
* @param Geo Correspond aux triangles sur lesquels seront projetés les points
*/
double Hausdorff(OGRMultiPolygon * GeoPoints, OGRMultiPolygon * Geo)
{
    std::vector<OGRPoint *> Points;

    for(int i = 0; i < GeoPoints->getNumGeometries(); ++i)
    {
        OGRPolygon * Poly = (OGRPolygon *)GeoPoints->getGeometryRef(i);
        OGRLinearRing * Ring = Poly->getExteriorRing();
        for(int j = 0; j < Ring->getNumPoints(); ++j)
        {
            OGRPoint * Point = new OGRPoint;
            Ring->getPoint(j, Point);
            Points.push_back(Point);
        }
    }

	for(int i = 0; i < Geo->getNumGeometries(); ++i) //On parcourt tous les polygones pour transformer les éventuels rectangles (ou autres polygones convexes à 4 côtés) en deux triangles : temporaire, ne fonctionne que pour ces polygones précis
	{
		OGRPolygon * Poly = (OGRPolygon *)Geo->getGeometryRef(i);
		OGRLinearRing * Ring = Poly->getExteriorRing();
		if(Ring->getNumPoints() == 5)
		{
			//SaveGeometrytoShape("Rectangle_" + std::to_string(i) + ".shp", Poly);
			OGRPoint * P1 = new OGRPoint;
            OGRPoint * P2 = new OGRPoint;
            OGRPoint * P3 = new OGRPoint;
			OGRPoint * P4 = new OGRPoint;
            Ring->getPoint(0, P1);
            Ring->getPoint(1, P2);
            Ring->getPoint(2, P3);
			Ring->getPoint(3, P4);
			OGRLinearRing * Ring1 = new OGRLinearRing;
			OGRLinearRing * Ring2 = new OGRLinearRing;
			Ring1->addPoint(P1);
			Ring1->addPoint(P2);
			Ring1->addPoint(P3);
			Ring1->addPoint(P1);
			Ring2->addPoint(P1);
			Ring2->addPoint(P3);
			Ring2->addPoint(P4);
			Ring2->addPoint(P1);

			OGRPolygon * Poly1 = new OGRPolygon;
			OGRPolygon * Poly2 = new OGRPolygon;
			Poly1->addRingDirectly(Ring1);
			Poly2->addRingDirectly(Ring2);
			//SaveGeometrytoShape("Rectangle_" + std::to_string(i) + "_Triangle1.shp", Poly1);
			//SaveGeometrytoShape("Rectangle_" + std::to_string(i) + "_Triangle2.shp", Poly2);
			Geo->removeGeometry(i);
			--i;
			Geo->addGeometry(Poly1);
			Geo->addGeometry(Poly2);
		}
	}

    double D = 0;
    for(size_t i = 0; i < Points.size(); ++i)
    {
		int J = -1;
        double D_1 = 10000;
        OGRPoint * OgrP0 = Points.at(i);
        TVec3d P0(OgrP0->getX(),OgrP0->getY(), OgrP0->getZ());
        for(int j = 0; j < Geo->getNumGeometries(); ++j)
        {
            double D_2;
            OGRPolygon * PolyTriangle = (OGRPolygon *)Geo->getGeometryRef(j);
            OGRLinearRing * Triangle = PolyTriangle->getExteriorRing();
            if(Triangle->getNumPoints() > 4 || Triangle->get_Area() < 0.01)//Si la géometry courante n'est pas un triangle
            {
                continue;
            }
            OGRPoint * OgrP1 = new OGRPoint; //Point du triangle
            OGRPoint * OgrP2 = new OGRPoint;
            OGRPoint * OgrP3 = new OGRPoint;
            Triangle->getPoint(0, OgrP1);
            Triangle->getPoint(1, OgrP2);
            Triangle->getPoint(2, OgrP3);

            TVec3d P1(OgrP1->getX(),OgrP1->getY(), OgrP1->getZ());
            TVec3d P2(OgrP2->getX(),OgrP2->getY(), OgrP2->getZ());
            TVec3d P3(OgrP3->getX(),OgrP3->getY(), OgrP3->getZ());

            delete OgrP1;
            delete OgrP2;
            delete OgrP3;

            TVec3d P1P0 = P0 - P1; //Vecteur P1P0
            TVec3d P2P0 = P0 - P2;
            TVec3d P3P0 = P0 - P3;

            double nP1P0 = P1P0.length(); //Norme du vecteur P1P0
            double nP2P0 = P2P0.length();
            double nP3P0 = P3P0.length();

            if(nP1P0 == 0 || nP2P0 == 0 || nP3P0 == 0) // Si le point P0 est confondu avec l'un des points du triangle
            {
				J = j;
                D_1 = 0;
                break;
            }

            TVec3d P1P2 = P2 - P1;//Vecteur P1P2
            TVec3d P1P3 = P3 - P1;
            TVec3d P2P3 = P3 - P2;

            TVec3d Np(P1P2.y * P1P3.z - P1P2.z * P1P3.y, P1P2.z * P1P3.x - P1P2.x * P1P3.z, P1P2.x * P1P3.y - P1P2.y * P1P3.x); //Normal du triangle
            double nNp = Np.length();

            double cosa = (P1P0.x * Np.x + P1P0.y * Np.y + P1P0.z * Np.z)/(nP1P0 * nNp); //Calcul du cosinus de langle a entre Np et P1P0

            double nP0P0_ = nP1P0 * cosa;
            TVec3d P0P0_(- nP0P0_ * Np.x / nNp, - nP0P0_ * Np.y / nNp, - nP0P0_ * Np.z / nNp); //Vecteur P0P0_, P0_ étant le projeté de P0 sur le plan du triangle

            TVec3d P0_(P0.x + P0P0_.x, P0.y + P0P0_.y, P0.z + P0P0_.z); // Position du projeté de P0 sur le plan du triangle

            double s, t;//Coordonnées barycentriques du point P0_ par rapport au point P1 et aux vecteurs P1P2 et P1P3

            t = (P1.y * P1P2.x - P1.x * P1P2.y + P1P2.y * P0_.x - P1P2.x * P0_.y) / (P1P2.y * P1P3.x - P1P2.x * P1P3.y);
            s = (P0_.x - P1.x - t * P1P3.x) / P1P2.x;

            if(t >= 0 && s >= 0 && t + s <= 1)//Le projeté est dans le triangle
            {
                D_2 = nP0P0_;
                if(D_1 > D_2)
				{
					J = j;
                    D_1 = D_2;
				}
            }
            else//Le projeté est en dehors du triangle
            {
                //On va donc le comparer aux trois arrêtes du triangle en le projetant à nouveau sur celles-ci
                TVec3d P0_P1 = P1 - P0_;//Vecteur P0_P1
                TVec3d P0_P2 = P2 - P0_;
                TVec3d P0_P3 = P3 - P0_;
                double nP0_P1 = P0_P1.length();//Norme du vecteur P0_P1
                double nP0_P2 = P0_P2.length();

                //Sur P1P2 :
                TVec3d Temp(P0_P2.y * P0_P1.z - P0_P2.z * P0_P1.y, P0_P2.z * P0_P1.x - P0_P2.x * P0_P1.z, P0_P2.x * P0_P1.y - P0_P2.y * P0_P1.x);
                TVec3d R(Temp.y * P1P2.z - Temp.z * P1P2.y, Temp.z * P1P2.x - Temp.x * P1P2.z, Temp.x * P1P2.y - Temp.y * P1P2.x); //Direction de P0_ -> P0__
                double nR = R.length();

                double cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z)/(nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

                double nP0_P0__ = nP0_P1 * cosg;
                TVec3d P0_P0__(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
                TVec3d P0__(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

                double u = (P0__.x - P1.x) / (P1P2.x); //Position relative de P0__ sur le segment P1P2

                if(u <= 0)
                    D_2 = nP1P0; //P0 est le plus proche de P1
                else if(u >= 1)
                    D_2 = nP2P0; //P0 est le plus proche de P2
                else
                    D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

                if(D_1 > D_2)
				{
					J = j;
                    D_1 = D_2;
				}

                //Sur P1P3 :
                Temp = TVec3d(P0_P3.y * P0_P1.z - P0_P3.z * P0_P1.y, P0_P3.z * P0_P1.x - P0_P3.x * P0_P1.z, P0_P3.x * P0_P1.y - P0_P3.y * P0_P1.x);
                R = TVec3d(Temp.y * P1P3.z - Temp.z * P1P3.y, Temp.z * P1P3.x - Temp.x * P1P3.z, Temp.x * P1P3.y - Temp.y * P1P3.x); //Direction de P0_ -> P0__
                nR = R.length();

                cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z)/(nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

                nP0_P0__ = nP0_P1 * cosg;
                P0_P0__ = TVec3d(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
                P0__ = TVec3d(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

                u = (P0__.x - P1.x) / (P1P3.x); //Position relative de P0__ sur le segment P1P3

                if(u <= 0)
                    D_2 = nP1P0; //P0 est le plus proche de P1
                else if(u >= 1)
                    D_2 = nP3P0; //P0 est le plus proche de P3
                else
                    D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

                if(D_1 > D_2)
				{
					J = j;
                    D_1 = D_2;
				}

                //Sur P2P3 :
                Temp = TVec3d(P0_P3.y * P0_P2.z - P0_P3.z * P0_P2.y, P0_P3.z * P0_P2.x - P0_P3.x * P0_P2.z, P0_P3.x * P0_P2.y - P0_P3.y * P0_P2.x);
                R = TVec3d(Temp.y * P2P3.z - Temp.z * P2P3.y, Temp.z * P2P3.x - Temp.x * P2P3.z, Temp.x * P2P3.y - Temp.y * P2P3.x); //Direction de P0_ -> P0__
                nR = R.length();

                cosg = (P0_P2.x * R.x + P0_P2.y * R.y + P0_P2.z * R.z)/(nP0_P2 * nR); //Calcul du cosinus de langle g entre R et P0_P2

                nP0_P0__ = nP0_P2 * cosg;
                P0_P0__ = TVec3d(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__étant le projeté de P0_ sur l'arrête courante du triangle
                P0__ = TVec3d(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projeté de P0_ sur l'arrête du triangle

                u = (P0__.x - P2.x) / (P2P3.x); //Position relative de P0__ sur le segment P2P3

                if(u <= 0)
                    D_2 = nP2P0; //P0 est le plus proche de P2
                else if(u >= 1)
                    D_2 = nP3P0; //P0 est le plus proche de P3
                else
                    D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

                if(D_1 > D_2)
				{
                    D_1 = D_2;
					J = j;
				}
            }
        }
		/*if(D_1 > 5)
		{
			std::cout << "Erreur = " << D_1 << std::endl;
			std::cout << P0 <<std::endl;
			SaveGeometrytoShape("ERREUR_" + std::to_string(i) + ".shp", (OGRPolygon *)Geo->getGeometryRef(J));
		}*/
        if(D_1 > D)
            D = D_1;
    }
	//int a;
	//std::cin >> a;
    return D;
}

/**
* @brief Calcule la distance de Hausdorff entre deux bâtiments composés de triangles : méthode basée sur "3D Distance from a Point to a Triangle", Mark W.Jones, 1995 (DistancePointTriangle.pdf).
* @param Geo1 Geometrie correspondant au premier bâtiment
* @param Geo2 Geometrie correspondant au second bâtiment
*/
double DistanceHausdorff(OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2)
{
    double D12 = 0;//Distance de Geo1 à Geo2
    double D21 = 0;//Distance de Geo2 à Geo1

    D12 = Hausdorff(Geo1, Geo2);
    D21 = Hausdorff(Geo2, Geo1);
	
    return std::max(D12, D21);
}

/**
* @brief Projette un polygone 2D Poly2D sur le plan formé par un polygone 3D Poly3D
* @param Poly2D Polygone 2D
* @param Poly3D Polygone 3D
*/
OGRPolygon* ProjectPolyOn3DPlane(OGRPolygon* Poly2D, OGRPolygon * Poly3D)
{
	OGRLinearRing * Ring2D = Poly2D->getExteriorRing();
	OGRLinearRing * Ring3D = Poly3D->getExteriorRing();

	//Il faut prendre trois points A,B,C non alignés de Ring3D pour pouvoir établir l'équation paramétrique du plan 3D formé par ce Polygone
	OGRPoint * A = new OGRPoint;
	OGRPoint * B = new OGRPoint;
	OGRPoint * C = new OGRPoint;

	TVec3d AB;
    TVec3d AC;
    TVec3d AM;

	Ring3D->getPoint(0, A);
	int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe à 1 tant que C n'est pas correctement rempli
	for(int i = 1; i < Ring3D->getNumPoints(); ++i)
	{
		if(test == 0)
		{
			Ring3D->getPoint(i, B);
			if(A != B)
			{
				AB.x = B->getX() - A->getX();
				AB.y = B->getY() - A->getY();
				AB.z = B->getZ() - A->getZ();
				++test;
			}
		}
		else if(test == 1)
		{
			Ring3D->getPoint(i, C);
			if((C->getX() - A->getX()) / (B->getX() - A->getX()) != (C->getY() - A->getY()) / (B->getY() - A->getY())) //C n'est pas aligné avec A et B
			{
				AC.x = C->getX() - A->getX();
				AC.y = C->getY() - A->getY();
				AC.z = C->getZ() - A->getZ();
				++test;
				break;
			}
		}
	}
	if(test != 2)
    {
        std::cout << "Erreur lors de la creation du plan. \n";
        delete A;
        delete B;
		delete C;
        return nullptr;
    }

	OGRLinearRing * Ring = new OGRLinearRing;

	for(int i = 0; i < Ring2D->getNumPoints(); ++i)
	{
		OGRPoint * M = new OGRPoint;
		Ring2D->getPoint(i, M);
		double s,t;

		t = (A->getY() * AB.x - A->getX() * AB.y + AB.y * M->getX() - AB.x * M->getY()) / (AB.y * AC.x - AB.x * AC.y);
		s = (M->getX() - A->getX() - t * AC.x) / AB.x;

		Ring->addPoint(M->getX(), M->getY(), A->getZ() + s * AB.z + t * AC.z);
	}

	OGRPolygon * Poly = new OGRPolygon;
	Poly->addRingDirectly(Ring);

	return Poly;
}
OGRMultiPolygon* ProjectPolyOn3DPlane(OGRMultiPolygon* Poly2D, OGRPolygon * Poly3D)
{
	OGRMultiPolygon* MultiPolygon = new OGRMultiPolygon;
	for(int i = 0; i < Poly2D->getNumGeometries(); ++i)
	{
		OGRPolygon* Poly = (OGRPolygon*)Poly2D->getGeometryRef(i);
		OGRPolygon* Res = ProjectPolyOn3DPlane(Poly, Poly3D);
		MultiPolygon->addGeometryDirectly(Res);
	}
	return MultiPolygon;
}

/**
* @brief Compare deux ensembles de geometries en retournant les liens entre leurs polygones et l'information sur ces liens : si un polygone se retrouve de manière identique dans les deux ensembles de geometries, dans un seul ou s'il a été modifié
* @param Geo1 Premier ensemble de geometries qui ont été unies : deux triangles voisins sont réunis en un rectangle par exemple
* @param Geo2 Second ensemble de geometries qui ont été unies
* @param Geo1P Premier ensemble de geometries non unies : pour un polygone de Geo1, il donne la liste des polygones non unis qui le composent
* @param Geo2P Second ensemble de geometries non unies
*/
std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > CompareBati(std::string Folder, OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2, std::vector<OGRMultiPolygon* > Geo1P, std::vector<OGRMultiPolygon *> Geo2P)
{
    std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Res; //Enregistre les liens entre les polygones. Pour un polygone donnée de Geo1, si il est en lien avec un de Geo2, l'indice sera précédé de -1 ou -2 pour inchangé/changé

    int NbGeo1 = Geo1->getNumGeometries(); //Nb de bâtiments de la date1
    int NbGeo2 = Geo2->getNumGeometries(); //Nb de bâtiments de la date2

    Res.first.resize(NbGeo1);
    Res.second.resize(NbGeo2);

	OGRMultiPolygon* PolyZonesCommunes = new OGRMultiPolygon;

    for(int i = 0; i < NbGeo1; ++i)
    {
        OGRPolygon * Bati1 = (OGRPolygon *)Geo1->getGeometryRef(i)->clone();

        for(int j = 0; j < NbGeo2; ++j)
        {
            OGRPolygon * Bati2 = (OGRPolygon *)Geo2->getGeometryRef(j)->clone();

            if(!Bati1->Intersects(Bati2))
				continue;

			//std::cout << "Avancement de CompareGeos : " << i << " - " << j << std::endl;

            double Area = 0;

			//SaveGeometrytoShape("Bati1", Bati1);
			//SaveGeometrytoShape("Bati2", Bati2);

			OGRGeometry* Intersection = Bati1->Intersection(Bati2);
			//SaveGeometrytoShape("Intersection", Intersection);
            OGRwkbGeometryType Type = Intersection->getGeometryType();

            if(Type == OGRwkbGeometryType::wkbPolygon || Type == OGRwkbGeometryType::wkbPolygon25D) 
            {
                OGRPolygon * tmp = (OGRPolygon *)(Intersection);
                Area = tmp->get_Area();
                delete tmp;
            }
			else if(Type == OGRwkbGeometryType::wkbMultiPolygon || Type == OGRwkbGeometryType::wkbMultiPolygon25D)
			{
				OGRMultiPolygon * tmp = (OGRMultiPolygon *)(Intersection);
                Area = tmp->get_Area();
                delete tmp;
			}
			else if(Type == OGRwkbGeometryType::wkbGeometryCollection || Type == OGRwkbGeometryType::wkbGeometryCollection25D)
			{
				OGRGeometryCollection * tmp = (OGRGeometryCollection *)(Intersection);
                Area = tmp->get_Area();
                delete tmp;
			}
			
            double val1 = (Bati1->get_Area() - Area)/Area;
            double val2 = (Bati2->get_Area() - Area)/Area;

            if(val1 < 0.01 && val2 < 0.01 && Bati1->get_Area() - Area < 5 && Bati2->get_Area() - Area < 5)//Les polygons sont identiques
            {
                if(DistanceHausdorff(Geo1P.at(i), Geo2P.at(j)) < 1)//Si la distance de Hausdorff entre les deux bâtiments est inférieure à 5m.
                {
                    Res.first[i].push_back(-1);
                    Res.second[j].push_back(-1);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
					delete Bati2;
					//std::cout << "Fin 1" << std::endl;
                    continue;
                }
                else//Batiment modifie en "hauteur"
                {
                    Res.first[i].push_back(-2);
                    Res.second[j].push_back(-2);
                    Res.first[i].push_back(j);
                    Res.second[j].push_back(i);
					delete Bati2;
					//std::cout << "Fin 2" << std::endl;
                    continue;
                }
            }
			//SaveGeometrytoShape(std::to_string(i) + "_Bati1.shp", Bati1);
			//SaveGeometrytoShape(std::to_string(i) + "_Bati2.shp", Bati2);
            /*if(val1 < 0.5 && val2 < 0.5)//Le bâtiment a été modifié car les emprises au sol restent suffisament proches
            {
                Res.first[i].push_back(-2);
                Res.second[j].push_back(-2);
                Res.first[i].push_back(j);
                Res.second[j].push_back(i);
				delete Bati2;
				//std::cout << "Fin 3" << std::endl;
                continue;
            }*/
			//// Si on arrive jusqu'ici, les premiers tests disent que les deux bâtiments sont respectivement détruit/construit. Dernier test pour extraire les bâtiments qui ont des parties identiques => bâtiment modifié
			OGRMultiPolygon* ZonesCommunes = new OGRMultiPolygon;

			//std::cout << "ZoneCommune Debut" << std::endl;
			for(int u = 0; u < Geo1P.at(i)->getNumGeometries(); ++u)
			{
				OGRPolygon* Poly1 = (OGRPolygon*)Geo1P.at(i)->getGeometryRef(u);
				if(Poly1 == NULL)
					continue;
				for(int v = 0; v < Geo2P.at(j)->getNumGeometries(); ++v)
				{
					OGRPolygon* Poly2 = (OGRPolygon*)Geo2P.at(j)->getGeometryRef(v);
					if(Poly2 == NULL)
						continue;
                    if(!Poly1->Intersects(Poly2))
						continue;

					OGRGeometry* Inter = Poly1->Intersection(Poly2);
					if(Inter->getGeometryType() != wkbPolygon && Inter->getGeometryType() != wkbPolygon25D && Inter->getGeometryType() != wkbMultiPolygon && Inter->getGeometryType() != wkbMultiPolygon25D)//Il faut que l'intersection existe et soit au moins un polygon
					{
						delete Inter;
						continue;
					}

					//std::cout << Inter->getGeometryName() <<std::endl;
					OGRMultiPolygon* InterMP = new OGRMultiPolygon; //Intersection entre les deux polygones courants sous la forme d'un multipolygon même si il n'y qu'un seul polygone pour éviter de traiter les deux cas par la suite

					if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D)
						InterMP->addGeometry(Inter);
					else if(Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D)
						InterMP = (OGRMultiPolygon*) Inter;

					////// TODO : CES POLYGONES SONT A TRIANGULER POUR AVOIR UNE DISTANCE DE HAUSDORFF VIABLE !!!! //////

					OGRMultiPolygon* InterOnPoly1 = ProjectPolyOn3DPlane(InterMP, Poly1);
					OGRMultiPolygon* InterOnPoly2 = ProjectPolyOn3DPlane(InterMP, Poly2);

					OGRMultiPolygon* MP1 = new OGRMultiPolygon;
					MP1->addGeometry(Poly1);
					OGRMultiPolygon* MP2 = new OGRMultiPolygon;
					MP2->addGeometry(Poly2);

					double d1 = Hausdorff(InterOnPoly1, MP2);
					double d2 = Hausdorff(InterOnPoly2, MP1);

					if(d1 < 0.5 && d2 < 0.5)
					{
						for(int k = 0; k < InterMP->getNumGeometries(); ++k)
						{
							OGRGeometry* InterGeo = InterMP->getGeometryRef(k);

							//SaveGeometrytoShape("A_Inter_" + std::to_string(k) + ".shp", InterGeo);
							//SaveGeometrytoShape("A_Union_" + std::to_string(k) + ".shp", ZonesCommunes);

							OGRGeometry* tmp = ZonesCommunes->Union(InterGeo);

							if(!tmp->IsValid())
							{
								//std::cout << "UNION NON VALIDE " << std::endl;
								continue;
							}
							
							//std::cout << tmp->getGeometryName() << std::endl;
							if(tmp->getGeometryType() == wkbPolygon || tmp->getGeometryType() == wkbPolygon25D)
							{
								delete ZonesCommunes;
								ZonesCommunes = new OGRMultiPolygon;
								ZonesCommunes->addGeometry((OGRPolygon*) tmp);
							}
							else if(tmp->getGeometryType() == wkbMultiPolygon || tmp->getGeometryType() == wkbMultiPolygon25D)
							{
								delete ZonesCommunes;
								ZonesCommunes = (OGRMultiPolygon*) tmp;
							}
							//SaveGeometrytoShape("A_Communes_" + std::to_string(k) + ".shp", ZonesCommunes);
							//int a;
							//std::cin>>a;
						}
					}
					delete MP1;
					delete MP2;
					delete InterMP;
				}
			}

			//std::cout << "ZoneCommune 1" << std::endl;

			if(ZonesCommunes->IsEmpty()) //Si il n'y a pas d'intersection, les lignes suivantes bug donc il faut sortir
			{
				//std::cout << "ZonesCommunes vide" << std::endl;
				continue;
			}
			if(!ZonesCommunes->IsValid())
			{
				//SaveGeometrytoShape(std::to_string(i) + "_Bati1.shp", Bati1);
				//SaveGeometrytoShape(std::to_string(i) + "_Bati2.shp", Bati2);
				//std::cout << "ZonesCommunes non valid" << std::endl;
				continue;
			}
			//std::cout << "ZoneCommune 2" << std::endl;

			//SaveGeometrytoShape("Bati1P.shp", Geo1P.at(i));
			//SaveGeometrytoShape("Bati2P.shp", Geo2P.at(j));
			//SaveGeometrytoShape("BatiCommun.shp", ZonesCommunes);
			//std::cout << ZonesCommunes->getGeometryName() << std::endl;

			//std::cout << "ZoneCommune 3" << std::endl;

			if(ZonesCommunes->getGeometryType() == wkbPolygon || ZonesCommunes->getGeometryType() == wkbPolygon25D)
			{
				OGRPolygon* Poly = (OGRPolygon*) ZonesCommunes;
				if(Poly->get_Area() > 10) //La zone commune est un polygone d'aire supérieure à 10m² -> Bâtiment modifié car une zone est restée identique
				{
					Res.first[i].push_back(-2);
					Res.second[j].push_back(-2);
					Res.first[i].push_back(j);
					Res.second[j].push_back(i);
					PolyZonesCommunes->addGeometry(Poly);
				}
			}
			else if(ZonesCommunes->getGeometryType() == wkbMultiPolygon || ZonesCommunes->getGeometryType() == wkbMultiPolygon25D)
			{
				OGRMultiPolygon* MultiPoly = (OGRMultiPolygon*) ZonesCommunes;
				bool Modified = false; //Passe à true dès qu'un polygon a une aire supérieure au seuil pour seulement remplir PolyZonesCommunes
				for(int k = 0; k < MultiPoly->getNumGeometries(); ++k)
				{
					OGRPolygon* Poly = (OGRPolygon*)MultiPoly->getGeometryRef(k);
					if(Poly->get_Area() > 10) //La zone commune comporte au moins un polygone d'aire supérieure à 10m² -> Bâtiment modifié car une zone est restée identique
					{
						if(!Modified) //Seulement pour avoir un PolyZonesCommunes complet, inutile pour l'algo : à retirer ce test et remettre break pour gain de temps
						{
							Res.first[i].push_back(-2);
							Res.second[j].push_back(-2);
							Res.first[i].push_back(j);
							Res.second[j].push_back(i);
							Modified = true;
						}
						PolyZonesCommunes->addGeometry(Poly);
						//break;
					}
				}
			}
			//std::cout << "ZoneCommune FIN" << std::endl;
            delete Bati2;
        }
        delete Bati1;
        std::cout << "Avancement de CompareGeos : " << i + 1 << " / " << NbGeo1 << "\r" << std::flush;
    }
    std::cout << "\n";

	SaveGeometrytoShape(Folder + "/ZonesCommunes.shp", PolyZonesCommunes);

    return Res;
}

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Compare deux fichiers CityGML d'une même zone obtenus à deux dates différentes en donnant les modifications entre leurs bâtiments
*/
void CompareTiles(std::string Folder, citygml::CityModel* City1, citygml::CityModel* City2)//Version GDAL
{
    std::vector<citygml::CityModel *> tiles;
    tiles.push_back(City1);
    tiles.push_back(City2);

    //On part du principe que le plus vieux est dans le 0 et le plus récent dans le 1
    std::vector<OGRMultiPolygon *> EnveloppeCity[2];//Version avec un ensemble de polygones pour chaque bâtiment (un bâtiment = un polygone distinct après union)
    OGRMultiPolygon * EnveloppeCityU[2];//Version avec des polygones unis pour représenter le bâtiment (ce n'est pas l'ensemble de polygones bruts)

    EnveloppeCityU[0] = new OGRMultiPolygon;//nullptr;
    EnveloppeCityU[1] = new OGRMultiPolygon;//nullptr;

    for(int i = 0; i < 2; ++i)
    {
        citygml::CityModel* model = tiles[i];

        int cpt = 0;

		OGRMultiPolygon * ModelPolygons = new OGRMultiPolygon;//Contient tous les polygones composant le model courant

        for(citygml::CityObject* obj : model->getCityObjectsRoots())
        {
            if(obj->getType() == citygml::COT_Building)
            {
                OGRMultiPolygon* Building = new OGRMultiPolygon;//Version OGR du bâtiment qui va être remplie

                for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
                {
                    if(object->getType() == citygml::COT_RoofSurface)
                    {
                        for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
                        {
                            for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
                            {
                                OGRPolygon * OgrPoly = new OGRPolygon;
                                OGRLinearRing * OgrRing = new OGRLinearRing;

                                for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
                                {
									OgrRing->addPoint(Point.x, Point.y, Point.z);
                                }

                                OgrRing->closeRings();

                                if(OgrRing->getNumPoints() > 3)
                                {
                                    OgrPoly->addRingDirectly(OgrRing);
                                    if(OgrPoly->IsValid())
                                    {
                                        Building->addGeometry(OgrPoly);
										ModelPolygons->addGeometryDirectly(OgrPoly);
                                    }
                                }
                                else
                                    delete OgrRing;
                            }
                        }
                    }
                }

				if(Building->IsEmpty())
				{
					cpt++;
					std::cout << "Avancement tuile " << i+1 << " : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
					continue;
				}

                OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);
                if(EnveloppeCityU[i] == nullptr)
                    EnveloppeCityU[i] = Enveloppe;
                else
                {
                    OGRMultiPolygon * tmp = EnveloppeCityU[i];
                    EnveloppeCityU[i] = (OGRMultiPolygon *)tmp->Union(Enveloppe);
                    delete tmp;
                }
            }

            cpt++;
            std::cout << "Avancement tuile " << i+1 << " : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
        }
        std::cout << std::endl;

		//SaveGeometrytoShape("ModelInit.shp", ModelPolygons);

        //Création de EnveloppeCity : pour chaque bâtiment distinct, il contient la liste des polygones qui le composent. Ces bâtiments ne correspondent pas à ceux du CityGML, mais
        //aux polygones distincts apparus avec les unions successives.

        for(int g = 0; g < EnveloppeCityU[i]->getNumGeometries(); ++g)
        {
            OGRPolygon * CurrBati = (OGRPolygon *)EnveloppeCityU[i]->getGeometryRef(g); //On parcourt chaque "bâtiment" (bâtiment = un polygon isolé après l'union)

			if(!CurrBati->IsValid())
			{
				EnveloppeCityU[i]->removeGeometry(g);
				g--;
				continue;
			}

			OGRMultiPolygon * Bati = new OGRMultiPolygon;//Contiendra liste des polygones pour bâtiment i

			//std::cout << "G = " << g << "/" << EnveloppeCityU[i]->getNumGeometries() << std::endl;
			for(int j = 0; j < ModelPolygons->getNumGeometries(); ++j)//Pour le bâtiment courant, on va chercher dans ModelPolygons quels sont les polygones qui le composent.
            {
                if(ModelPolygons->getGeometryRef(j)->Intersects(CurrBati))//Ce polygone appartient bien à CurrBati
				{
					OGRGeometry * Inter = ModelPolygons->getGeometryRef(j)->Intersection(CurrBati);
					if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D || Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D) //L'intersection n'est pas un simple point
					{
						Bati->addGeometry(ModelPolygons->getGeometryRef(j));
						ModelPolygons->removeGeometry(j);//On le retire de ModelPolygons car il est maintenant associé à CurrBati
						j--;
					}
                }
            }
			EnveloppeCity[i].push_back(Bati);
        }
    }

    std::pair<std::vector<std::vector<int> >, std::vector<std::vector<int> > > Compare = CompareBati(Folder, EnveloppeCityU[0], EnveloppeCityU[1], EnveloppeCity[0], EnveloppeCity[1]);

    OGRMultiPolygon* BatiDetruits = new OGRMultiPolygon;
    OGRMultiPolygon* BatiCrees = new OGRMultiPolygon;
    OGRMultiPolygon* BatiModifies1= new OGRMultiPolygon;
    OGRMultiPolygon* BatiModifies2= new OGRMultiPolygon;
    OGRMultiPolygon* BatiInchanges= new OGRMultiPolygon;

    for(int i = 0; i < EnveloppeCityU[0]->getNumGeometries(); ++i)
    {
        if(Compare.first[i].size() == 0)
            BatiDetruits->addGeometry(EnveloppeCityU[0]->getGeometryRef(i));
        else
        {
			bool modifie = false;
			for(int j = 0; j < Compare.first[i].size(); j+=2)
			{
				if(Compare.first[i][j] == -2)
				{
					modifie = true;
					BatiModifies2->addGeometry(EnveloppeCityU[1]->getGeometryRef(Compare.first[i][j+1]));
				}
			}
			if(modifie)
				BatiModifies1->addGeometry(EnveloppeCityU[0]->getGeometryRef(i));
			else
                BatiInchanges->addGeometry(EnveloppeCityU[1]->getGeometryRef(Compare.first[i][1]));
        }
    }
    for(int i = 0; i < EnveloppeCityU[1]->getNumGeometries(); ++i)
    {
        if(Compare.second[i].size() == 0)
            BatiCrees->addGeometry(EnveloppeCityU[1]->getGeometryRef(i));
    }

	SaveGeometrytoShape(Folder + "/BatisOld.shp", EnveloppeCityU[0]);
    SaveGeometrytoShape(Folder + "/BatisNew.shp", EnveloppeCityU[1]);
    SaveGeometrytoShape(Folder + "/BatisCrees.shp", BatiCrees);
    SaveGeometrytoShape(Folder + "/BatisDetruits.shp", BatiDetruits);
	SaveGeometrytoShape(Folder + "/BatisModifiesOld.shp", BatiModifies1);
    SaveGeometrytoShape(Folder + "/BatisModifiesNew.shp", BatiModifies2);
    SaveGeometrytoShape(Folder + "/BatisInchanges.shp", BatiInchanges);

    delete BatiInchanges;
    delete BatiModifies2;
    delete BatiModifies1;
    delete BatiCrees;
    delete BatiDetruits;

    delete EnveloppeCityU[0];
    delete EnveloppeCityU[1];

    for(auto& it : EnveloppeCity[0]) delete it;
    for(auto& it : EnveloppeCity[1]) delete it;
}
