#include "LinkCityGMLShape.hpp"
#include "lodsmanagement.hpp"
#include "ExportToShape.hpp"
#include "ToolAlgoCut.hpp"
#include <iomanip>
#include <math.h>

#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>

#include "export/exportCityGML.hpp"
////////////////////////////////////////////////////////////////////////////////
typedef std::pair<double,double> Point;
typedef std::vector<Point> Polygon2D;
typedef std::set<Polygon2D> PolySet;
////////////////////////////////////////////////////////////////////////////////
/**
* @brief Projette les toits du CityObject sélectionné sur le plan (xy)
* @param obj CityObject sélectionné
* @param roofProj un set de Polygon, le résultat de la projection
* @param heightmax Enregistre le Zmax des murs du bâtiment
* @param heightmin Enregistre le Zmin des murs du bâtiment
*/
void projectRoof(citygml::CityObject* obj, PolySet &roofProj, double * heightmax, double * heightmin)
{
	if(obj->getType() == citygml::COT_RoofSurface) //Si surface de toit : COT_RoofSurface COT_WallSurface
	{
		std::vector<citygml::Geometry*> geoms = obj->getGeometries();
		std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
		for(; itGeom != geoms.end(); ++itGeom) //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)
		{
			std::vector<citygml::Polygon*> polys = (*itGeom)->getPolygons();
			std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
			for(; itPoly != polys.end(); ++itPoly) //Pour chaque polygone
			{
				Polygon2D poly;
				citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
				const std::vector<TVec3d> vertices = ring->getVertices();
				std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
				for(; itVertices != vertices.end(); ++itVertices)//pour Chaque sommet
				{
					TVec3d point = *itVertices;
					poly.push_back(std::make_pair(point.x, point.y)); //on récupere le point
					if(point.z > *heightmax)
						*heightmax = point.z;

					//std::cout << " (x,y) = (" << point.x<< "," << point.y<< ")" << std::endl;
				}
				roofProj.insert(poly); // on récupere le polygone
			}
		}
	}
	else if(obj->getType() == citygml::COT_WallSurface)
	{
		std::vector<citygml::Geometry*> geoms = obj->getGeometries();
		std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
		for(; itGeom != geoms.end(); ++itGeom) //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)
		{
			std::vector<citygml::Polygon*> polys = (*itGeom)->getPolygons();
			std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
			for(; itPoly != polys.end(); ++itPoly) //Pour chaque polygone
			{
				citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
				const std::vector<TVec3d> vertices = ring->getVertices();
				std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
				for(; itVertices != vertices.end(); ++itVertices)//pour Chaque sommet
				{
					TVec3d point = *itVertices;
					if(point.z < *heightmin || *heightmin == -1)
						*heightmin = point.z;
				}
			}
		}
	}
	citygml::CityObjects cityObjects = obj->getChildren();
	citygml::CityObjects::iterator itObj = cityObjects.begin();
	for(; itObj != cityObjects.end(); ++itObj)
	{
		projectRoof(*itObj,roofProj, heightmax, heightmin);
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Parcourt tous les bâtiments du CityModel, calcule leurs emprises au sol, les stock dans un Vector de Polygon en respectant leur ordre de stockage dans le CityModel. Un seul polygon par emprise au sol : Passage dans SplitBuildingsFromCityGML obligatoire !
* @param model contient les données du CityGML traité
*/
std::vector<OGRPolygon*> GetFootPrintsfromCityGML(citygml::CityModel* Model)
{
	std::vector<OGRPolygon *> ListFootPrints;

	int cpt = 0;
	for(citygml::CityObject* obj : Model->getCityObjectsRoots())
	{
		if(obj->getType() == citygml::COT_Building)
		{
			OGRGeometry* Building = new OGRPolygon;//Version OGR du bâtiment qui va être remplie

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

							OgrRing->closeRings();

							if(OgrRing->getNumPoints() > 3)
							{
								OGRPolygon * OgrPoly = new OGRPolygon;
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid())
								{
									OGRGeometry* tmp = Building;
									Building = tmp->Union(OgrPoly);
									delete tmp;
								}
								delete OgrPoly;
							}
							else
								delete OgrRing;
						}
					}
				}
			}

			if(Building->IsEmpty() || (Building->getGeometryType() != wkbPolygon && Building->getGeometryType() != wkbPolygon25D))
			{
				++cpt;
				std::cout << "Erreur sur GetFootPrintsfromCityGML Batiment " << cpt << std::endl;
				//SaveGeometrytoShape("Building.shp", Building);
				//std::cout << "Avancement etape 1 : " << cpt << "/" << Model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
				ListFootPrints.push_back(nullptr);
				continue;
			}
			OGRPolygon* Envelope = (OGRPolygon*)Building;
			//On retire les InteriorRing avec une aire négligeable qui sont issues des imprécisions des données vectorielles
			if(Envelope->getNumInteriorRings() > 0)
			{
				OGRPolygon* ResBuilding = new OGRPolygon;
				ResBuilding->addRing((OGRLinearRing*)Envelope->getExteriorRing()->clone());
				for(int r = 0; r < Envelope->getNumInteriorRings(); ++r)
				{
					OGRLinearRing* Ring = Envelope->getInteriorRing(r);
					OGRPolygon* tmp = new OGRPolygon;
					tmp->addRing(Ring);
					if(tmp->get_Area() > 10 * Precision_Vect)
						ResBuilding->addRing(Ring);
					delete tmp;
				}
				delete Envelope;
				ListFootPrints.push_back(ResBuilding);
			}
			else
				ListFootPrints.push_back(Envelope);
		}

		++cpt;
		//std::cout << "Avancement etape 1 : " << cpt << "/" << Model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
	}
	//std::cout << std::endl;

	return ListFootPrints;
}

/**
* @brief Parcourt tous les bâtiments du Layer, récupère leurs emprises au sol, les stock dans un Vector de Polygon en respectant leur ordre de stockage dans le Layer. Un seul polygon par emprise au sol.
* @param Layer contient les données du Shapefile traité
*/
std::vector<OGRPolygon*> GetFootPrintsfromShapeFile(OGRLayer* Layer)
{
	std::vector<OGRPolygon *> ListFootPrints;
	OGRFeature *Feature;
	int cpt = 0;
	Layer->ResetReading();
	while((Feature = Layer->GetNextFeature()) != NULL)
	{
		OGRGeometry* Building = Feature->GetGeometryRef();
		if(Building->getGeometryType() == wkbPolygon || Building->getGeometryType() == wkbPolygon25D)
			ListFootPrints.push_back((OGRPolygon *)Building);
		else
		{
			std::cout << "Erreur sur GetFootPrintsfromShapeFile Batiment " << cpt << std::endl;
			ListFootPrints.push_back(nullptr);
		}
		++cpt;
	}
	return ListFootPrints;
}

/**
* @brief Projette le point désiré sur l'arête du polygone la plus proche et ne contenant pas ce point, retourne le point projeté
* @param Point que l'on veut projeter
* @param Envelope Polygone sur lequel on souhaite projeter le point
*/
OGRPoint* ProjectPointOnEnvelope(OGRPoint* Point, OGRPolygon* Envelope, OGRLineString* CurrLine, OGRLineString* PrevLine, bool* PointIsModified)
{
	OGRLinearRing* ExtRing = Envelope->getExteriorRing();

	OGRPoint* projete = new OGRPoint;

	//On va parcrourir toutes les arêtes de Envelope, on va projeter Point dessus et on va stocker un certains nombre d'informations sur les projections qui semblent intéressantes.
	std::vector<double> Distances; //Contiendra la liste des distances
	std::vector<double> Angles; //Contiendra la liste des Angles
	std::vector<OGRPoint*> Projetes; //Contient la liste des projetés.

	for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j) //Il ne faut pas reparcourir le premier point qui est bouclé à la fin
	{
		OGRPoint * PointCurr = new OGRPoint;
		ExtRing->getPoint(j, PointCurr);
		OGRPoint * PointNext = new OGRPoint;
		ExtRing->getPoint(j + 1, PointNext);
		OGRLineString* LineBC = new OGRLineString;
		LineBC->addPoint(PointCurr);
		LineBC->addPoint(PointNext);

		if(LineBC->Intersects(Point)) //Si l'arête contient Point, inutile de calculer son projeté sur cette même arête.
		{
			delete PointCurr;
			delete PointNext;
			delete LineBC;
			continue;
		}
		//On a trouvé un segment sur lequel sera projeté le point (il ne contient pas Point), il faut maintenant calculer cette projection : H est la projection de A sur [BC] de vecteur directeur v

		double Ax = Point->getX();
		double Ay = Point->getY();
		double Bx = PointCurr->getX();
		double By = PointCurr->getY();
		double Cx = PointNext->getX();
		double Cy = PointNext->getY();
		double vx = Cx-Bx;
		double vy = Cy-By;
		double Normev = sqrt(vx*vx + vy*vy);
		double BH = ((Ax - Bx) * vx + (Ay - By) * vy) / Normev; // <=> BA.v (produit scalaire)
		double Hx;
		double Hy;
		if(BH < 0)
		{
			Hx = Bx;
			Hy = By;
		}
		else if(BH > Normev)
		{
			Hx = Cx;
			Hy = Cy;
		}
		else
		{
			Hx = Bx + BH * vx / Normev;
			Hy = By + BH * vy / Normev;
		}
		double AH = sqrt((Hx-Ax)*(Hx-Ax) + (Hy-Ay)*(Hy-Ay));
		double AHx = Hx - Ax;
		double AHy = Hy - Ay;

		OGRPoint * Proj = new OGRPoint; // Projeté de Point
		Proj->setX(Hx);
		Proj->setY(Hy);

		double PrevLineX = PrevLine->getX(0) - PrevLine->getX(1);
		double PrevLineY = PrevLine->getY(0) - PrevLine->getY(1);
		double AnglePrev = acos((PrevLineX * AHx + PrevLineY * AHy)/(AH*PrevLine->get_Length())); //Mesure de l'angle entre AH et PrevLine
		//AnglePrev = AnglePrev * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

		double CurrLineX = CurrLine->getX(1) - CurrLine->getX(0);
		double CurrLineY = CurrLine->getY(1) - CurrLine->getY(0);
		double AngleCurr = acos((CurrLineX * AHx + CurrLineY * AHy)/(AH*CurrLine->get_Length())); //Mesure de l'angle entre AH et CurrLine
		//AngleCurr = AngleCurr * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

		Distances.push_back(AH);
		double tmp = std::min(AnglePrev / (AnglePrev + AngleCurr), AngleCurr / (AnglePrev + AngleCurr));
		Angles.push_back(tmp); //Permet de repérer angulairement la direction du projeté par rapport aux Lines contenant le point. Si cette valeur vaut 1/2, cela signifie que le projeté forme la bissectrice de l'angle 
		//formé par PrevLine et CurrLine => Visuellement c'est bien 

		Projetes.push_back(Proj);

		delete PointCurr;
		delete PointNext;
		delete LineBC;
	}

	for(int i = 0; i < Envelope->getNumInteriorRings(); ++i)
	{
		OGRLinearRing* IntRing = Envelope->getInteriorRing(i);
		for(int j = 0; j < IntRing->getNumPoints() - 1; ++j) //Il ne faut pas reparcourir le premier point qui est bouclé à la fin
		{
			OGRPoint * PointCurr = new OGRPoint;
			IntRing->getPoint(j, PointCurr);
			OGRPoint * PointNext = new OGRPoint;
			IntRing->getPoint(j + 1, PointNext);
			OGRLineString* LineBC = new OGRLineString;
			LineBC->addPoint(PointCurr);
			LineBC->addPoint(PointNext);

			if(LineBC->Intersects(Point)) //Si l'arête contient Point, inutile de calculer son projeté sur cette même arête.
			{
				delete PointCurr;
				delete PointNext;
				delete LineBC;
				continue;
			}
			//On a trouvé un segment sur lequel sera projeté le point (il ne contient pas Point), il faut maintenant calculer cette projection : H est la projection de A sur [BC] de vecteur directeur v

			double Ax = Point->getX();
			double Ay = Point->getY();
			double Bx = PointCurr->getX();
			double By = PointCurr->getY();
			double Cx = PointNext->getX();
			double Cy = PointNext->getY();
			double vx = Cx-Bx;
			double vy = Cy-By;
			double Normev = sqrt(vx*vx + vy*vy);
			double BH = ((Ax - Bx) * vx + (Ay - By) * vy) / Normev; // <=> BA.v (produit scalaire)
			double Hx;
			double Hy;
			if(BH < 0)
			{
				Hx = Bx;
				Hy = By;
			}
			else if(BH > Normev)
			{
				Hx = Cx;
				Hy = Cy;
			}
			else
			{
				Hx = Bx + BH * vx / Normev;
				Hy = By + BH * vy / Normev;
			}
			double AH = sqrt((Hx-Ax)*(Hx-Ax) + (Hy-Ay)*(Hy-Ay));
			double AHx = Hx - Ax;
			double AHy = Hy - Ay;

			OGRPoint * Proj = new OGRPoint; // Projeté de Point
			Proj->setX(Hx);
			Proj->setY(Hy);

			double PrevLineX = PrevLine->getX(0) - PrevLine->getX(1);
			double PrevLineY = PrevLine->getY(0) - PrevLine->getY(1);
			double AnglePrev = acos((PrevLineX * AHx + PrevLineY * AHy)/(AH*PrevLine->get_Length())); //Mesure de l'angle entre AH et PrevLine
			//AnglePrev = AnglePrev * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

			double CurrLineX = CurrLine->getX(1) - CurrLine->getX(0);
			double CurrLineY = CurrLine->getY(1) - CurrLine->getY(0);
			double AngleCurr = acos((CurrLineX * AHx + CurrLineY * AHy)/(AH*CurrLine->get_Length())); //Mesure de l'angle entre AH et CurrLine
			//AngleCurr = AngleCurr * 360.0 / (2.0 * 3.1416); //Radians -> Degrés

			Distances.push_back(AH);
			double tmp = std::min(AnglePrev / (AnglePrev + AngleCurr), AngleCurr / (AnglePrev + AngleCurr));
			Angles.push_back(tmp); //Permet de repérer angulairement la direction du projeté par rapport aux Lines contenant le point. Si cette valeur vaut 1/2, cela signifie que le projeté forme la bissectrice de l'angle 
			//formé par PrevLine et CurrLine => Visuellement c'est bien 

			Projetes.push_back(Proj);
			delete PointCurr;
			delete PointNext;
			delete LineBC;
		}
	}

	double distance = -1;
	double Angle;
	for(int i = 0; i < Distances.size(); ++i)//Parmi toutes les projections possibles, il va falloir choisir celle que l'on retient : pour l'instant c'est celui dont le projeté est le plus proche, si possible ne contenant pas d'arête entre les deux
	{
		if(distance == -1 || distance > Distances.at(i))
		{
			delete projete;
			projete = (OGRPoint*)Projetes.at(i)->clone();
			distance = Distances.at(i);
			Angle = Angles.at(i);
		}
	}

	distance = 2 * distance; //On double cette valeur pour le test sur les angles : si on trouve un meilleur angle, on accepte une distance deux fois supérieure
	if(Angle < 0.25) //Cela signifie que la ligne formée par Point et son projeté est beaucoup plus proche d'une des Line que de l'autre, on veut regarder si on ne peut pas trouver mieux (idéal = bissectrice ?) sans trop perdre en terme de distance
	{
		for(int i = 0; i < Distances.size(); ++i)// Si l'angle est trop faible, on va chercher un éventuel autre projeté avec une distance convenable et un meilleur angle
		{
			if(Distances.at(i) < distance && Angles.at(i) > 0.25) //On juge la distance convenable si elle est inférieure au double de la distance min, et si on en trouve un après on fait récupère une valeur de distance non doublée
			{
				//SaveGeometrytoShape("Point.shp", Point);
				//SaveGeometrytoShape("Polygon.shp", Envelope);
				//SaveGeometrytoShape("Projete1.shp", projete);
				//SaveGeometrytoShape("Projete2.shp", Projetes.at(i));
				//std::cout << Angle << " - " << Angles.at(i) << std::endl;
				//int a;
				//std::cin >>a;
				delete projete;
				projete = (OGRPoint*)Projetes.at(i)->clone();
				distance = Distances.at(i);
			}
		}
	}

	for(OGRPoint* PointTemp : Projetes)
		delete PointTemp;

	//Il faut vérifier que ces deux points ne forment pas une arête déjà existante, car dans ce cas là il ne sert à rien de la recréer. Point appartient à deux arêtes : CurrLine et la PrevLine précédente, il suffit donc de vérifier ces deux là.
	bool LineFound = projete->Intersects(CurrLine) || projete->Intersects(PrevLine);
	//Il faut vérifier que la ligne formée par ces deux points soit bien contenue dans le polygone. On va tester le milieu formé par ce segment.	
	OGRPoint* Milieu = new OGRPoint((Point->getX() + projete->getX())/2, (Point->getY() + projete->getY())/2);
	bool Outside = !Milieu->Intersects(Envelope);
	delete Milieu;

	if(LineFound || Outside) //On veut prolonger pour voir si on peut fixer le projeté plus loin tout en restant dans le PolygonGMLDiffShape, sinon on ne fait rien car cela signifie qu'il n'y a vraisemblablement pas besoin de couper le polygone en deux
	{
		TVec2f Vecteur;
		Vecteur.x = projete->getX() - Point->getX();
		Vecteur.y = projete->getY() - Point->getY();

		OGRLineString* ProlongementProjete = new OGRLineString;
		OGRPoint* Projete2 = new OGRPoint(projete->getX() + Vecteur.x, projete->getY() + Vecteur.y);
		ProlongementProjete->addPoint(projete);//Crée un point basé sur la droite Point-Projeté, mais un peu plus que loin le Projeté. La question est maintenant de savoir si il est en dehors de PolygonGMLDiffShape ou non.
		ProlongementProjete->addPoint(Projete2);
		//Il ne faut pas oublier qu'à ce moment, Projete n'est pas forcément sur PolygonGMLDiffShape à cause des imprécisions. Du coup il faut faire attention avec les tests d'intersection qui peuvent être faussés
		if(!ProlongementProjete->Intersects(Envelope)) //Cela signifie que Projete2 n'est pas dans le polygon (et que l'intersection de Projete avec le polygon ne fonctionne pas, car Projete est très légèrement en dehors) : îl n'y a pas de projete valide
		{
			delete ProlongementProjete;
			delete Projete2;
			delete projete;
			return new OGRPoint;
		}
		OGRGeometry* Inter = ProlongementProjete->Intersection(Envelope);
		if(Inter->getGeometryType() == wkbPoint || Inter->getGeometryType() == wkbPoint25D) //Cela signifie que Projete2 n'est pas dans le polygon (et que l'intersection de Projete avec le polygon a fonctionné)
		{
			delete ProlongementProjete;
			delete Projete2;
			delete projete;
			delete Inter;
			return new OGRPoint;
		}
		OGRLineString* InterLS = (OGRLineString*) Inter; //Arrivé ici, l'intersection est forcément une LineString
		double Length = InterLS->get_Length();
		if(Length < 10 * Precision_Vect) //Cela signifie que Projete2 n'est pas dans le polygon : l'intersection de Projete avec le polygon n'a pas fonctionné correctement car il est très légèrement à l'intérieur
		{                                //et l'intersection donne donc une ligne très courte correspondant à la différence de précision entre Projete et la ligne du Polygon sur laquelle il censé être
			delete ProlongementProjete;
			delete Projete2;
			delete projete;
			delete InterLS;
			return new OGRPoint;
		}
		//On considère qu'il faut prolonger la ligne jusqu'à la prochaine arête afin de couper le polygone en deux

		double Coeff = 1;

		bool Test1, Test2;
		Test1 = Projete2->Intersects(Envelope); //Projete2 est dans le Polygon
		Test2 = (InterLS->getGeometryType() == wkbLineString || InterLS->getGeometryType() == wkbLineString25D); //Projete2 n'a pas traversé d'interioring Ring car l'intersection serait alors un MutliLineString puisqu'elle serait coupée en deux
		while(Test1 && Test2)//Tant que Projete2 est inclus dans le Polygon et que nous n'avons pas traversé d'InteriorRing
		{
			++Coeff;
			delete InterLS;
			delete ProlongementProjete;
			delete Projete2;

			Projete2 = new OGRPoint(projete->getX() + Coeff * Vecteur.x, projete->getY() + Coeff * Vecteur.y);
			ProlongementProjete = new OGRLineString;
			ProlongementProjete->addPoint(projete);
			ProlongementProjete->addPoint(Projete2);
			InterLS = dynamic_cast<OGRLineString*>(ProlongementProjete->Intersection(Envelope));
			if(InterLS == nullptr)
			{
				Test2 = false;
			}
			else
			{
				Test1 = Projete2->Intersects(Envelope);
			}
		}//Si on sort avec !Test1 et Test2 -> On a trouvé le bon ProlongementProjete. Si on sort avec !Test2 -> On a traversé un Interior Ring donc il faut revenir en arrière sur le coeff pour que Projete2 tombe dedans.
		//Si on sort avec !Test1 mais !Test2 -> On a traveré un InteriorRing, puis la portion de Polygon située derrière pour finalement ressortir (dans un autre InterioRing ou en dehors du Polygon). Pas bon non plus car il faut revenir avant l'intRing
		double PrevCoeff = Coeff - 1; //Définit la valeur de Coeff précédente pour définir l'intervalle de recherche
		int compteur = 0;
		//std::cout << "NEW COEFF" << std::endl;
		while(Test1 || !Test2)//Tant que Projete2 est inclus dans le Polygon
		{
			double TempCoeff = Coeff;
			Coeff = (PrevCoeff + Coeff) / 2; //Dichotomie pour trouver le bon Coeff
			//std::cout << "PrefCoeff = " << PrevCoeff << " TempCoeff = " << TempCoeff << " Coeff = " << Coeff << std::endl;
			delete InterLS;
			delete ProlongementProjete;
			delete Projete2;

			Projete2 = new OGRPoint(projete->getX() + Coeff * Vecteur.x, projete->getY() + Coeff * Vecteur.y);
			ProlongementProjete = new OGRLineString;
			ProlongementProjete->addPoint(projete);
			ProlongementProjete->addPoint(Projete2);
			InterLS = dynamic_cast<OGRLineString*>(ProlongementProjete->Intersection(Envelope));
			++ compteur;
			if(compteur > 20)
			{
				//std::cout << "Boucle Infinie." << std::endl;
				/*SaveGeometrytoShape("Polygon.shp", Envelope);
				SaveGeometrytoShape("Point.shp", Point);
				SaveGeometrytoShape("Projete1.shp", projete);
				SaveGeometrytoShape("Prolongement.shp", ProlongementProjete);
				std::cout << "PrefCoeff = " << PrevCoeff << " TempCoeff = " << TempCoeff << " Coeff = " << Coeff << std::endl;
				int a;
				std::cin >> a;*/
				delete InterLS;
				delete ProlongementProjete;
				delete Projete2;
				return new OGRPoint;
			}
			if(InterLS == nullptr)
				Test2 = false;
			else
				Test2 = true;

			if(!Test2)//On doit diminuer le prochain Coeff
			{
				PrevCoeff = std::min(PrevCoeff, TempCoeff);
			}
			else//On doit augmenter le prochain Coeff
			{
				PrevCoeff = std::max(PrevCoeff, TempCoeff);
			}
		}
		delete InterLS;
		delete Projete2;
		///////////////On a suffisament allongé la ligne formée par le point initiale et le projeté pour couper en deux la portion de polygon désirée, il faut maintenant calculer l'intersection entre cette ligne et le polygone pour trouver le projeté à retourner

		OGRPoint* OldProjete = (OGRPoint*)projete->clone();
		delete projete;
		projete = new OGRPoint;
		distance = Precision_Vect; //On ne fixe pas à 0, pour que si Line intersecte ProlongementProjete en Point, il ne soit pas retenu comme projeté valide

		for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j) //On va reparcourir toutes les lignes du polygone, regarder celles qui intersectent InterLS, et prendre leurs points d'intersection. On conservera celui qui est le plus éloigne de Point
		{
			OGRPoint * PointCurr = new OGRPoint;
			ExtRing->getPoint(j, PointCurr);
			OGRPoint * PointNext = new OGRPoint;
			ExtRing->getPoint(j + 1, PointNext);
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(PointCurr);
			Line->addPoint(PointNext);
			delete PointCurr;
			delete PointNext;

			if(!Line->Intersects(ProlongementProjete))
			{
				delete Line;
				continue;
			}

			OGRGeometry* PointInter = Line->Intersection(ProlongementProjete);
			delete Line;
			if(PointInter->getGeometryType() != wkbPoint && PointInter->getGeometryType() != wkbPoint25D)
			{
				delete PointInter;
				continue;
			}
			double ecartement = PointInter->Distance(Point);
			if(ecartement > distance)
			{
				delete projete;
				projete = (OGRPoint*)PointInter;
				distance = ecartement;
			}
			else
				delete PointInter;
		}
		for(int k = 0; k < Envelope->getNumInteriorRings(); ++k)
		{
			OGRLinearRing* IntRing = Envelope->getInteriorRing(k);
			for(int j = 0; j < IntRing->getNumPoints() - 1; ++j) //On va reparcourir toutes les lignes du polygone, regarder celles qui intersectent InterLS, et prendre leurs points d'intersection. On conservera celui qui est le plus éloigne de Point
			{
				OGRPoint * PointCurr = new OGRPoint;
				IntRing->getPoint(j, PointCurr);
				OGRPoint * PointNext = new OGRPoint;
				IntRing->getPoint(j + 1, PointNext);
				OGRLineString* Line = new OGRLineString;
				Line->addPoint(PointCurr);
				Line->addPoint(PointNext);
				delete PointCurr;
				delete PointNext;

				if(!Line->Intersects(ProlongementProjete))
				{
					delete Line;
					continue;
				}

				OGRGeometry* PointInter = Line->Intersection(ProlongementProjete);
				delete Line;
				if(PointInter->getGeometryType() != wkbPoint && PointInter->getGeometryType() != wkbPoint25D)
				{
					delete PointInter;
					continue;
				}
				double ecartement = PointInter->Distance(Point);
				if(ecartement > distance)
				{
					delete projete;
					projete = (OGRPoint*)PointInter;
					distance = ecartement;
				}
				else
					delete PointInter;
			}
		}
		/*SaveGeometrytoShape("ProlongementProjete.shp", ProlongementProjete);
		SaveGeometrytoShape("Polygon.shp", Envelope);
		SaveGeometrytoShape("Point.shp", Point);
		SaveGeometrytoShape("Projete.shp", projete);
		SaveGeometrytoShape("ProjeteOld.shp", OldProjete);
		int a;
		std::cin >> a;*/

		delete ProlongementProjete;
		*PointIsModified = true;
		delete Point;
		Point = (OGRPoint*)OldProjete->clone();
		delete OldProjete;
	}
	return projete;
}

/**
* @brief Parcourt les points des polygones de Points, regarde s'ils semblent appartenir à une arête de Lines et si c'est le cas, on coupe cette arête en deux afin que ce point existe et que l'intersection fonctionne.
* On retourne une GeometryCollection qui contient les mêmes polygones que Lines, mais avec des arêtes parfois partagées en deux. Globalement ça ne change rien, mais on pourra calculer l'intersection entre les points de Points et celles ci.
* @param Points Liste de polygones auxquels on va s'intéresser pour leurs points
* @param Lines Liste de polygones auxquels on va s'intéresser pour leurs arêtes
*/
OGRGeometryCollection* CreatePointsOnLine(OGRGeometryCollection* Points, OGRGeometryCollection* Lines)
{
	OGRGeometryCollection* Res = new OGRMultiPolygon;

	for(int i = 0; i < Lines->getNumGeometries(); ++i)
	{
		OGRGeometry * GeoL = Lines->getGeometryRef(i);
		if(GeoL->getGeometryType() != wkbPolygon && GeoL->getGeometryType() != wkbPolygon25D)
			continue;
		if(GeoL == nullptr)
			continue;

		OGRPolygon * PolyL = (OGRPolygon*) GeoL;
		OGRLinearRing * RingL = PolyL->getExteriorRing();

		OGRPolygon * ResPoly = new OGRPolygon;
		OGRLinearRing * ResRing = new OGRLinearRing;

		for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
		{
			OGRPoint* P1 = new OGRPoint;
			OGRPoint* P2 = new OGRPoint;
			RingL->getPoint(l, P1);
			RingL->getPoint(l + 1, P2);
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(P1);
			Line->addPoint(P2);

			std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

			for(int j = 0; j < Points->getNumGeometries(); ++j)
			{
				OGRGeometry * GeoP = Points->getGeometryRef(j);

				if(GeoP->getGeometryType() != wkbPolygon && GeoP->getGeometryType() != wkbPolygon25D)
					continue;
				if(GeoP == nullptr)
					continue;

				OGRPolygon * PolyP = (OGRPolygon*) GeoP;
				OGRLinearRing * RingP = PolyP->getExteriorRing();

				for(int p = 0; p < RingP->getNumPoints() - 1; ++p)
				{
					OGRPoint* P = new OGRPoint;
					RingP->getPoint(p, P);

					if(P->Intersects(P1) || P->Intersects(P2))
					{
						delete P;
						continue;
					}

					if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
					{
						bool AlreadyInVec = false;
						for(OGRPoint* Ptemp:PointsCut)
						{
							if(P->Intersects(Ptemp))
							{
								AlreadyInVec = true;
								delete P;
								break;
							}
						}
						if(!AlreadyInVec)
						{
							PointsCut.push_back(P);
							continue;
						}
					}
					delete P;
				}
			}

			ResRing->addPoint(P1);
			while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
			{
				OGRPoint* P = PointsCut.at(0);
				double Dist = P->Distance(P1);
				int indice = 0;
				for(int j = 1; j < PointsCut.size(); ++j)
				{
					OGRPoint* PointTemp = PointsCut.at(j);
					double DistTemp = PointTemp->Distance(P1);
					if(DistTemp < Dist)
					{
						P = PointTemp;
						Dist = DistTemp;
						indice = j;
					}
				}
				ResRing->addPoint(P);

				delete P;
				PointsCut.erase(PointsCut.begin() + indice);
			}

			delete P1;
			delete P2;
			delete Line;
		}
		ResRing->closeRings();
		ResPoly->addRing(ResRing);

		delete ResRing;

		for(int r = 0; r < PolyL->getNumInteriorRings(); ++r)
		{
			ResRing = new OGRLinearRing;
			RingL = PolyL->getInteriorRing(r);

			for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
			{
				OGRPoint* P1 = new OGRPoint;
				OGRPoint* P2 = new OGRPoint;
				RingL->getPoint(l, P1);
				RingL->getPoint(l + 1, P2);
				OGRLineString* Line = new OGRLineString;
				Line->addPoint(P1);
				Line->addPoint(P2);

				std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

				for(int j = 0; j < Points->getNumGeometries(); ++j)
				{
					OGRGeometry * GeoP = Points->getGeometryRef(j);
					if(GeoP->getGeometryType() != wkbPolygon && GeoP->getGeometryType() != wkbPolygon25D)
						continue;
					if(GeoP == nullptr)
						continue;
					OGRPolygon * PolyP = (OGRPolygon*) GeoP;
					OGRLinearRing * RingP = PolyP->getExteriorRing();
					for(int p = 0; p < RingP->getNumPoints() - 1; ++p)
					{
						OGRPoint* P = new OGRPoint;
						RingP->getPoint(p, P);

						if(P->Intersects(P1) || P->Intersects(P2))
						{
							delete P;
							continue;
						}

						if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
						{
							bool AlreadyInVec = false;
							for(OGRPoint* Ptemp:PointsCut)
							{
								if(P->Intersects(Ptemp))
								{
									AlreadyInVec = true;
									delete P;
									break;
								}
							}
							if(!AlreadyInVec)
							{
								PointsCut.push_back(P);
								continue;
							}
						}
						delete P;
					}
				}

				ResRing->addPoint(P1);
				while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
				{
					OGRPoint* P = PointsCut.at(0);
					double Dist = P->Distance(P1);
					int indice = 0;
					for(int j = 1; j < PointsCut.size(); ++j)
					{
						OGRPoint* PointTemp = PointsCut.at(j);
						double DistTemp = PointTemp->Distance(P1);
						if(DistTemp < Dist)
						{
							P = PointTemp;
							Dist = DistTemp;
							indice = j;
						}
					}
					ResRing->addPoint(P);

					delete P;
					PointsCut.erase(PointsCut.begin() + indice);
				}

				delete P1;
				delete P2;
				delete Line;
			}
			ResRing->closeRings();
			ResPoly->addRing(ResRing);

			delete ResRing;
		}

		Res->addGeometryDirectly(ResPoly);
	}

	//SaveGeometrytoShape("VecShape1.shp", Lines);
	//SaveGeometrytoShape("VecShape2.shp", Res);

	return Res;
}

/**
* @brief Parcourt les points du polygones de Points, regarde s'ils semblent appartenir à une arête de Lines et si c'est le cas, on coupe cette arête en deux afin que ce point existe et que l'intersection fonctionne.
* On retourne une les même polygone que Lines, mais avec des arêtes parfois partagées en deux. Globalement ça ne change rien, mais on pourra calculer l'intersection entre les points de Points et celles ci.
* @param Points : Polygone auquel on va s'intéresser pour ses points
* @param Lines : Polygone auquel on va s'intéresser pour ses arêtes
*/
OGRGeometry* CreatePointsOnLine(OGRGeometry* Points, OGRGeometry* Lines)
{
	if((Lines->getGeometryType() != wkbPolygon && Lines->getGeometryType() != wkbPolygon25D) || (Points->getGeometryType() != wkbPolygon && Points->getGeometryType() != wkbPolygon25D))
	{
		OGRGeometryCollection* GC_Points = dynamic_cast<OGRGeometryCollection*>(Points);
		OGRGeometryCollection* GC_Lines = dynamic_cast<OGRGeometryCollection*>(Lines);
		if(GC_Points == nullptr)
		{
			GC_Points = new OGRGeometryCollection;
			GC_Points->addGeometry(Points);
		}
		if(GC_Lines == nullptr)
		{
			GC_Lines = new OGRGeometryCollection;
			GC_Lines->addGeometry(Lines);
		}
		return CreatePointsOnLine(GC_Points, GC_Lines);
		//return nullptr;
	}
	OGRPolygon * PolyP = (OGRPolygon*) Points;
	OGRLinearRing * ExtRingP = PolyP->getExteriorRing();

	std::vector<OGRPoint*> ListPoints; //On stocke la liste des points du polygone Points, le reste ne nous intéresse pas.

	for(int j = 0; j < ExtRingP->getNumPoints() - 1; ++j)
	{
		ListPoints.push_back(new OGRPoint(ExtRingP->getX(j), ExtRingP->getY(j), ExtRingP->getZ(j)));
	}
	for(int i = 0; i < PolyP->getNumInteriorRings(); ++i)
	{
		OGRLinearRing* IntRingP = PolyP->getInteriorRing(i);
		for(int j = 0; j < IntRingP->getNumPoints() - 1; ++j)
		{
			ListPoints.push_back(new OGRPoint(IntRingP->getX(j), IntRingP->getY(j), IntRingP->getZ(j)));
		}
	}

	OGRPolygon * PolyL = (OGRPolygon*) Lines;
	OGRLinearRing * RingL = PolyL->getExteriorRing();

	OGRPolygon * ResPoly = new OGRPolygon;
	OGRLinearRing * ResRing = new OGRLinearRing;

	//// !! On va ajouter les points intermédiaires dans les arêtes concernées, mais on va conserver les coordonnées en Z de Lines. !! ////

	for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
	{
		OGRPoint* P1 = new OGRPoint;
		OGRPoint* P2 = new OGRPoint;
		RingL->getPoint(l, P1);
		RingL->getPoint(l + 1, P2);
		OGRLineString* Line = new OGRLineString;
		Line->addPoint(P1);
		Line->addPoint(P2);

		std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

		for(OGRPoint* P:ListPoints)
		{
			if(P->Intersects(P1) || P->Intersects(P2)) //Si le point existe déjà, inutile de le recréer
				continue;

			if(P->Distance(P1) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P1 (on garde z de P1)
			{
				P1->setX(P->getX());
				P1->setY(P->getY());
				continue;
			}
			if(P->Distance(P2) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P2 (on garde z de P2)
			{
				P2->setX(P->getX());
				P2->setY(P->getY());
				continue;
			}

			if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
			{
				bool AlreadyInVec = false;
				for(OGRPoint* Ptemp:PointsCut)
				{
					if(P->Intersects(Ptemp))
					{
						AlreadyInVec = true;
						break;
					}
				}
				if(!AlreadyInVec)
				{
					double zP = P1->getZ() + (P->getX() - P1->getX()) / (P2->getX() - P1->getX()) * (P2->getZ() - P1->getZ());
					P->setZ(zP);
					PointsCut.push_back((OGRPoint*)P->clone());
					continue;
				}
			}
		}

		ResRing->addPoint(P1);
		while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
		{
			OGRPoint* P = PointsCut.at(0);
			double Dist = P->Distance(P1);
			int indice = 0;
			for(int j = 1; j < PointsCut.size(); ++j)
			{
				OGRPoint* PointTemp = PointsCut.at(j);
				double DistTemp = PointTemp->Distance(P1);
				if(DistTemp < Dist)
				{
					P = PointTemp;
					Dist = DistTemp;
					indice = j;
				}
			}
			ResRing->addPoint(P);

			delete P;
			PointsCut.erase(PointsCut.begin() + indice);
		}

		delete P1;
		delete P2;
		delete Line;
	}
	ResRing->closeRings();
	ResPoly->addRing(ResRing);

	delete ResRing;

	//// Même traitement sur les InteriorRing du Polygon Lines

	for(int r = 0; r < PolyL->getNumInteriorRings(); ++r)
	{
		ResRing = new OGRLinearRing;
		RingL = PolyL->getInteriorRing(r);

		for(int l = 0; l < RingL->getNumPoints() - 1; ++l)
		{
			OGRPoint* P1 = new OGRPoint;
			OGRPoint* P2 = new OGRPoint;
			RingL->getPoint(l, P1);
			RingL->getPoint(l + 1, P2);
			OGRLineString* Line = new OGRLineString;
			Line->addPoint(P1);
			Line->addPoint(P2);

			std::vector<OGRPoint*> PointsCut; //Liste des points qu'il va falloir ajouter au nouveau ResRing

			for(OGRPoint* P:ListPoints)
			{
				if(P->Intersects(P1) || P->Intersects(P2))
					continue;

				if(P->Distance(P1) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P1 (on garde z de P1)
				{
					P1->setX(P->getX());
					P1->setY(P->getY());
					continue;
				}
				if(P->Distance(P2) < Precision_Vect) //Il y a un problème de précision, on fixe P sur P2 (on garde z de P2)
				{
					P2->setX(P->getX());
					P2->setY(P->getY());
					continue;
				}

				if(P->Distance(Line) < Precision_Vect) // Le point doit intersecter Line donc on va créer un point afin que ce soit le cas.
				{
					bool AlreadyInVec = false;
					for(OGRPoint* Ptemp:PointsCut)
					{
						if(P->Intersects(Ptemp))
						{
							AlreadyInVec = true;
							break;
						}
					}
					if(!AlreadyInVec)
					{
						double zP = P1->getZ() + (P->getX() - P1->getX()) / (P2->getX() - P1->getX()) * (P2->getZ() - P1->getZ());
						P->setZ(zP);
						PointsCut.push_back((OGRPoint*)P->clone());
						continue;
					}
				}
			}

			ResRing->addPoint(P1);
			while(!PointsCut.empty())// Trier les points du vector par ordre croissant en fonction de leurs distances par rapport à P1
			{
				OGRPoint* P = PointsCut.at(0);
				double Dist = P->Distance(P1);
				int indice = 0;
				for(int j = 1; j < PointsCut.size(); ++j)
				{
					OGRPoint* PointTemp = PointsCut.at(j);
					double DistTemp = PointTemp->Distance(P1);
					if(DistTemp < Dist)
					{
						P = PointTemp;
						Dist = DistTemp;
						indice = j;
					}
				}
				ResRing->addPoint(P);

				delete P;
				PointsCut.erase(PointsCut.begin() + indice);
			}

			delete P1;
			delete P2;
			delete Line;
		}
		ResRing->closeRings();
		ResPoly->addRing(ResRing);

		delete ResRing;
	}

	for(OGRPoint* P:ListPoints)
		delete P;

	return ResPoly;
}

/**
* @brief Parcourt tous polygons/bâtiments du cadastre et modifie leurs enveloppes pour correspondre au mieux à celles du CityGML
* @param VecGML contient les emprises au sol issues du CityGML
* @param VecShape contient les emprises au sol issues du Shapefile et que l'on va modifier puis exporter dans un nouveau vector de polygons
* @param Link contient les liens entre les emprises au sol des deux fichiers qui s'intersectent
*/
std::vector<OGRPolygon*> FusionEnvelopes(std::vector<OGRPolygon*>* VecGML, std::vector<OGRPolygon*>* VecShape, std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>>* Link)
{
	std::vector<OGRPolygon*> NewVecShape;//Vector résultat
	for(OGRPolygon* Poly:*VecShape)
		NewVecShape.push_back((OGRPolygon*)Poly->clone());

	//OGRMultiPolygon* CutPolygons = new OGRMultiPolygon;
	//OGRMultiLineString* CutLines = new OGRMultiLineString;

	for(int i = 0; i < VecGML->size(); ++i) //Parcourt les bâtiments du CityGML
	{
		//if(i%10 == 0)
		std::cout << "Batiment CityGML : " << i << " / " << VecGML->size() << std::endl;

		/////////////////////////// Pour chaque bâtiment CityGML, on soustrait à son emprise au sol celles des bâtiments cadastraux qui lui sont liés. Cette soustraction créé des points intermédiaires sur des arêtes droites
		/////////////////////////// qui correspondent aux endroits où deux bâtiments cadastraux se touchent. ->GMLDiffShape
		if(VecGML->at(i) == nullptr)
			continue;
		if(Link->first.at(i).size() == 0) // Le bâtiment CityGML n'est lié à aucun bâtiment cadastral, on passe donc au suivante
			continue;

		OGRGeometry* GMLDiffShape = VecGML->at(i)->clone();
		for(int j : Link->first.at(i)) //On parcourt tous les polygones du Shape liés au bâtiment CityGML courant
		{
			OGRGeometry* tmp = GMLDiffShape;
			GMLDiffShape = tmp->Difference(VecShape->at(j));
			delete tmp;
		}
		if(GMLDiffShape->IsEmpty()) //Cela signifie que l'emprise au sol du Bâtiment CityGML est complètement incluse dans les polygones du cadastre, il n'y a donc rien à faire à cette étape
			continue;

		/////////////////////////// Transformation de GMLDiffShape en GeometryCollection pour pouvoir parcrourir ses éléments un par un -> GC_GMLDiffShape

		OGRGeometryCollection* GC_GMLDiffShape = new OGRGeometryCollection;
		if(GMLDiffShape->getGeometryType() == wkbPolygon || GMLDiffShape->getGeometryType() == wkbPolygon25D)
			GC_GMLDiffShape->addGeometryDirectly(GMLDiffShape);
		else if(GMLDiffShape->getGeometryType() == wkbGeometryCollection || GMLDiffShape->getGeometryType() == wkbGeometryCollection25D || GMLDiffShape->getGeometryType() == wkbMultiPolygon || GMLDiffShape->getGeometryType() == wkbMultiPolygon25D)
			GC_GMLDiffShape = (OGRGeometryCollection*) GMLDiffShape;

		/////////////////////////// L'objectif est d'assigner chaque polygone de GC_GMLDiffShape à des emprises au sol du cadastre pour que celles ci recouvrent la surface totalement du bâtiment CityGML. Cependant, certains polygones de
		/////////////////////////// GC_GMLDiffShape peuvent sembler pouvoir être assignés à deux (ou plus) polygones du cadastre, il faut donc les partager en plus petite parties qui seront chacune assignées à un bâtiment cadastral
		/////////////////////////// On va parcourir chaque polygone de GC_GMLDiffShape, regarder s'il partage plusieurs bâtiments cadastraux et si c'est le cas, le découper en plusieurs petits polygones. On assigne ensuite chaque polygone
		/////////////////////////// à un bâtiment cadastral et on unit les emprises au sol afin d'en générer une de plus en plus grosse, jusqu'à ce que la somme des emprises au sol du cadastre recouvre entièrement le bâtiment CityGML

		for(int g = 0; g < GC_GMLDiffShape->getNumGeometries(); ++g)
		{
			OGRGeometry* GeometryGMLDiffShape = GC_GMLDiffShape->getGeometryRef(g);
			if(GeometryGMLDiffShape->getGeometryType() != wkbPolygon && GeometryGMLDiffShape->getGeometryType() != wkbPolygon25D) //Si ce n'est pas un polygon, on passe au suivant
				continue;

			OGRPolygon* PolygonGMLDiffShape = (OGRPolygon*) GeometryGMLDiffShape;
			if(PolygonGMLDiffShape->get_Area() < 10*Precision_Vect)//Filtre sur l'aire des polygons pour enlever ceux issus des imprécisions des fichiers vectoriels (lorsqu'un point ne colle pas parfaitement à la ligne d'un polygone voisin)
				continue;

			/////////////////////////// On va parcourir tous les points du Ring exterior du polygon courant de GC_GMLDiffShape, et compter le nombre de bâtiments cadastraux sur lequel chaque point est également situé. Si ce nombre
			/////////////////////////// est d'au moins 2, alors ce point fait partie de la frontière entre deux bâtiments cadastraux, cela signifie qu'il faut se servir de ce point pour découper PolygonGMLDiffShape en deux. Ces deux polygones
			/////////////////////////// seront ensuite assignés aux deux bâtiments cadastraux concernés. Pour l'instant, on se contente de projeter ce point pour trouver le second avec lequel il formera la ligne qui coupera le polygon en deux.

			OGRLinearRing* RingGMLDiffShape = PolygonGMLDiffShape->getExteriorRing();

			std::vector<OGRPoint*> ListAProjetes; //Contiendra tous les points que l'on aura projetés 
			std::vector<OGRPoint*> ListProjetes; //Contiendra tous les points projetés dont on se servira afin de partager des LineString en deux
			std::vector<OGRPoint*> ListAProjetesModifies; //Contiendra les points projetés "modifiés" : qui ne correspondent pas forcément à des points du Polygone et qu'il faudra donc créer sur le MultiLS
			OGRMultiLineString* MultiLS = new OGRMultiLineString; //Contiendra les LineString qui serviront à générer les polygones désirés

			for(int j = 0; j < RingGMLDiffShape->getNumPoints() - 1; ++j) // -1 car il ne faut pas reparcourir le premier point qui est bouclé à la fin
			{
				OGRPoint * Point = new OGRPoint;
				RingGMLDiffShape->getPoint(j, Point);

				OGRPoint * PointNext = new OGRPoint;
				RingGMLDiffShape->getPoint(j + 1, PointNext);
				OGRLineString* LS = new OGRLineString;
				LS->addPoint(Point);
				LS->addPoint(PointNext);
				delete PointNext;

				MultiLS->addGeometryDirectly(LS); //Pour remplir MultiLS qui servira dans l'étape suivante

				int ShapesByPoint = 0; //Compte le nombre de polygons du Shape qui contiennent également le point courant
				for(int k : Link->first.at(i)) //On parcourt tous les polygones du Shape liés au bâtiment CityGML courant
				{
					if(Point->Intersects(VecShape->at(k)))
						++ShapesByPoint;
					if(ShapesByPoint > 1) //Cela signifie que ce point est partagé par plusieurs emprises au sol de cadastre -> point que l'on doit projeter sur l'enveloppe du CityGML pour diviser le polygon g de GMLDiffShape
					{
						OGRLineString* PrevLine; //On calcule l'arête précédente car elle contient également Point
						if(j == 0) //On est obligé d'aller chercher l'arête précédente à la fin du LinearRing
						{
							PrevLine = new OGRLineString;
							OGRPoint* PrevPoint = new OGRPoint;
							RingGMLDiffShape->getPoint(RingGMLDiffShape->getNumPoints() - 2, PrevPoint); //Il faut aller le chercher le -2 car le -1 est le même que Point;
							PrevLine->addPoint(PrevPoint);
							PrevLine->addPoint(Point);
						}
						else
							PrevLine = (OGRLineString*)MultiLS->getGeometryRef(MultiLS->getNumGeometries() - 2); //On vient d'ajouter le LS courant, donc il faut aller chercher le -2

						bool PointIsModified = false;

						OGRPoint* Projete = ProjectPointOnEnvelope(Point, PolygonGMLDiffShape, LS, PrevLine, &PointIsModified);

						if(Projete->getX() == 0.0 && Projete->getY() == 0.0) //La projection n'est pas nécessaire (si c'est sur un bord du polygon)
							break;

						if(PointIsModified)
							ListAProjetesModifies.push_back((OGRPoint*)Point->clone());

						ListAProjetes.push_back(Point);
						ListProjetes.push_back(Projete);

						break;//Inutile de continuer à parcourir les autres polygons du Shape, cela ne changera rien au résultat puisque la projection sera le même qu'il y ait 2 ou + polygones associés à un point donné
					}
				}
			}

			for(int r = 0; r < PolygonGMLDiffShape->getNumInteriorRings(); ++r) //Il faut faire pareil sur les Interior Ring, au cas où le CityGML enveloppe entièrement un bâtiment shape
			{
				OGRLinearRing* IntRingGMLDiffShape = PolygonGMLDiffShape->getInteriorRing(r);

				for(int j = 0; j < IntRingGMLDiffShape->getNumPoints() - 1; ++j) // -1 car il ne faut pas reparcourir le premier point qui est bouclé à la fin
				{
					OGRPoint * Point = new OGRPoint;
					IntRingGMLDiffShape->getPoint(j, Point);

					OGRPoint * PointNext = new OGRPoint;
					IntRingGMLDiffShape->getPoint(j + 1, PointNext);
					OGRLineString* LS = new OGRLineString;
					LS->addPoint(Point);
					LS->addPoint(PointNext);
					delete PointNext;

					MultiLS->addGeometryDirectly(LS); //Pour remplir MultiLS qui servira dans l'étape suivante

					int ShapesByPoint = 0; //Compte le nombre de polygons du Shape qui contiennent également le point courant
					for(int k : Link->first.at(i)) //On parcourt tous les polygones du Shape liés au bâtiment CityGML courant
					{
						if(Point->Intersects(VecShape->at(k)))
							++ShapesByPoint;
						if(ShapesByPoint > 1) //Cela signifie que ce point est partagé par plusieurs emprises au sol de cadastre -> point que l'on doit projeter sur l'enveloppe du CityGML pour diviser le polygon g de GMLDiffShape
						{
							OGRLineString* PrevLine; //On calcule l'arête précédente car elle contient également Point
							if(j == 0) //On est obligé d'aller chercher l'arête précédente à la fin du LinearRing
							{
								PrevLine = new OGRLineString;
								OGRPoint* PrevPoint = new OGRPoint;
								IntRingGMLDiffShape->getPoint(IntRingGMLDiffShape->getNumPoints() - 2, PrevPoint); //Il faut aller le chercher le -2 car le -1 est le même que Point;
								PrevLine->addPoint(PrevPoint);
								PrevLine->addPoint(Point);
							}
							else
								PrevLine = (OGRLineString*)MultiLS->getGeometryRef(MultiLS->getNumGeometries() - 2); //On vient d'ajouter le LS courant, donc il faut aller chercher le -2

							bool PointIsModified = false;

							OGRPoint* Projete = ProjectPointOnEnvelope(Point, PolygonGMLDiffShape, LS, PrevLine, &PointIsModified);

							if(Projete->getX() == 0.0 && Projete->getY() == 0.0) //La projection n'est pas nécessaire (si c'est sur un bord du polygon)
								break;

							if(PointIsModified)
								ListAProjetesModifies.push_back((OGRPoint*)Point->clone());

							ListAProjetes.push_back(Point);
							ListProjetes.push_back(Projete);

							break;//Inutile de continuer à parcourir les autres polygons du Shape, cela ne changera rien au résultat puisque la projection sera le même qu'il y ait 2 ou + polygones associés à un point donné
						}
					}
				}
			}

			//////// !!!!! : Traiter le cas où un Polygon possède un InteriorRing et une seule projection qui le découpe : Risque de ne pas découper le polygon en deux car l'arête coupante s'arrêtera à l'InteriorRing.

			/////////////////////////// On a une liste de couples de points (en deux vector remplis simultanément) qui doivent partager PolygonGMLDiffShape. Pour mettre en place cette découpe, on récupère la liste LS des arêtes du polygone,
			/////////////////////////// on la parcourt en testant chaque ligne pour voir si elle contient (l'intersection ne fonctionne pas, test sur la distance) un des points projetés précédemment calculés. 
			/////////////////////////// Si c'est le cas, on partage cette ligne en deux afin d'intégrer ce point dans LS, et on ajoute également la ligne formée par ce point et celui qui l'a généré au départ, celui dont il est le projeté.

			/*OGRGeometry* tmpMultiLS = MultiLS->clone(); ///////////////////////// DEBUG

			OGRMultiPoint* LAP = new OGRMultiPoint; ////// DEBUG
			OGRMultiPoint* LAPM = new OGRMultiPoint; ////// DEBUG
			OGRMultiPoint* LP = new OGRMultiPoint; ////// DEBUG
			for(OGRPoint* PointTemp : ListAProjetes)
			{
			LAP->addGeometry(PointTemp);
			}
			for(OGRPoint* PointTemp : ListAProjetesModifies)
			{
			LAPM->addGeometry(PointTemp);
			}
			for(OGRPoint* PointTemp : ListProjetes)
			{
			LP->addGeometry(PointTemp);
			}*/

			for(int j = 0; j < MultiLS->getNumGeometries(); ++j)
			{
				OGRLineString* LS = (OGRLineString*)MultiLS->getGeometryRef(j);
				OGRPoint* Point1 = new OGRPoint;
				LS->getPoint(0, Point1);
				OGRPoint* Point2 = new OGRPoint;
				LS->getPoint(1, Point2);

				bool test = false;
				int l = 0;
				for(OGRPoint* Projete:ListProjetes)
				{
					if(Projete->Distance(LS) < Precision_Vect) //Utilisation de Distance avec un seuil très faible car l'intersection ne fonctionne pas entre un point et un segment qui est censé passer par ce point (à part si c'est un des deux points du segment)
					{
						OGRLineString* LS1 = new OGRLineString;
						LS1->addPoint(Point1);
						LS1->addPoint(Projete);
						OGRLineString* LS2 = new OGRLineString;
						LS2->addPoint(Projete);
						LS2->addPoint(Point2);

						OGRLineString* LS3 = new OGRLineString; //Ligne qui "coupe en deux" les polygones
						LS3->addPoint(Projete);
						LS3->addPoint(ListAProjetes.at(l));

						delete ListProjetes.at(l);
						delete ListAProjetes.at(l);
						ListProjetes.erase(ListProjetes.begin() + l);
						ListAProjetes.erase(ListAProjetes.begin() + l);
						MultiLS->removeGeometry(j);
						MultiLS->addGeometryDirectly(LS1);
						MultiLS->addGeometryDirectly(LS2);
						MultiLS->addGeometryDirectly(LS3);
						--j;
						test = true;
						break;
					}
					++l;
				}

				if(test) //Si on a déjà ajouté des points dans MultiLS via la boucle précédente, on sort.
					continue;

				l = 0;

				for(OGRPoint* AProjete:ListAProjetesModifies) //Il faut s'assurer que les points à projeter qui ont changé, donc qui ne sont plus forcément directement sur le polygon, soient bien présents dans MultiLS
				{
					if(AProjete->Distance(LS) < Precision_Vect && AProjete->Distance(Point1) > Precision_Vect && AProjete->Distance(Point2) > Precision_Vect)
					{
						OGRLineString* LS1 = new OGRLineString;
						LS1->addPoint(Point1);
						LS1->addPoint(AProjete);
						OGRLineString* LS2 = new OGRLineString;
						LS2->addPoint(AProjete);
						LS2->addPoint(Point2);

						delete ListAProjetesModifies.at(l);
						ListAProjetesModifies.erase(ListAProjetesModifies.begin() + l);

						MultiLS->removeGeometry(j);
						MultiLS->addGeometryDirectly(LS1);
						MultiLS->addGeometryDirectly(LS2);
						--j;
						break;
					}
					++l;
				}

				delete Point1;
				delete Point2;
			}

			for(OGRPoint* PointTemp : ListAProjetes)
				delete PointTemp;
			for(OGRPoint* PointTemp : ListAProjetesModifies)
				delete PointTemp;
			for(OGRPoint* PointTemp : ListProjetes)
				delete PointTemp;

			/////////////////////////// Grâce aux nouvelles lignes insérées, on utilise la fonction Polygonize qui va les prendre en compte pour générer un nouvel ensemble de polygone. Si la ligne coupe PolygonGMLDiffShape en deux, Polygonize va 
			/////////////////////////// ressortir les deux polygons désirés.

			//for(int  j = 0; j < MultiLS->getNumGeometries(); ++j)
			//	CutLines->addGeometry(MultiLS->getGeometryRef(j)->clone());

			OGRGeometryCollection* SplitPolygon = (OGRGeometryCollection*) MultiLS->Polygonize();

			if(SplitPolygon->IsEmpty())
			{
				std::cout << "ERREUR : SPLIT POLYGON VIDE !!" << std::endl;
				//SaveGeometrytoShape("GeometryGMLDiffShape.shp", GeometryGMLDiffShape);
				//SaveGeometrytoShape("MultiLS.shp", MultiLS);
				/*SaveGeometrytoShape("MultiLS_Old.shp", tmpMultiLS);
				SaveGeometrytoShape("ListAProjetes.shp", LAP);
				SaveGeometrytoShape("ListAProjetesModifies.shp", LAPM);
				SaveGeometrytoShape("ListProjetes.shp", LP);*/
				delete MultiLS;
				//int a;
				//std::cin >> a;
				continue;
			}

			delete MultiLS;

			/// On parcourt les polygones de SplitPolygon nouvellement créés pour mettre à jour les Polygon du Shape auxquels ils sont liés en ajoutant les éventuels points nécessaires
			/// A une intersection/union fonctionnelle. On profite également de ce parcourt pour repérer à quel bâtiment du Shape chaque polygon de SplitPolygon va devoir être ajouté.

			for(int j = 0; j < SplitPolygon->getNumGeometries(); ++j)
			{
				//CutPolygons->addGeometry(Geo->clone());
				OGRGeometry* Geo = SplitPolygon->getGeometryRef(j); //C'est forcément un Polygon

				//Il faut retirer les Interior Ring de PolygonGMLDiffShape qui ont créé des polygons à cause de Polygonize : on calcule l'intersection et si l'intersection n'est pas un Polygon, c'est que Geo est un InteriorRing

				OGRGeometry* InterGEO_GMLDiffShape = Geo->Intersection(PolygonGMLDiffShape);

				if(InterGEO_GMLDiffShape->getGeometryType() != wkbPolygon && InterGEO_GMLDiffShape->getGeometryType() != wkbPolygon25D)
					continue;

				delete InterGEO_GMLDiffShape;

				if(((OGRPolygon*)Geo)->get_Area() < 10 * Precision_Vect)
					continue;

				double distance = 0;
				double area = 0;
				int indice = -1;
				int indice2 = -1;

				for(int k : Link->first.at(i))
				{
					if(Geo->Distance(VecShape->at(k)) > Precision_Vect) //Le polygon de SplitPolygon n'est pas lié au bâtiment shape courant, on passe au suivant
						continue;
					/////////////////////////// Certains points de Geo qui sont censés s'intersecter avec les polygones cadastraux (qui ont servi à leur génération) ne correspondent pas à des points sur ceux ci (par exemple
					/////////////////////////// ils se retrouvent au milieu d'une ligne d'un polygone cadastral). Du coup l'intersection ne fonctionne pas. Il faut donc créer ces points sur les lignes concernées car l'intersection Point-Point fonctionne.

					OGRGeometry* ShapeWithPoints = CreatePointsOnLine(Geo, VecShape->at(k)); //On considère que ces deux polygons s'intersectent, on va donc créer les points de Geo sur VecShape pour que ce soit effectivement le cas pour GDAL

					OGRGeometry* Inter = Geo->Intersection(ShapeWithPoints); //Inter peut être Point, LineString, MultiLineString ou GeometryCollection (Point + Line ?)

					if(Inter == nullptr)
					{
						indice = -1; //Voir slide 7 du Pwp CutCityGMLShape pour cas d'application où on passe ici.
						indice2 = -1;

						//SaveGeometrytoShape("Geo1.shp", Geo);
						//SaveGeometrytoShape("Geo2.shp", VecShape->at(k));
						//SaveGeometrytoShape("Geo3.shp", ShapeWithPoints);
						//int a;
						//std::cin >> a;
						delete ShapeWithPoints;

						break;//Pour pas que Geo soit assigné à un autre k car il y aurait alors du recouvrement entre les deux, on n'associe Geo à aucun polygon du Shape.
					}
					delete ShapeWithPoints;
					OGRGeometryCollection* InterGC = dynamic_cast<OGRGeometryCollection*>(Inter);

					if(InterGC == nullptr) //Si Inter est un  Point ou un LineString, il ne pourra être converti en GC donc on va l'ajouter pour en faire un GC avec un seul élément, ce qui n'est pas gênant et évite trop de if
					{
						InterGC = new OGRGeometryCollection;
						InterGC->addGeometry(Inter);
					}
					double distanceTemp = 0;
					double areaTemp = 0;
					for(int z = 0; z < InterGC->getNumGeometries(); ++z)
					{
						OGRGeometry* CurrGeo = InterGC->getGeometryRef(z);
						if(CurrGeo->getGeometryType() == wkbLineString || CurrGeo->getGeometryType() == wkbLineString25D)
							distanceTemp += ((OGRLineString*)CurrGeo)->get_Length();
						else if(CurrGeo->getGeometryType() == wkbPolygon || CurrGeo->getGeometryType() == wkbPolygon25D)
							areaTemp += ((OGRPolygon*)CurrGeo)->get_Area();
					}
					if(distanceTemp > distance)
					{
						distance = distanceTemp;
						indice = k;
					}
					if(areaTemp > area)
					{
						area = areaTemp;
						indice2 = k;
					}
					if(indice == -1)
						indice = k; //Si aucun indice n'a été relevé et qu'on est arrivé jusqu'ici, cela signifie qu'il y a une intersection (Point) et qu'il faut donc l'ajouter au cas où on en trouve pas d'autre. On laisse distance à 0 
					//Pour qu'il soit facilement changé si une intersection LineString ou Polygon est trouvée
					delete Inter;
				}
				if(indice2 != -1) //On privilégie les intersections avec des aires, dû à des imprécisions mais qui témoignent d'une certaine proximité.
				{
					/*SaveGeometrytoShape("Geo.shp", Geo);
					SaveGeometrytoShape("SplitPolygon.shp", SplitPolygon);
					SaveGeometrytoShape("MultiLS.shp", MultiLS);
					SaveGeometrytoShape("Geo1.shp", NewVecShape.at(indice));
					SaveGeometrytoShape("Geo2.shp", NewVecShape.at(indice2));
					std::cout << "AIRE " << area << "  " << distance << std::endl;
					int a;
					std::cin >> a;*/
					indice = indice2;
				}

				if(indice == -1) //Le polygon découpé n'a pas réussi à se lier à un bati Shape : A DEBUG
				{
					std::cout << "ERREUR Cas INDICE -1" << std::endl;

					/*SaveGeometrytoShape("Geo1.shp", Geo);
					int a;
					std::cin >> a;*/
					continue;
				}

				OGRGeometry* ShapeWithPoints = CreatePointsOnLine(Geo, NewVecShape.at(indice));


				if(!ShapeWithPoints->IsValid())
				{
					continue; //////////////////// Pour corriger bug sur Oullins
				}

				OGRGeometry* ShapeUnion = ShapeWithPoints->Union(Geo);

				delete ShapeWithPoints;
				if(ShapeUnion->getGeometryType() != wkbPolygon && ShapeUnion->getGeometryType() != wkbPolygon25D) //A cause des imprécisions, on peut avoir un MultiPolygon ... Il faut combler les trous pour obtenir un Polygon
				{
					OGRGeometryCollection* GC = (OGRGeometryCollection*)ShapeUnion;
					OGRGeometry* Test = new OGRPolygon;
					for(int m = 0; m < GC->getNumGeometries(); ++m)
					{
						//SaveGeometrytoShape("Geo" + std::to_string(m) + ".shp", GC->getGeometryRef(m));
						//std::cout << GC->getGeometryRef(m)->getGeometryName() << std::endl;
						if(GC->getGeometryRef(m)->getGeometryType() == wkbPolygon || GC->getGeometryRef(m)->getGeometryType() == wkbPolygon25D)
						{
							OGRGeometry* tmp = Test;
							Test = tmp->Union(GC->getGeometryRef(m));
							delete tmp;
						}
					}
					delete NewVecShape.at(indice);
					NewVecShape.at(indice) = (OGRPolygon*) Test;
				}
				else
				{
					delete NewVecShape.at(indice);
					NewVecShape.at(indice) = (OGRPolygon*) ShapeUnion;
				}
			}
			delete SplitPolygon;
		}
		delete GC_GMLDiffShape;
	}

	//SaveGeometrytoShape("CutPolygons.shp", CutPolygons); //TODO : faire une fonction qui soustrait le cadastre au CityGML et ressort les différences (calcul d'air, shapefile des différences, ...)
	//SaveGeometrytoShape("CutLines.shp", CutLines);

	OGRMultiPolygon* NewShape = new OGRMultiPolygon;

	for(OGRPolygon* Poly:NewVecShape)
	{
		NewShape->addGeometry(Poly);
	}

	//SaveGeometrytoShape("NewShape.shp", NewShape);

	delete NewShape;

	return NewVecShape;
}

/**
* @brief Compare une par une les emprises au sol des deux vector afin de savoir quelles sont celles qui s'intersectent. Pour chaque bâtiment d'un vector, on connait la liste de ceux de l'autre vector qui l'intersectent (avec les indices)
* @param Vec1 contient les emprises au sol de la première base de données (CityGML)
* @param Vec2 contient les emprises au sol de la seconde base de données (Shape)
*/
std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> LinkCityGMLandShapefilesFootprints(std::vector<OGRPolygon*> Vec1, std::vector<OGRPolygon*> Vec2)
{
	std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> Link;

	Link.first.resize(Vec1.size());
	Link.second.resize(Vec2.size());

	for(int i = 0; i < Vec1.size(); ++i)
	{
		if(Vec1.at(i) == nullptr)
			continue;
		OGRPolygon* Poly1 = Vec1.at(i);
		for(int j = 0; j < Vec2.size(); ++j)
		{
			if(Vec2.at(j) == nullptr)
				continue;
			OGRPolygon* Poly2 = Vec2.at(j);
			if(!Poly1->Intersects(Poly2))
				continue;

			OGRGeometry* Inter = Poly1->Intersection(Poly2);
			if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D) 
			{
				if(((OGRPolygon*)Inter)->get_Area() > 1) //Pour éliminer les liaisons non pertinentes (lorsque la surface commune est trop petite). Plus de détails sur CorrectionBugs.pptx.
				{
					Link.first[i].push_back(j);
					Link.second[j].push_back(i);
				}
			}
			if(Inter->getGeometryType() == wkbMultiPolygon || Inter->getGeometryType() == wkbMultiPolygon25D)
			{
				if(((OGRMultiPolygon*)Inter)->get_Area() > 1)
				{
					Link.first[i].push_back(j);
					Link.second[j].push_back(i);
				}
			}
			delete Inter;
		}
	}
	return Link;
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

	if(AB.x == 0) //Pour le calcul de s et t, cela pose problème donc on intervertit B et C pour avoir un AB.x != 0. En effet, AB.x et AC.x ne peuvent tous deux être égaux à 0 sinon le triangle serait plat.
	{
		TVec3d VecTemp = AB;

		AB = AC;
		AC = VecTemp;

		VecTemp = B;
		B = C;
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
* @brief Parcourt les polygones des bâtiments d'un fichier CityGML et les assigne, selon leurs positions, à des emprises au sol générées à partir d'un fichier ShapeFile afin de créer un nouveau fichier CityGML.
* @param FootprintsShape Contient les emprises au sol des bâtiments ShapeFile que l'on souhaite extruder en 3D grâce aux polygones CityGML.
* @param LayerShape Contient toutes les données du ShapeFile afin de transférer les éventuelles informations sémantiques dans le CityGML créé.
* @param ModelGML Contient toutes les données du CityGML que l'on va parcourir.
* @param Link Contient les liens entre les bâtiments CityGML et Shape afin de savoir lesquels sont à mettre en relation.
*/
citygml::CityModel* AssignPolygonGMLtoShapeBuildings(std::vector<OGRGeometryCollection*>* FootprintsShape, OGRLayer* LayerShape, citygml::CityModel* ModelGML, std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>>* Link, std::vector<TextureCityGML*>* TexturesList)
{
	citygml::CityModel* ModelOut = new citygml::CityModel;

	OGRFeature *Feature;

	///On effectue un premier passage afin de détecter les murs qu'il faudra générer pour avoir des bâtiments fermés
	std::vector<OGRMultiLineString*>* ListGeneratedWalls = new std::vector<OGRMultiLineString*>[FootprintsShape->size()];

	//OGRGeometryCollection* Test = new OGRGeometryCollection; //Aretes internes
	//OGRGeometryCollection* Test2 = new OGRGeometryCollection; //Aretes internes 3D

	int cpt = -1;

	for(citygml::CityObject* obj : ModelGML->getCityObjectsRoots())
	{
		++ cpt;

		std::vector<int> ListBatiShp = Link->first.at(cpt); //Contient les indices des polygons Shp liés à ce Bâtiment CityGML

		if(ListBatiShp.empty()) //Si le bâtiment CityGML n'a aucun équivalent dans le Shapefile, il faut quand même l'ajouter tel quel.
		{
			if(obj->getType() == citygml::COT_Building && !obj->IsEmpty())
			{
				citygml::CityObject* BuildingCO = new citygml::Building(obj->getId());
				for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
				{
					ModelOut->addCityObject(object);
					BuildingCO->insertNode(object);

					//Récupération des textures sur ces données
					for(citygml::Geometry* Geometry : object->getGeometries()) //pour chaque géométrie
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons()) //Pour chaque polygone
						{
							if(PolygonCityGML->getTexture() == nullptr)
								continue;

							std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

							std::string Url = PolygonCityGML->getTexture()->getUrl();
							citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

							TexturePolygonCityGML Poly;

							Poly.Id = PolygonCityGML->getId();
							Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
							Poly.TexUV = TexUV;

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
					}
				}
				ModelOut->addCityObject(BuildingCO);
				ModelOut->addCityObjectAsRoot(BuildingCO);
			}
			continue; //Le bâtiment a été copié, pas besoin d'aller plus loin.
		}

		std::cout << "Elaboration des murs interieurs du batiment " << cpt << " / " << ModelGML->getCityObjectsRoots().size() << " : " << obj->getId() << std::endl;

		std::vector<OGRPolygon*> ListPolygonRoofCityGML; //Contiendra les polygons du toits du bâtiment CityGML courant

		if(obj->getType() == citygml::COT_Building)
		{
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

							OgrRing->closeRings();

							if(OgrRing->getNumPoints() > 3)
							{
								OGRPolygon * OgrPoly = new OGRPolygon;
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid())
								{
									ListPolygonRoofCityGML.push_back(OgrPoly);
								}
								else
									delete OgrPoly;
							}
							else
								delete OgrRing;
						}
					}
				}
			}
		}

		for(int i = 0; i < ListBatiShp.size() - 1; ++i) //On regroupe les bâtiment Shp par Bâtiment CityGML afin de faciliter la recherche de voisins
		{
			OGRGeometryCollection* Footprint1 = FootprintsShape->at(ListBatiShp.at(i));

			if(Footprint1 == nullptr || Footprint1->IsEmpty())
				continue;

			for(int j = i + 1; j < ListBatiShp.size(); ++j) //On va regarder les prochains footprints pour trouver les voisins et déterminer leurs frontières
			{
				OGRGeometryCollection* Footprint2 = FootprintsShape->at(ListBatiShp.at(j));
				if(Footprint2 == nullptr || Footprint2->IsEmpty())
					continue;

				if(Footprint1->Distance(Footprint2) > Precision_Vect)
					continue;

				//OGRGeometryCollection* Footprint1WithPoints = CreatePointsOnLine(Footprint2, Footprint1);
				//OGRGeometryCollection* Footprint2WithPoints = CreatePointsOnLine(Footprint1, Footprint2);

				OGRMultiLineString* ListAretesIntersection = new OGRMultiLineString;

				/*std::vector<OGRMultiLineString*> LineStringsFootprint1 = GetLineStringsFromMultiPolygon(Footprint1WithPoints);

				for(OGRMultiLineString* MultiLS1 : LineStringsFootprint1) //Chaque MultiLineString correspond à un Ring (exterior ou interior) parmi ceux des polygones de Footprint1WithPoints
				{
				for(int L1 = 0; L1 < MultiLS1->getNumGeometries(); ++L1)
				{
				OGRLineString* Ring1 = MultiLS1->getGeometryRef(L1); // Un LineString issu GetLineStringsFromMultiPolygon correspond à seulement deux points.

				if(Ring1->Distance(Geo2) > Precision_Vect)
				continue;

				OGRPoint* P1 = new OGRPoint;
				OGRPoint* P2 = new OGRPoint;

				Ring1->getPoint(0, P1);
				Ring1->getPoint(1, P2);

				if(P1->Distance(Geo2) < Precision_Vect && P2->Distance(Geo2) < Precision_Vect)
				{
				OGRLineString* LineInter = new OGRLineString;
				LineInter->addPoint(P1);
				LineInter->addPoint(P2);
				Test->addGeometry(LineInter);
				ListAretesIntersection->addGeometryDirectly(LineInter);
				}
				delete P1;
				delete P2;
				}
				}*/

				for(int G1 = 0; G1 < Footprint1->getNumGeometries(); ++G1)
				{
					OGRGeometry* Geo1 = Footprint1->getGeometryRef(G1);

					if(Geo1->getGeometryType() != wkbPolygon && Geo1->getGeometryType() != wkbPolygon25D)
						continue;
					if(((OGRPolygon*)Geo1)->get_Area() < Precision_Vect)
						continue;
					if(Geo1->Distance(Footprint2) > Precision_Vect)
						continue;

					OGRLinearRing* Ring1 = ((OGRPolygon*)Geo1)->getExteriorRing();

					for(int G2 = 0; G2 < Footprint2->getNumGeometries(); ++G2)
					{
						OGRGeometry* Geo2 = Footprint2->getGeometryRef(G2);
						if(Geo2->getGeometryType() != wkbPolygon && Geo2->getGeometryType() != wkbPolygon25D)
							continue;
						if(((OGRPolygon*)Geo2)->get_Area() < Precision_Vect)
							continue;
						if(Geo2->Distance(Geo1) > Precision_Vect)
							continue;

						for(int p = 0; p < Ring1->getNumPoints() - 1; ++p)
						{
							OGRPoint* P1 = new OGRPoint;
							OGRPoint* P2 = new OGRPoint;

							Ring1->getPoint(p, P1);
							Ring1->getPoint(p + 1, P2);

							if(P1->Distance(Geo2) < Precision_Vect && P2->Distance(Geo2) < Precision_Vect)
							{
								OGRLineString* LineInter = new OGRLineString;
								LineInter->addPoint(P1);
								LineInter->addPoint(P2);

								//On vérifie que le milieu entre P1 et P2 appartient bien aux deux footprints (pour éviter les cas où l'arête correspond à un bord d'une cour intérieur sur un des deux bâtiments)
								OGRPoint* Milieu = new OGRPoint((P1->getX() + P2->getX())/2, (P1->getY() + P2->getY())/2);

								if(Milieu->Distance(Geo1) < Precision_Vect && Milieu->Distance(Geo2) < Precision_Vect)
								{
									//Test->addGeometry(LineInter);
									ListAretesIntersection->addGeometryDirectly(LineInter);
								}
								delete Milieu;
							}
							delete P1;
							delete P2;
						}
					}
				}

				//delete Footprint1WithPoints;
				//delete Footprint2WithPoints;

				if(ListAretesIntersection == nullptr || ListAretesIntersection->getNumGeometries() == 0)
					continue;

				OGRMultiLineString* ListAretesWithZ = new OGRMultiLineString;

				for(int k = 0; k < ListAretesIntersection->getNumGeometries(); ++k)
				{
					OGRLineString* Aretes = (OGRLineString*)ListAretesIntersection->getGeometryRef(k); //Chaque arête correspond à seulement deux points

					OGRPoint * PointArete1 = new OGRPoint;
					OGRPoint * PointArete2 = new OGRPoint;

					Aretes->getPoint(0, PointArete1);
					Aretes->getPoint(1, PointArete2);

					double ZPoint1 = 99999; //Contiendra la valeur de Z que l'on va assigner au premier point de l'arête courante. Ce Z correspondra au Roof le plus bas qui aura été relié à l'arête.
					double ZPoint2; //Contiendra la valeur de Z que l'on va assigner au second point.

					for(OGRPolygon* Poly : ListPolygonRoofCityGML)
					{
						if(PointArete1->Distance(Poly) < Precision_Vect && PointArete2->Distance(Poly) < Precision_Vect)
						{
							OGRPoint* PointArete1_3D = ProjectPointOnPolygon3D(PointArete1, Poly);

							if(ZPoint1 > PointArete1_3D->getZ())
							{
								ZPoint1 = PointArete1_3D->getZ();

								OGRPoint* PointArete2_3D = ProjectPointOnPolygon3D(PointArete2, Poly);
								ZPoint2 = PointArete2_3D->getZ();
								delete PointArete2_3D;
							}

							delete PointArete1_3D;
						}
					}

					if(ZPoint1 == 99999)
					{
						/*std::cout << "ERREUR CALCUL DE Z DU POINT ARETE" << std::endl;
						SaveGeometrytoShape("A_Footprint1.shp", Footprint1);
						SaveGeometrytoShape("A_Footprint2.shp", Footprint2);
						SaveGeometrytoShape("A_Aretes.shp", Aretes);

						OGRMultiPolygon* MP = new OGRMultiPolygon;
						int num = 0;

						for(OGRPolygon* Poly : ListPolygonRoofCityGML)
						{
						MP->addGeometry(Poly);
						if(PointArete1->Distance(Poly) < Precision_Vect && PointArete2->Distance(Poly) < Precision_Vect)
						{
						OGRPoint* PointArete1_3D = ProjectPointOnPolygon3D(PointArete1, Poly);
						std::cout << num << " : " << PointArete1_3D->getZ() << std::endl;

						if(ZPoint1 > PointArete1_3D->getZ())
						{
						ZPoint1 = PointArete1_3D->getZ();

						OGRPoint* PointArete2_3D = ProjectPointOnPolygon3D(PointArete2, Poly);
						ZPoint2 = PointArete2_3D->getZ();
						delete PointArete2_3D;
						}

						delete PointArete1_3D;
						}
						++num;
						}

						SaveGeometrytoShape("A_ListPolygonRoofCityGML.shp", MP);
						delete MP;
						int a;
						std::cin >> a;*/
					}
					else
					{
						OGRLineString* AretesWithZ = new OGRLineString;

						AretesWithZ->addPoint(PointArete1->getX(), PointArete1->getY(), ZPoint1);
						AretesWithZ->addPoint(PointArete2->getX(), PointArete2->getY(), ZPoint2);

						ListAretesWithZ->addGeometryDirectly(AretesWithZ);
						//Test2->addGeometry(AretesWithZ);
					}
					delete PointArete1;
					delete PointArete2;
				}

				ListGeneratedWalls[ListBatiShp.at(i)].push_back(ListAretesWithZ);
				ListGeneratedWalls[ListBatiShp.at(j)].push_back(ListAretesWithZ);
				delete ListAretesIntersection;
			}
		}
	}

	//std::cout << "FIN " << std::endl;

	//SaveGeometrytoShape("Test.shp", Test);
	//SaveGeometrytoShape("Test2.shp", Test2);
	//int a;
	//std::cin >> a;
	///

	const std::string NameModel = ModelGML->getId();
	LayerShape->ResetReading();
	cpt = -1;
	while((Feature = LayerShape->GetNextFeature()) != NULL)
	{
		++cpt;
		std::cout << cpt << " batiments generes en CityGML sur " << LayerShape->GetFeatureCount() << std::endl;

		OGRGeometryCollection* Building = FootprintsShape->at(cpt);

		if(Building == nullptr)
			continue;

		std::string Name = NameModel + "_" + std::to_string(cpt);
		citygml::CityObject* BuildingCO = new citygml::Building(Name);
		citygml::CityObject* RoofCO = new citygml::RoofSurface(Name+"_Roof");
		citygml::Geometry* Roof = new citygml::Geometry(Name+"_RoofGeometry", citygml::GT_Roof, 2);
		citygml::CityObject* WallCO = new citygml::WallSurface(Name+"_Wall");
		citygml::Geometry* Wall = new citygml::Geometry(Name+"_WallGeometry", citygml::GT_Wall, 2);

		int cptPolyRoof = 0; //Compteur de polygones représentant un Roof du bâtiment courant (pour avoir des noms différents)
		int cptPolyWall = 0; //Compteur de polygones représentant un Wall du bâtiment courant (pour avoir des noms différents)
		int cptPolyGenericWall = 0; //Compteur de polygones représentant un GenericWall du bâtiment courant (pour avoir des noms différents)

		for(int b = 0; b < Building->getNumGeometries(); ++b)
		{
			OGRPolygon* BuildingShp = dynamic_cast<OGRPolygon*>(Building->getGeometryRef(b));
			if(BuildingShp == nullptr || BuildingShp->get_Area() < Precision_Vect)
				continue;

			//SaveGeometrytoShape("A_BuildingShp.shp", BuildingShp);
			OGRPolygon* BuildingShp2 = (OGRPolygon*) BuildingShp->clone(); //Contiendra le polygon Shape mais avec les points intermédiaires ajoutés à partir des Polygons du Wall pour que les intersections fonctionnent.
			OGRMultiPolygon* PolygonsRoofBuildingShp = new OGRMultiPolygon; //Contiendra la liste des Polygons de toit ajouté.

			int cpt2 = - 1;
			for(citygml::CityObject* obj : ModelGML->getCityObjectsRoots())
			{
				++cpt2;
				bool IsLinked = false;
				for(int i : Link->second.at(cpt))
				{
					if(cpt2 == i)
					{
						IsLinked = true;
						break;
					}
				}
				if(!IsLinked) //On regarde si le bâtiment CityGML courant est en relation avec le bâtiment ShapeFile. Si ce n'est pas le cas, on passe au suivant.
				{
					continue;
				}

				if(obj->getType() == citygml::COT_Building)
				{
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
										if(OgrPoly->IsValid() && OgrPoly->Intersects(BuildingShp))
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

											OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, BuildingShp, &TexUV, &TexUVout);

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
													PolygonsRoofBuildingShp->addGeometry(CutPoly);
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
															PolygonsRoofBuildingShp->addGeometry(CutMultiPoly->getGeometryRef(i));
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
											delete CutPoly;
										}
										delete OgrPoly;
									}
									else
										delete OgrRing;
								}
							}
						}
					}
				}
			}

			cpt2 = - 1;
			double Zmin = -1; //Stock la valeur minimale en Z des murs afin d'avoir une valeur à appliquer pour les murs créés à partir de rien.

			for(citygml::CityObject* obj : ModelGML->getCityObjectsRoots())
			{
				++cpt2;
				bool IsLinked = false;
				for(int i : Link->second.at(cpt))
				{
					if(cpt2 == i)
					{
						IsLinked = true;
						break;
					}
				}
				if(!IsLinked) //On regarde si le bâtiment CityGML courant est en relation avec le bâtiment ShapeFile. Si ce n'est pas le cas, on passe au suivant.
				{
					continue;
				}

				if(obj->getType() == citygml::COT_Building)
				{
					for(citygml::CityObject* object : obj->getChildren())//On parcourt les objets (Wall, Roof, ...) du bâtiment
					{
						if(object->getType() == citygml::COT_WallSurface)
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
										if(Zmin == -1 || Zmin > Point.z)
											Zmin = Point.z;
									}
									WallRing->closeRings();

									bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

									std::string Url;
									citygml::Texture::WrapMode WrapMode;
									std::vector<TVec2f> TexUV;

									if(HasTexture)
									{
										Url = PolygonCityGML->getTexture()->getUrl();
										WrapMode = PolygonCityGML->getTexture()->getWrapMode();
										TexUV = PolygonCityGML->getTexCoords();
									}

									if(WallRing->getNumPoints() > 3)
									{
										OGRPolygon * WallPoly = new OGRPolygon;
										WallPoly->addRingDirectly(WallRing);
										if(WallPoly->Distance(BuildingShp) < Precision_Vect) //Le polygon du Wall semble intersecter le Roof du Shape, on va donc l'ajouter à ce bâtiment.
										{
											//Wall->addPolygon(ConvertOGRPolytoGMLPoly(WallPoly, Name)); // Pour simplement ajouter les Polygons de Mur qui touchent un Polygon Shp -> Il y aura des doublons
											//continue;

											OGRPolygon* tmp = BuildingShp2;
											BuildingShp2 = (OGRPolygon*) CreatePointsOnLine(WallPoly, BuildingShp2); //On crée les points du mur sur l'enveloppe du bâtiment pour la génération des murs non présents dans le CityGML.
											delete tmp;

											for(int i = 0; i < PolygonsRoofBuildingShp->getNumGeometries(); ++i)
											{
												OGRPolygon* PolyRoof = (OGRPolygon*) PolygonsRoofBuildingShp->getGeometryRef(i);

												std::vector<TVec2f> TexUVWall;

												if(PolyRoof->Distance(WallPoly) < Precision_Vect) //On recherche quels sont les polygons du Roof qui intersectent le polygon du Wall afin de créer les points de l'un sur l'autre.
												{
													OGRPolygon* PolyRoofwithWallPoints = (OGRPolygon*)CreatePointsOnLine(WallPoly, PolyRoof);
													OGRPolygon* tmp = (OGRPolygon*)CreatePointsOnLine(PolyRoof, WallPoly);
													delete WallPoly;
													WallPoly = tmp;

													//On vérifie que le polygon du Wall se situe bien en dessous du polygon du Roof, sinon il ne faut pas l'ajouter à ce bâtiment (cas d'un mur frontière entre deux bâtiments)
													OGRLinearRing* RingPolyRoofwithWallPoints = PolyRoofwithWallPoints->getExteriorRing();
													OGRLinearRing* RingWallPoly = WallPoly->getExteriorRing();
													bool TestWall = false;
													for(int pw = 0; pw < RingWallPoly->getNumPoints() - 1; ++pw)
													{
														OGRPoint* PointWall = new OGRPoint;
														RingWallPoly->getPoint(pw, PointWall);
														for(int pr = 0; pr < RingPolyRoofwithWallPoints->getNumPoints() - 1; ++ pr)
														{
															OGRPoint* PointRoof = new OGRPoint;
															RingPolyRoofwithWallPoints->getPoint(pr, PointRoof);
															if(PointRoof->Distance(PointWall) < Precision_Vect)
															{
																if(PointRoof->getZ() - PointWall->getZ() < - Precision_Vect) //Cela signifie que le Wall est situé au dessus du Roof
																{
																	TestWall = true;
																	delete PointRoof;
																	break;
																}
															}
															delete PointRoof;
														}
														delete PointWall;
														if(TestWall)
															break;
													}

													if(TestWall) //Si le mur est situé au dessus du toit, c'est que le mur ne doit pas appartenir à ce bâtiment (à part si un autre polygone du toit se situe bien en dessous, donc on continue et on break pas)
													{
														delete PolyRoofwithWallPoints;
														continue;
													}
													// Fin du filtre des Wall

													OGRLinearRing* ExtRingWall = WallPoly->getExteriorRing();
													OGRPolygon* WallPolyRes = new OGRPolygon; //Contiendra le polygon du Wall que l'on aura limité à la zone 2D formée par le polygon BuildingShp
													OGRLinearRing* WallRingRes = new OGRLinearRing;

													for(int i = 0; i < ExtRingWall->getNumPoints() - 1; ++i)
													{
														OGRPoint* WallPoint1 = new OGRPoint(ExtRingWall->getX(i), ExtRingWall->getY(i), ExtRingWall->getZ(i));
														OGRPoint* WallPoint2 = new OGRPoint(ExtRingWall->getX(i+1), ExtRingWall->getY(i+1), ExtRingWall->getZ(i+1));

														if(WallPoint1->Distance(PolyRoofwithWallPoints) < Precision_Vect && WallPoint2->Distance(PolyRoofwithWallPoints) < Precision_Vect)
														{
															WallRingRes->addPoint(WallPoint1);
															WallRingRes->addPoint(WallPoint2);
														}
														delete WallPoint1;
														delete WallPoint2;
													}

													delete PolyRoofwithWallPoints;

													if(WallRingRes != nullptr && WallRingRes->getNumPoints() > 0)
													{
														WallRingRes->closeRings();
														OGRLinearRing* CleanWallRingRes = new OGRLinearRing; //Des points dans WallRingRes sont potentiellement en double, il faut les retirer avant de créer le Polygon, on va mettre le LinearRing modifié ici.
														double prevX = -1, prevY = -1, prevZ = -1;
														for(int i = 0; i < WallRingRes->getNumPoints(); ++i)
														{
															if(WallRingRes->getX(i) != prevX || WallRingRes->getY(i) != prevY || WallRingRes->getZ(i) != prevZ)
															{
																prevX = WallRingRes->getX(i);
																prevY = WallRingRes->getY(i);
																prevZ = WallRingRes->getZ(i);
																if(HasTexture)
																	TexUVWall.push_back(CalculUV(&PointsWall, &TexUV, TVec3d(prevX, prevY, prevZ)));

																CleanWallRingRes->addPoint(prevX, prevY, prevZ);
															}
														}

														delete WallRingRes;
														if(CleanWallRingRes->getNumPoints() < 4)
														{
															delete CleanWallRingRes;
															continue;
														}
														WallPolyRes->addRingDirectly(CleanWallRingRes);
														citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)WallPolyRes, Name + "_Wall_" + std::to_string(cptPolyWall));

														Wall->addPolygon(GMLPoly);

														//delete WallPolyRes;
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
											}
										}
										else
											delete WallPoly;
									}
									else
										delete WallRing;
								}
							}
						}
					}
				}
			}

			//Création des murs correspondant aux arêtes intérieures des bâtiments afin d'avoir des objets 3D fermés

			for(OGRMultiLineString* MultiLS : ListGeneratedWalls[cpt])
			{
				for(int i = 0; i < MultiLS->getNumGeometries(); ++i)
				{
					OGRLineString* Line = (OGRLineString*)MultiLS->getGeometryRef(i);
					OGRPoint* Point1R = new OGRPoint;
					OGRPoint* Point2R = new OGRPoint;

					Line->getPoint(0, Point1R);
					Line->getPoint(1, Point2R);

					if(Point1R->Distance(BuildingShp) > Precision_Vect && Point2R->Distance(BuildingShp) > Precision_Vect) //Ce n'est pas à ce bâtiment qu'il faut assigner le mur.
					{
						delete Point1R;
						delete Point2R;
						continue;
					}

					OGRLinearRing* WallLine = new OGRLinearRing;
					WallLine->addPoint(Point1R);
					WallLine->addPoint(Point1R->getX(), Point1R->getY(), Zmin);
					WallLine->addPoint(Point2R->getX(), Point2R->getY(), Zmin);
					WallLine->addPoint(Point2R);
					WallLine->addPoint(Point1R);
					OGRPolygon* NewWallPoly = new OGRPolygon;
					NewWallPoly->addRingDirectly(WallLine);
					Wall->addPolygon(ConvertOGRPolytoGMLPoly(NewWallPoly, Name + "_GenericWall_" + std::to_string(cptPolyGenericWall)));
					++ cptPolyGenericWall;
					delete NewWallPoly;
					delete Point1R;
					delete Point2R;
				}
			}

			delete PolygonsRoofBuildingShp;
			delete BuildingShp2;
		}

		//Ajout des données sémantiques du shapefile
		for(int i = 0; i < Feature->GetFieldCount(); ++i)
		{
			BuildingCO->setAttribute(std::string(Feature->GetFieldDefnRef(i)->GetNameRef()), std::string(Feature->GetFieldAsString(i)));
		}

		RoofCO->addGeometry(Roof);
		ModelOut->addCityObject(RoofCO);
		BuildingCO->insertNode(RoofCO);

		WallCO->addGeometry(Wall);
		ModelOut->addCityObject(WallCO);
		BuildingCO->insertNode(WallCO);

		ModelOut->addCityObject(BuildingCO);
		ModelOut->addCityObjectAsRoot(BuildingCO);
	}

	delete [] ListGeneratedWalls;

	return ModelOut;
}

/**
* @brief Teste deux Ring afin de déterminer s'ils s'intersectent et comment. Retourne 0 si pas d'intersection, 1 pour un seul point commun et 2 pour plus.
* @param Ring1 Premier Ring
* @param Ring2 Second Ring
*/
int IntersectRings3D(OGRLinearRing* Ring1, OGRLinearRing* Ring2)
{
	int cptPoint = 0; //Compteur de points communs
	for(int i1 = 0; i1 < Ring1->getNumPoints() - 1; ++i1)
	{
		for(int i2 = 0; i2 < Ring2->getNumPoints() - 1; ++i2)
		{
			if(std::abs(Ring1->getX(i1) - Ring2->getX(i2)) < 100*Precision_Vect && std::abs(Ring1->getY(i1) - Ring2->getY(i2)) < 100*Precision_Vect && std::abs(Ring1->getZ(i1) - Ring2->getZ(i2)) < 1000*Precision_Vect)
				++cptPoint;
			if(cptPoint > 1)
				break;
		}
		if(cptPoint > 1)
			break;
	}

	return cptPoint;
}

/**
* @brief Parcourt les polygones représentant des emprises au sol, leur assigne des polygones du toit du CityGML et propose une nouvelle version de ces emprises au sol corrigeant les incohérences 3D.
* @param Footprints Contient les emprises au sol des bâtiments.
* @param ModelGML Contient toutes les données du CityGML que l'on va parcourir.
* @param Link Contient les liens entre les bâtiments CityGML et Shape afin de savoir lesquels sont à mettre en relation.
*/
std::vector<OGRGeometryCollection*> CleanFootprintsWith3D(std::vector<OGRPolygon*>* Footprints, citygml::CityModel* ModelGML, std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>>* Link)
{
	std::vector<OGRMultiPolygon*> CleanedFootprintsMP;

	std::vector<OGRPolygon*>* ListPolygonsFootprints = new std::vector<OGRPolygon*>[Footprints->size()]; //Contiendra tous les polygones de toit de chaque bâtiment Footprints.

	for(int i = 0; i < Footprints->size(); ++i)
	{
		CleanedFootprintsMP.push_back(new OGRMultiPolygon); //Initialisation de CleanedFootprintsMP

		OGRPolygon* Footprint = Footprints->at(i);

		int cpt2 = - 1;
		for(citygml::CityObject* obj : ModelGML->getCityObjectsRoots())
		{
			++cpt2;
			bool IsLinked = false;
			for(int ind : Link->second.at(i))
			{
				if(cpt2 == ind)
				{
					IsLinked = true;
					break;
				}
			}
			if(!IsLinked) //On regarde si le bâtiment CityGML courant est en relation avec l'emprise au sol. Si ce n'est pas le cas, on passe au suivant.
			{
				continue;
			}
			if(obj->getType() == citygml::COT_Building)
			{
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

								OgrRing->closeRings();

								if(OgrRing->getNumPoints() > 3)
								{
									OGRPolygon * OgrPoly = new OGRPolygon;
									OgrPoly->addRingDirectly(OgrRing);
									if(OgrPoly->IsValid() && OgrPoly->Intersects(Footprint))
									{
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, Footprint);

										if(CutPoly != nullptr)
										{
											if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
											{
												ListPolygonsFootprints[i].push_back((OGRPolygon*)CutPoly->clone());
											}
											else
											{
												OGRMultiPolygon* CutMultiPoly = dynamic_cast<OGRMultiPolygon*>(CutPoly);
												if(CutMultiPoly != nullptr)
												{
													for(int g = 0; g < CutMultiPoly->getNumGeometries(); ++g)
													{
														ListPolygonsFootprints[i].push_back((OGRPolygon*)(CutMultiPoly->getGeometryRef(g)->clone()));
													}
												}
											}
										}
										delete CutPoly;
									}
									delete OgrPoly;
								}
								else
									delete OgrRing;
							}
						}
					}
				}
			}
		}
	}

	/*//
	OGRGeometryCollection* Res2 = new OGRGeometryCollection;

	for(int i = 0; i < Footprints->size(); ++i)
	{
	OGRGeometry* MP = new OGRPolygon;
	for(int j = 0; j < ListPolygonsFootprints[i].size(); ++j)
	{
	OGRGeometry* tmp = MP;
	MP = tmp->Union(ListPolygonsFootprints[i].at(j));
	delete tmp;
	}
	Res2->addGeometryDirectly(MP);
	}

	SaveGeometrytoShape("ListPolygonsFootprints.shp", Res2);
	delete Res2;
	//*/

	OGRMultiPolygon* PolygonsDiscontinus = new OGRMultiPolygon;
	std::vector<int> IndiceOrigineDesPolygonsDiscontinus; //Contiendra l'indice du polygone Footprint à partir duquel chaque polygone discontinu aura été extrait, ainsi que sa place à l'intérieur de celui ci dans ListPolygonsFootprints.

	for(int i = 0; i < Footprints->size(); ++i)
	{
		int numPolygonsFootprints = ListPolygonsFootprints[i].size();
		if(numPolygonsFootprints == 0)
			continue;

		bool* PolyIsAssigned = new bool[numPolygonsFootprints]; //Pour chaque Polygone de ListPolygonsFootprints[i], permet de stocker l'information de s'il a déjà été ajouté à un élément du toit.
		memset(PolyIsAssigned, false, numPolygonsFootprints * sizeof(bool));

		for(int j = 0; j < numPolygonsFootprints; ++j)
		{
			if(PolyIsAssigned[j])//Ici, on cherche à créer un nouveau PolyToit. Tous les indices < j sont déjà stockés dans des PolyToit auquel j n'a pas été ajouté, il faut donc lui en créer un nouveau et chercher ses éventuels voisins.
				continue;

			PolyIsAssigned[j] = true;

			OGRPolygon* PolyBase = ListPolygonsFootprints[i].at(j);

			OGRGeometry* PolyToit = PolyBase->clone(); //L'union des polygones qui se touchent en 3D, sera donc normalement un polygone.

			std::vector<int> ListIndices; //Contient la liste des indices de Polygones reliés à PolyToit et pour lesquels le voisinage n'a pas encore été étudié.

			OGRLinearRing* RingBase = PolyBase->getExteriorRing();

			for(int k = j + 1; k < numPolygonsFootprints; ++k)
			{
				if(PolyIsAssigned[k])
					continue;

				OGRPolygon* PolyTest = ListPolygonsFootprints[i].at(k);

				if(PolyBase->Distance(PolyTest) > Precision_Vect) //Un test 2D suffit pour déterminer si les polygones sont censés se toucher ou non.
					continue;

				OGRLinearRing* RingTest = PolyTest->getExteriorRing();

				//On va comparer point par point les deux LinearRing afin de regarder si il y a au moins des points (x,y) qui se retrouvent dans les deux et avec des Z identiques.
				if(IntersectRings3D(RingBase, RingTest) == 2) //Les deux polygones ont au moins deux points en commun
				{
					PolyIsAssigned[k] = true;
					OGRGeometry* tmp = PolyToit;
					PolyToit = tmp->Union(PolyTest);
					delete tmp;
					ListIndices.push_back(k);
				}
			}

			while(!ListIndices.empty())//On a assigné un certain nombre de polygones voisins de PolyBase, on va maintenant les parcourir afin de repérer leurs voisins (PolyIsAssigned permet de ne pas ajouter plusieurs fois le même polygone) de manière récursive
			{
				PolyBase = ListPolygonsFootprints[i].at(ListIndices.at(0));

				RingBase = PolyBase->getExteriorRing();

				for(int k = 0; k < numPolygonsFootprints; ++k)
				{
					if(PolyIsAssigned[k])
						continue;

					OGRPolygon* PolyTest = ListPolygonsFootprints[i].at(k);

					if(PolyBase->Distance(PolyTest) > Precision_Vect) //Un test 2D suffit pour déterminer si les polygones sont censés se toucher ou non.
						continue;

					OGRLinearRing* RingTest = PolyTest->getExteriorRing();

					//On va comparer point par point les deux LinearRing afin de regarder si il y a au moins deux points (x,y) qui se retrouvent dans les deux et avec des Z identiques.

					if(IntersectRings3D(RingBase, RingTest) == 2) //Les deux polygones ont au moins deux points en commun
					{
						PolyIsAssigned[k] = true;
						OGRGeometry* tmp = PolyToit;
						PolyToit = tmp->Union(PolyTest);
						delete tmp;
						ListIndices.push_back(k);
					}
				}

				ListIndices.erase(ListIndices.begin());
			}

			if(PolyToit->getGeometryType() == wkbPolygon || PolyToit->getGeometryType() == wkbPolygon25D)
			{
				double A = ((OGRPolygon*)PolyToit)->get_Area();
				double P = ((OGRPolygon*)PolyToit)->getExteriorRing()->get_Length();
				double Kg = P / (2*sqrt(M_PI * A)); //Indice de compacité de Gravelius (1914) d'un Polygone : Kg = P / (2*sqrt(pi*A)), P = périmètre, A = aire. Kg = 1 : cercle.

				if(Kg > 2.0 || (Kg > 1.3 && A < 5) || A < 1)
				{
					PolygonsDiscontinus->addGeometry(PolyToit);

					IndiceOrigineDesPolygonsDiscontinus.push_back(i);
				}
				else
				{
					CleanedFootprintsMP.at(i)->addGeometry(PolyToit);
				}
			}
			else
			{
				OGRGeometryCollection* GC_PolyToit = dynamic_cast<OGRGeometryCollection*>(PolyToit);
				if(GC_PolyToit != nullptr)
				{
					for(int k = 0; k < GC_PolyToit->getNumGeometries(); ++k)
					{
						if(GC_PolyToit->getGeometryRef(k)->getGeometryType() == wkbPolygon || GC_PolyToit->getGeometryRef(k)->getGeometryType() == wkbPolygon25D)
						{
							double A = ((OGRPolygon*)(GC_PolyToit->getGeometryRef(k)))->get_Area();
							double P = ((OGRPolygon*)(GC_PolyToit->getGeometryRef(k)))->getExteriorRing()->get_Length();
							double Kg = P / (2*sqrt(M_PI * A)); //Indice de compacité de Gravelius (1914) d'un Polygone : Kg = P / (2*sqrt(pi*A)), P = périmètre, A = aire. Kg = 1 : cercle.

							if(Kg > 2.0 || (Kg > 1.3 && A < 5) || A < 1)
							{
								PolygonsDiscontinus->addGeometry(GC_PolyToit->getGeometryRef(k));

								IndiceOrigineDesPolygonsDiscontinus.push_back(i);
							}
							else
							{
								CleanedFootprintsMP.at(i)->addGeometry(GC_PolyToit->getGeometryRef(k));
							}
						}
					}
				}
			}
			delete PolyToit;
		}

		delete [] PolyIsAssigned;
	}

	//SaveGeometrytoShape("PolygonsDiscontinus.shp", PolygonsDiscontinus);

	/// Uniquement pour sauvegarder PolygonsNonDiscontinus.shp

	OGRMultiPolygon* ResTemp = new OGRMultiPolygon;

	for(int i = 0; i < CleanedFootprintsMP.size(); ++i)
	{
		OGRMultiPolygon* MP = CleanedFootprintsMP.at(i);

		if(MP->getNumGeometries() == 0)
		{
			continue;
		}

		OGRGeometryCollection* GC_Union = new OGRGeometryCollection;

		OGRGeometry* UnionPoly = new OGRPolygon;
		for(int j = 0; j < MP->getNumGeometries(); ++j)
		{
			if(((OGRPolygon*)MP->getGeometryRef(j))->get_Area() < 10*Precision_Vect)
				continue;

			OGRGeometry* tmp = UnionPoly;
			UnionPoly = tmp->Union(MP->getGeometryRef(j));
			delete tmp;
		}
		if(UnionPoly->getGeometryType() == wkbPolygon || UnionPoly->getGeometryType() == wkbPolygon25D)
		{
			ResTemp->addGeometryDirectly(UnionPoly);
		}
		else
		{
			GC_Union = dynamic_cast<OGRGeometryCollection*>(UnionPoly);
			if(GC_Union == nullptr)
				continue;
			for(int j = 0; j < GC_Union->getNumGeometries(); ++j)
			{
				OGRPolygon* Poly = dynamic_cast<OGRPolygon*>(GC_Union->getGeometryRef(j));
				if(Poly == nullptr)
					continue;
				ResTemp->addGeometryDirectly(Poly);
			}
		}
	}

	//SaveGeometrytoShape("PolygonsNonDiscontinus.shp", ResTemp);
	delete ResTemp;

	/// Fin

	//On va maintenant parcourir chaque polygone discontinu, regarder s'il semble voisin avec un autre Footprint que celui duquel il est issu et se rattacher à lui. S'il n'en trouve pas, on le remet simplement dans son Footprint d'origine.

	//Un polygon discontinu peut se lier à plusieurs polygons footprints si la séparation entre eux est purement 2D. Il faut donc ajouter une étape de découpe afin de partager si nécessaire le polygon discontinu entre eux.
	std::vector<std::vector<int>> ListFootprintsLinkedtoPolyDiscontinu; //Chaque élément du vector contiendra une liste des indices (dans CleanedFootprintsMP) des polygons se partageant un polygon discontinu.
	std::vector<int> IndicePolygonsDiscontinusToCut; //Contiendra la liste des indices (dans PolygonsDiscontinus) des polygons discontinus concernés par ceci, afin de pouvoir les récupérer rapidement.
	std::vector<int> ListeDesPolygonsDiscontinusNonAssignes; //Contiendra les indices des PolygonsDiscontinus non assignés dès la première passe.

	std::vector<OGRMultiPolygon*> CleanedFootprintsMPSaved;
	for(OGRMultiPolygon* MP : CleanedFootprintsMP) //On crée une sauvegarde de CleanedFootprintsMP car on va maintenant lui ajouter des PolygonsDiscontinus mais on veut conserver cette version pour faire les tests de IntersectRings3D
		CleanedFootprintsMPSaved.push_back((OGRMultiPolygon*)MP->clone());

	for(int k = 0; k < PolygonsDiscontinus->getNumGeometries(); ++k)
	{
		OGRPolygon* PolyDiscontinu = (OGRPolygon*) PolygonsDiscontinus->getGeometryRef(k);
		OGRLinearRing* RingDiscontinu = PolyDiscontinu->getExteriorRing();

		std::vector<int> IndicesPolygonsCommuns; //Contiendra les indices de tous les polygons qui ont en commun une continuité avec PolyDiscontinu.

		for(int i = 0; i < CleanedFootprintsMPSaved.size(); ++i)
		{
			if(i == IndiceOrigineDesPolygonsDiscontinus.at(k)) //Il ne faut pas comparer PolyDiscontinu avec le Footprint depuis lequel il est issu
				continue;

			for(int j = 0; j < CleanedFootprintsMPSaved.at(i)->getNumGeometries(); ++j)
			{
				OGRPolygon* PolyFootprint = dynamic_cast<OGRPolygon*>(CleanedFootprintsMPSaved.at(i)->getGeometryRef(j)->clone());
				if(PolyFootprint == nullptr)
				{
					std::cout << "ERREUR : PolyFootprint n'est pas un Polygon : " << CleanedFootprintsMPSaved.at(i)->getGeometryRef(j)->getGeometryName() << std::endl;
					int a;
					std::cin >> a;
					continue;
				}
				if(PolyDiscontinu->Distance(PolyFootprint) > Precision_Vect)
				{
					delete PolyFootprint;
					continue;
				}

				OGRLinearRing* RingFootprint = PolyFootprint->getExteriorRing();
				if(IntersectRings3D(RingDiscontinu, RingFootprint) == 2) //Cela signifie que PolyDiscontinu et PolyFootprint sont continus en 3D : ils partagent une arrête commune.
				{
					IndicesPolygonsCommuns.push_back(i); //On stocke ce polygon au cas où PolyDiscontinu veut se lier à un autre footprint afin que l'on ait la liste de tous les footprint concernés.

					delete PolyFootprint;
					break;
				}
				else //Le test a peut être négatif car il manquait des points sur Footprint, on va donc re tester une fois les points de PolyDiscontinu projetés dessus.
				{
					delete PolyFootprint;
					PolyFootprint = dynamic_cast<OGRPolygon*>(CreatePointsOnLine(PolyDiscontinu, CleanedFootprintsMPSaved.at(i)->getGeometryRef(j)));
					RingFootprint = PolyFootprint->getExteriorRing();
					if(IntersectRings3D(RingDiscontinu, RingFootprint) == 2) //Cela signifie que PolyDiscontinu et PolyFootprint sont continus en 3D : ils partagent une arrête commune.
					{
						IndicesPolygonsCommuns.push_back(i); //On stocke ce polygon au cas où PolyDiscontinu veut se lier à un autre footprint afin que l'on ait la liste de tous les footprint concernés.

						delete PolyFootprint;
						break;
					}
					else if(PolyDiscontinu->getNumInteriorRings() > 0) //On va maintenant tester les éventuels Interior Rings de PolyDiscontinu
					{
						bool test = false;
						for(int r = 0; r < PolyDiscontinu->getNumInteriorRings(); ++r)
						{
							OGRLinearRing* IntRingDiscontinu = PolyDiscontinu->getInteriorRing(r);
							if(IntersectRings3D(IntRingDiscontinu, RingFootprint) == 2) //Cela signifie que PolyDiscontinu et PolyFootprint sont continus en 3D : ils partagent une arrête commune.
							{
								IndicesPolygonsCommuns.push_back(i); //On stocke ce polygon au cas où PolyDiscontinu veut se lier à un autre footprint afin que l'on ait la liste de tous les footprint concernés.

								delete PolyFootprint;
								test = true;
							}
						}
						if(test)
							break;
					}
				}
				delete PolyFootprint;
			}
		}

		if(IndicesPolygonsCommuns.size() == 0)
			ListeDesPolygonsDiscontinusNonAssignes.push_back(k); //On met de côté PolyDiscontinu pour l'instant
		else if(IndicesPolygonsCommuns.size() == 1) //Il n'y a qu'un seul polygon assigné, donc IndicesPolygonsCommuns ne contient qu'un seul indice, celui qui nous intéresse.
			CleanedFootprintsMP.at(IndicesPolygonsCommuns.at(0))->addGeometry(PolyDiscontinu);
		else if(IndicesPolygonsCommuns.size() > 1 ) //Il y a plusieurs polygons footprint assignés.
		{
			//CleanedFootprintsMP.at(IndicesPolygonsCommuns.at(0))->addGeometry(PolyDiscontinu); ///DECOMMENTER CECI (et commenter le reste) POUR VOIR CE QUE CA DONNE SANS DECOUPE

			ListFootprintsLinkedtoPolyDiscontinu.push_back(IndicesPolygonsCommuns); //Pour le polygon discontinu K, on aura ici la liste des footprints avec lesquels il semble continu en 3D (partage d'une arrête).
			IndicePolygonsDiscontinusToCut.push_back(k);
		}
	}

	//OGRGeometryCollection* PolygonsToCut = new OGRGeometryCollection;//// Debug
	//OGRGeometryCollection* PolygonsCut = new OGRGeometryCollection;////

	//On va parcourir tous les cas de Polygons discontinus liés à plusieurs footprints afin de mettre en place une éventuelle découpe de ce polygon afin de le partager entre les footprints.
	for(int i = 0; i < IndicePolygonsDiscontinusToCut.size(); ++i)
	{
		OGRPolygon* PolyDiscontinu = (OGRPolygon*) PolygonsDiscontinus->getGeometryRef(IndicePolygonsDiscontinusToCut.at(i));

		//PolygonsToCut->addGeometry(PolyDiscontinu);////

		//OGRLinearRing* RingDiscontinu = PolyDiscontinu->getExteriorRing();
		std::vector<OGRMultiPoint*> ListPoints = GetPointsFromPolygon(PolyDiscontinu); //Contient un OGRMultiPoint par Ring du Polygon

		std::vector<OGRPoint*> ListAProjetes; //Contiendra tous les points que l'on aura projetés
		std::vector<OGRPoint*> ListProjetes; //Contiendra tous les points projetés dont on se servira afin de partager des LineString en deux
		std::vector<OGRPoint*> ListAProjetesModifies; //Contiendra les points projetés "modifiés" : qui ne correspondent pas forcément à des points du Polygone et qu'il faudra donc créer sur le MultiLS
		OGRMultiLineString* MultiLS = new OGRMultiLineString; //Contiendra les LineString qui serviront à générer les polygones désirés

		//On va maintenant parcourir point par point ce PolyDiscontinu, afin de repérer quels sont ceux qui se trouvent sur une frontière entre deux PolygonsCommuns,
		//ce seront ces points que l'on projettera pour chercher à découper PolyDiscontinu.
		for(int l = 0; l < ListPoints.size(); ++l)
		{
			for(int j = 0; j < ListPoints.at(l)->getNumGeometries() - 1; ++j)
			{
				OGRPoint * Point = (OGRPoint*)(ListPoints.at(l)->getGeometryRef(j)->clone());
				OGRPoint * PointNext = (OGRPoint*)(ListPoints.at(l)->getGeometryRef(j + 1));

				OGRLineString* LS = new OGRLineString;
				LS->addPoint(Point);
				LS->addPoint(PointNext);

				MultiLS->addGeometryDirectly(LS);

				int PolyByPoint = 0; //Compte le nombre de PolygonsCommuns (ce sont des Footprint) qui contiennent également le point courant
				for(int k : ListFootprintsLinkedtoPolyDiscontinu.at(i)) //On parcourt tous les Footprint liés à PolyDiscontinu
				{
					OGRMultiPolygon* PolyFootprint = CleanedFootprintsMPSaved.at(k);

					if(Point->Distance(PolyFootprint) < Precision_Vect)
						++PolyByPoint;
					if(PolyByPoint > 1) //Cela signifie que ce point est partagé par plusieurs PolyFootprint : on doit le projeter pour tenter de découper PolyDiscontinu.
					{
						OGRLineString* PrevLine; //On calcule l'arête précédente car elle contient également Point
						if(j == 0) //On est obligé d'aller chercher l'arête précédente à la fin du LinearRing
						{
							PrevLine = new OGRLineString;
							OGRPoint * PrevPoint = (OGRPoint *)(ListPoints.at(l)->getGeometryRef(ListPoints.at(l)->getNumGeometries() - 2));//Il faut aller le chercher le -2 car le -1 est le même que Point;
							PrevLine->addPoint(PrevPoint);
							PrevLine->addPoint(Point);
						}
						else
							PrevLine = (OGRLineString*)MultiLS->getGeometryRef(MultiLS->getNumGeometries() - 2)->clone(); //On vient d'ajouter le LS courant, donc il faut aller chercher le -2

						bool PointIsModified = false;

						OGRPoint* Projete = ProjectPointOnEnvelope(Point, PolyDiscontinu, LS, PrevLine, &PointIsModified); ///ATTENTION : Le point projeté est en 2D

						delete PrevLine;

						if(Projete == nullptr)
							break;

						if(Projete->getX() == 0.0 && Projete->getY() == 0.0) //La projection n'est pas nécessaire (si c'est sur un bord du polygon)'
						{
							delete Projete;
							break;
						}

						if(PointIsModified)
							ListAProjetesModifies.push_back((OGRPoint*)Point->clone());
						ListAProjetes.push_back((OGRPoint*)Point->clone());

						ListProjetes.push_back(Projete);

						break;//Inutile de continuer à parcourir les autres polygons du Shape, cela ne changera rien au résultat puisque la projection sera le même qu'il y ait 2 ou + polygones associés à un point donné
					}
				}
				delete Point;
			}
		}

		for(OGRMultiPoint* MP: ListPoints)
			delete MP;

		//Avec MultiLS, on va mettre en place les polygones découpés
		for(int j = 0; j < MultiLS->getNumGeometries(); ++j)
		{
			OGRLineString* LS = (OGRLineString*)MultiLS->getGeometryRef(j);
			OGRPoint* Point1 = new OGRPoint;
			LS->getPoint(0, Point1);
			OGRPoint* Point2 = new OGRPoint;
			LS->getPoint(1, Point2);

			bool test = false;
			int l = 0;
			for(OGRPoint* Projete:ListProjetes)
			{
				if(Projete->Distance(LS) < Precision_Vect) //Utilisation de Distance avec un seuil très faible car l'intersection ne fonctionne pas entre un point et un segment qui est censé passer par ce point (à part si c'est un des deux points du segment)
				{
					OGRLineString* LS1 = new OGRLineString;
					LS1->addPoint(Point1);
					LS1->addPoint(Projete);
					OGRLineString* LS2 = new OGRLineString;
					LS2->addPoint(Projete);
					LS2->addPoint(Point2);

					OGRLineString* LS3 = new OGRLineString; //Ligne qui "coupe en deux" les polygones
					LS3->addPoint(Projete);
					LS3->addPoint(ListAProjetes.at(l));

					delete ListProjetes.at(l);
					delete ListAProjetes.at(l);
					ListProjetes.erase(ListProjetes.begin() + l);
					ListAProjetes.erase(ListAProjetes.begin() + l);
					MultiLS->removeGeometry(j);
					MultiLS->addGeometryDirectly(LS1);
					MultiLS->addGeometryDirectly(LS2);
					MultiLS->addGeometryDirectly(LS3);
					--j;
					test = true;
					break;
				}
				++l;
			}

			if(test) //Si on a déjà ajouté des points dans MultiLS via la boucle précédente, on sort.
				continue;

			l = 0;

			for(OGRPoint* AProjete:ListAProjetesModifies) //Il faut s'assurer que les points à projeter qui ont changé, donc qui ne sont plus forcément directement sur le polygon, soient bien présents dans MultiLS
			{
				if(AProjete->Distance(LS) < Precision_Vect && AProjete->Distance(Point1) > Precision_Vect && AProjete->Distance(Point2) > Precision_Vect)
				{
					OGRLineString* LS1 = new OGRLineString;
					LS1->addPoint(Point1);
					LS1->addPoint(AProjete);
					OGRLineString* LS2 = new OGRLineString;
					LS2->addPoint(AProjete);
					LS2->addPoint(Point2);

					delete ListAProjetesModifies.at(l);
					ListAProjetesModifies.erase(ListAProjetesModifies.begin() + l);

					MultiLS->removeGeometry(j);
					MultiLS->addGeometryDirectly(LS1);
					MultiLS->addGeometryDirectly(LS2);
					--j;
					break;
				}
				++l;
			}
		}

		for(OGRPoint* PointTemp : ListAProjetes)
			delete PointTemp;
		for(OGRPoint* PointTemp : ListAProjetesModifies)
			delete PointTemp;
		for(OGRPoint* PointTemp : ListProjetes)
			delete PointTemp;

		OGRGeometryCollection* ListSplitPolygons = (OGRGeometryCollection*) MultiLS->Polygonize();

		delete MultiLS;

		//Maintenant que l'on a les SplitPolygons, il faut les associer aux footprints correspondants.

		if(ListSplitPolygons != nullptr)
		{
			for(int j = 0; j < ListSplitPolygons->getNumGeometries(); ++j)
			{
				OGRPolygon* SplitPolygon = dynamic_cast<OGRPolygon*>(ListSplitPolygons->getGeometryRef(j));
				if(SplitPolygon == nullptr)
					continue;

				//PolygonsCut->addGeometry(SplitPolygon);////

				OGRLinearRing* SplitRing = SplitPolygon->getExteriorRing();

				double LengthMax = 0;
				int indice = -1;
				for(int k : ListFootprintsLinkedtoPolyDiscontinu.at(i)) //On parcourt tous les Footprint liés à PolyDiscontinu
				{
					//On va rechercher le Footprint qui touche plus les arêtes de PolyDiscontinu
					double Length = 0;

					OGRMultiPolygon* PolyFootprint = CleanedFootprintsMP.at(k);
					for(int z = 0; z < SplitRing->getNumPoints() - 1; ++z)
					{
						OGRPoint* Point = new OGRPoint;
						OGRPoint* NextPoint = new OGRPoint;
						SplitRing->getPoint(z, Point);
						SplitRing->getPoint(z + 1, NextPoint);

						if(Point->Distance(PolyFootprint) < Precision_Vect && NextPoint->Distance(PolyFootprint) < Precision_Vect)
						{
							OGRLineString* Line = new OGRLineString;
							Line->addPoint(Point);
							Line->addPoint(NextPoint);
							Length += Line->get_Length();
							delete Line;
						}
						delete Point;
						delete NextPoint;
					}
					if(Length > LengthMax)
					{
						indice = k;
						LengthMax = Length;
					}
				}
				if(indice != -1)
					CleanedFootprintsMP.at(indice)->addGeometry(SplitPolygon);
			}
		}
		delete ListSplitPolygons;
	}

	//SaveGeometrytoShape("A_PolygonsToCut.shp", PolygonsToCut); ////
	//SaveGeometrytoShape("A_PolygonsCut.shp", PolygonsCut); ////

	for(OGRMultiPolygon* MP : CleanedFootprintsMPSaved) //On n'a plus besoin de cette version
		delete MP;
	std::vector<OGRMultiPolygon*> CleanedFootprintsMPSaved2;
	for(OGRMultiPolygon* MP : CleanedFootprintsMP) //On crée une nouvelle sauvegarde de CleanedFootprintsMP car on va maintenant lui ajouter les PolygonsDiscontinus non assignés mais on veut conserver cette version pour faire les tests de IntersectRings3D
		CleanedFootprintsMPSaved2.push_back((OGRMultiPolygon*)MP->clone());

	for(int k = 0; k < ListeDesPolygonsDiscontinusNonAssignes.size(); ++k) //On faire un second passage en parcourant les polygons discontinus mis de côté afin de les comparer aux footprints déjà modifiés par des polygons discontinus
		//Car un polygon discontinu peut se rattacher à un footprint par un autre polygon discontinu, et ceci n'était pas possible lors de la première phase car on comparait avec les footprints non modifiés
	{
		OGRPolygon* PolyDiscontinu = (OGRPolygon*) PolygonsDiscontinus->getGeometryRef(ListeDesPolygonsDiscontinusNonAssignes.at(k));
		OGRLinearRing* RingDiscontinu = PolyDiscontinu->getExteriorRing();

		std::vector<int> IndicesPolygonsCommuns; //Contiendra les indices de tous les polygons qui ont en commun une continuité avec PolyDiscontinu.

		for(int i = 0; i < CleanedFootprintsMPSaved2.size(); ++i)
		{
			if(i == IndiceOrigineDesPolygonsDiscontinus.at(ListeDesPolygonsDiscontinusNonAssignes.at(k))) //Il ne faut pas comparer PolyDiscontinu avec le Footprint depuis lequel il est issu
				continue;

			for(int j = 0; j < CleanedFootprintsMPSaved2.at(i)->getNumGeometries(); ++j)
			{
				OGRPolygon* PolyFootprint = dynamic_cast<OGRPolygon*>(CleanedFootprintsMPSaved2.at(i)->getGeometryRef(j)->clone());
				if(PolyFootprint == nullptr)
				{
					std::cout << "ERREUR : PolyFootprint n'est pas un Polygon : " << CleanedFootprintsMPSaved2.at(i)->getGeometryRef(j)->getGeometryName() << std::endl;
					int a;
					std::cin >> a;
					continue;
				}
				if(PolyDiscontinu->Distance(PolyFootprint) > Precision_Vect)
				{
					delete PolyFootprint;
					continue;
				}

				OGRLinearRing* RingFootprint = PolyFootprint->getExteriorRing();
				if(IntersectRings3D(RingDiscontinu, RingFootprint) == 2) //Cela signifie que PolyDiscontinu et PolyFootprint sont continus en 3D : ils partagent une arrête commune.
				{
					IndicesPolygonsCommuns.push_back(i); //On stocke ce polygon au cas où PolyDiscontinu veut se lier à un autre footprint afin que l'on ait la liste de tous les footprint concernés.

					delete PolyFootprint;
					break;
				}
				else //Le test a peut être négatif car il manquait des points sur Footprint, on va donc re tester une fois les points de PolyDiscontinu projetés dessus.
				{
					delete PolyFootprint;
					PolyFootprint = dynamic_cast<OGRPolygon*>(CreatePointsOnLine(PolyDiscontinu,CleanedFootprintsMPSaved2.at(i)->getGeometryRef(j)));
					RingFootprint = PolyFootprint->getExteriorRing();
					if(IntersectRings3D(RingDiscontinu, RingFootprint) == 2) //Cela signifie que PolyDiscontinu et PolyFootprint sont continus en 3D : ils partagent une arrête commune.
					{
						IndicesPolygonsCommuns.push_back(i); //On stocke ce polygon au cas où PolyDiscontinu veut se lier à un autre footprint afin que l'on ait la liste de tous les footprint concernés.

						delete PolyFootprint;
						break;
					}
					else if(PolyDiscontinu->getNumInteriorRings() > 0) //On va maintenant tester les éventuels Interior Rings de PolyDiscontinu
					{
						bool test = false;
						for(int r = 0; r < PolyDiscontinu->getNumInteriorRings(); ++r)
						{
							OGRLinearRing* IntRingDiscontinu = PolyDiscontinu->getInteriorRing(r);
							if(IntersectRings3D(IntRingDiscontinu, RingFootprint) == 2) //Cela signifie que PolyDiscontinu et PolyFootprint sont continus en 3D : ils partagent une arrête commune.
							{
								IndicesPolygonsCommuns.push_back(i); //On stocke ce polygon au cas où PolyDiscontinu veut se lier à un autre footprint afin que l'on ait la liste de tous les footprint concernés.

								delete PolyFootprint;
								test = true;
							}
						}
						if(test)
							break;
					}
				}
				delete PolyFootprint;
			}
		}

		if(IndicesPolygonsCommuns.size() == 0)
			CleanedFootprintsMP.at(IndiceOrigineDesPolygonsDiscontinus.at(ListeDesPolygonsDiscontinusNonAssignes.at(k)))->addGeometry(PolyDiscontinu); //Aucun footprint ne semble continue avec PolyDiscontinu, on va donc le laisser à son footprint initial.
		else if(IndicesPolygonsCommuns.size() >=/*==*/ 1) //Il y a au moins un polygon assigné, on lui ajoute PolyDiscontinu /*///Il n'y a qu'un seul polygon assigné, donc IndicesPolygonsCommuns ne contient qu'un seul indice, celui qui nous intéresse.*/
		{
			CleanedFootprintsMP.at(IndicesPolygonsCommuns.at(0))->addGeometry(PolyDiscontinu);
		}
		/*else if(IndicesPolygonsCommuns.size() > 1 ) //Il y a plusieurs polygons footprint assignés.
		{
		//CleanedFootprintsMP.at(IndicesPolygonsCommuns.at(0))->addGeometry(PolyDiscontinu); ///DECOMMENTER CECI (et commenter le reste) POUR VOIR CE QUE CA DONNE SANS DECOUPE
		ListFootprintsLinkedtoPolyDiscontinu.push_back(IndicesPolygonsCommuns); //Pour le polygon discontinu K, on aura ici la liste des footprints avec lesquels il semble continu en 3D (partage d'une arrête).
		IndicePolygonsDiscontinusToCut.push_back(ListeDesPolygonsDiscontinusNonAssignes.at(k));
		}*/
	}

	for(OGRMultiPolygon* MP : CleanedFootprintsMPSaved2) //On n'a plus besoin de cette version
		delete MP;

	//On met ça en forme dans un vector de Polygon unissant tous les petits polygones, afin d'obtenir des emprises au sol propres.
	std::vector<OGRGeometryCollection*> CleanedFootprints;
	OGRMultiPolygon* Res = new OGRMultiPolygon;

	for(int i = 0; i < CleanedFootprintsMP.size(); ++i)
	{
		OGRMultiPolygon* MP = CleanedFootprintsMP.at(i);

		if(MP->getNumGeometries() == 0)
		{
			CleanedFootprints.push_back(nullptr);
			continue;
		}

		OGRGeometryCollection* GC_Union = new OGRGeometryCollection;

		OGRGeometry* UnionPoly = new OGRPolygon;
		for(int j = 0; j < MP->getNumGeometries(); ++j)
		{
			if(((OGRPolygon*)MP->getGeometryRef(j))->get_Area() < 10*Precision_Vect)
				continue;

			OGRGeometry* tmp = UnionPoly;
			UnionPoly = tmp->Union(MP->getGeometryRef(j));
			delete tmp;
		}
		if(UnionPoly->getGeometryType() == wkbPolygon || UnionPoly->getGeometryType() == wkbPolygon25D)
		{
			GC_Union->addGeometryDirectly(UnionPoly);
			Res->addGeometryDirectly(UnionPoly);
		}
		else
		{
			GC_Union = dynamic_cast<OGRGeometryCollection*>(UnionPoly);
			if(GC_Union == nullptr)
				continue;
			for(int j = 0; j < GC_Union->getNumGeometries(); ++j)
			{
				OGRPolygon* Poly = dynamic_cast<OGRPolygon*>(GC_Union->getGeometryRef(j));
				if(Poly == nullptr)
					continue;
				Res->addGeometryDirectly(Poly);
			}
		}
		CleanedFootprints.push_back(GC_Union);
	}

	for(OGRMultiPolygon* MP : CleanedFootprintsMP)
		delete MP;

	//SaveGeometrytoShape("FootPrintsShape_Modifie.shp", Res);

	return CleanedFootprints;
}

void CompareFootprints(std::vector<OGRPolygon*> FootprintsCityGML, std::vector<OGRPolygon*> FootprintsShp)
{
	int NbGMLNontrouve = 0;
	int NbShpNontrouve = 0;

	OGRMultiPolygon* GMLNontrouve = new OGRMultiPolygon;
	OGRMultiPolygon* ShpNontrouve = new OGRMultiPolygon;

	double AireTotaleGML = 0;
	double AireTotaleShp = 0;
	double AireGMLNontrouve = 0;
	double AireShpNontrouve = 0;
	double AireCommune = 0;

	for(OGRPolygon* GMLPoly:FootprintsCityGML)
	{
		bool test = false;
		for(OGRPolygon* ShpPoly:FootprintsShp)
		{
			if(!GMLPoly->Intersects(ShpPoly))
				continue;

			OGRGeometry* Inter = GMLPoly->Intersection(ShpPoly);
			if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D)
			{
				test = true;
				AireCommune += ((OGRPolygon*)Inter)->get_Area();
				continue;
			}

			OGRGeometryCollection* Inter_GC = dynamic_cast<OGRGeometryCollection*>(Inter);
			if(Inter_GC == nullptr)
				continue;
			for(int i = 0; i < Inter_GC->getNumGeometries(); ++i)
			{
				OGRGeometry* Geo = Inter_GC->getGeometryRef(i);
				if(Geo->getGeometryType() == wkbPolygon || Geo->getGeometryType() == wkbPolygon25D)
				{
					test = true;
					AireCommune += ((OGRPolygon*)Geo)->get_Area();
				}
			}
		}
		if(!test)
		{
			GMLNontrouve->addGeometry(GMLPoly);
			AireGMLNontrouve += GMLPoly->get_Area();
			NbGMLNontrouve ++;
		}
		else
			AireTotaleGML += GMLPoly->get_Area();
	}

	for(OGRPolygon* ShpPoly:FootprintsShp)
	{
		bool test = false;
		for(OGRPolygon* GMLPoly:FootprintsCityGML)
		{
			if(!GMLPoly->Intersects(ShpPoly))
				continue;

			OGRGeometry* Inter = GMLPoly->Intersection(ShpPoly);
			if(Inter->getGeometryType() == wkbPolygon || Inter->getGeometryType() == wkbPolygon25D)
			{
				test = true;
				continue;
			}

			OGRGeometryCollection* Inter_GC = dynamic_cast<OGRGeometryCollection*>(Inter);
			if(Inter_GC == nullptr)
				continue;
			for(int i = 0; i < Inter_GC->getNumGeometries(); ++i)
			{
				OGRGeometry* Geo = Inter_GC->getGeometryRef(i);
				if(Geo->getGeometryType() == wkbPolygon || Geo->getGeometryType() == wkbPolygon25D)
				{
					test = true;
					break;
				}
			}
		}
		if(!test)
		{
			ShpNontrouve->addGeometry(ShpPoly);
			AireShpNontrouve += ShpPoly->get_Area();
			NbShpNontrouve ++;
		}
		else
			AireTotaleShp += ShpPoly->get_Area();
	}

	SaveGeometrytoShape("ShpNontrouve.shp", ShpNontrouve);
	SaveGeometrytoShape("GMLNontrouve.shp", GMLNontrouve);
	std::cout << "NbGML = " << FootprintsCityGML.size() << std::endl;
	std::cout << "NbShp = " << FootprintsShp.size() << std::endl;
	std::cout << "NbGMLNontrouve = " << NbGMLNontrouve << std::endl;
	std::cout << "NbShpNontrouve = " << NbShpNontrouve << std::endl;
	std::cout << "AireTotaleGML = " << AireTotaleGML << std::endl;
	std::cout << "AireTotaleShp = " << AireTotaleShp << std::endl;
	std::cout << "AireCommune = " << AireCommune << std::endl;
	std::cout << "AireGMLNontrouve = " << AireGMLNontrouve << std::endl;
	std::cout << "AireShpNontrouve = " << AireShpNontrouve << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Découpe les bâtiments du fichier CityGML ouvert dans Tile, à partie des bâtiments cadastraux contenus dans le fichier shapefile chargé dans DataSource.
* @param Tile Contient les données du fichier CityGML ouvert : il doit contenir un ensemble de bâtiments LOD2
* @param DataSource Contient les données du fichier Shapefile ouvert : il doit contenir un ensemble d'emprises au sol définissant les bâtiments de la zone étudiée
* @param TexturesList : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie;
*/
citygml::CityModel* CutCityGMLwithShapefile(vcity::Tile* Tile, OGRDataSource* DataSource, std::vector<TextureCityGML*>* TexturesList)
{
	citygml::CityModel* Model = Tile->getCityModel();
	OGRLayer* Layer = DataSource->GetLayer(0);

	//Créer 2 tableaux de polygons correspondant aux emprises au sol des bâtiments du CityGML et à celles du cadastre. 
	std::cout << "Generation des emprises au sol du CityGML." << std::endl;
	std::vector<OGRPolygon*> FootprintsCityGML = GetFootPrintsfromCityGML(Model); //On part du principe que chaque bâtiment CityGML correspond à une emprise au sol Polygon (et non MultiPolygon !). Passage dans SplitBuildingsFromCityGML obligatoire !
	/*OGRMultiPolygon* FootPrintsBuildingsCityGML = new OGRMultiPolygon;
	for(OGRPolygon* Poly:FootprintsCityGML)
	FootPrintsBuildingsCityGML->addGeometry(Poly);
	SaveGeometrytoShape("FootPrintsBuildingsCityGML.shp", FootPrintsBuildingsCityGML);*/
	std::cout << "Generation des emprises au sol du ShapeFile." << std::endl;
	std::vector<OGRPolygon*> FootprintsShapefile = GetFootPrintsfromShapeFile(Layer); //On part du principe que chaque featur du shapefile contient un unique polygon en geometry
	//Lier Bâtiments CityGML <-> Bâtiments Cadastraux qui s'intersectent pour orienter les étapes suivantes.

	//std::vector<OGRGeometryCollection*> FootprintsShapefile_GC; //Pour générer une découpe brute avec les polygons du cadastre non modifiés
	//for(OGRPolygon* Poly : FootprintsShapefile)
	//{
	//	OGRGeometryCollection* GC = new OGRMultiPolygon;
	//	GC->addGeometry(Poly);
	//	FootprintsShapefile_GC.push_back(GC);
	//}

	//CompareFootprints(FootprintsCityGML, FootprintsShapefile);
	//int a;
	//std::cin >> a;

	//Mettre en place un lien entre les éléments du CityGML et ceux du ShapeFile.
	std::cout << "Liaison des emprises au sol du CityGML avec celles du ShapeFile." << std::endl;
	std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> Link = LinkCityGMLandShapefilesFootprints(FootprintsCityGML, FootprintsShapefile);

	//Modifier chaque emprise au sol du Shapefile afin que ses arrêtes correspondent à celles des bâtiments du CityGML
	std::cout << "Modification des polygones du cadastre." << std::endl;
	std::vector<OGRPolygon*> NewFootprintsShapefile = FusionEnvelopes(&FootprintsCityGML, &FootprintsShapefile, &Link);
	for(OGRPolygon* Poly:FootprintsShapefile)
		delete Poly;
	for(OGRPolygon* Poly:FootprintsCityGML)
		delete Poly;

	//Repérer les incohérences en 3D provenant de la découpe 2D afin d'améliorer celle ci.
	std::cout << "Amelioration de la decoupe des emprises au sol." << std::endl;
	std::vector<OGRGeometryCollection*> CleanedFootprints = CleanFootprintsWith3D(&NewFootprintsShapefile, Model, &Link);

	//On a maintenant un ensemble d'emprises au sol (uniquement des Polygons, ce sont juste des zones d'influences non intersectées avec le CityGML, donc pas de MultiPolygon) contenues dans le vecteur
	//NewFootprintsShapefile. Chaque polygon correspond à un bâtiment cadastral et son emprise au sol recouvre une partie de l'emprise au sol d'un bâtiment CityGML lié. L'ensemble des emprises au sol
	//de NewFootprintsShapefile doit recouvrir les emprises au sol de tous les bâtiments CityGML. Il nous reste à parcourir les polygons du CityGML et à les assigner à ces différents bâtiments Shape.
	std::cout << "Decoupe des Polygons Wall et Roof du CityGML selon les emprises au sol generees a partir du cadastre." << std::endl;
	citygml::CityModel* ModelOut = AssignPolygonGMLtoShapeBuildings(&/*FootprintsShapefile_GC*/CleanedFootprints/*NewFootprintsShapefile*/, Layer, Model, &Link, TexturesList);

	std::cout << "Decoupe terminee." << std::endl;
	for(OGRPolygon* Poly:NewFootprintsShapefile)
		delete Poly;

	for(OGRGeometryCollection* GC:CleanedFootprints)
		delete GC;

	return ModelOut;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Parcourt tous les bâtiments du model, récupère leurs polygones de toit, et les unis afin d'obtenir une liste de polygones disjoints
* @param model : Contient les bâtiments que l'on souhaite traiter
* @param BuildingsFootprints : résultat
* @param Names : contient la liste de noms associés à la liste de Polygons/Bâtiments contenus dans BuildingsFootprints
*/
OGRMultiPolygon* GetBuildingsFootprintsFromCityModel(citygml::CityModel* model, std::vector<std::string>* Names)
{
	OGRMultiPolygon* BuildingsFootprints = new OGRMultiPolygon;
	int cpt = 0;

	std::vector<OGRMultiPolygon *> ListEnveloppes;
	std::vector<std::string> NameBuildings;

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
							OGRLinearRing * OgrRing = new OGRLinearRing;

							for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								OgrRing->addPoint(Point.x, Point.y, Point.z);

							OgrRing->closeRings();

							if(OgrRing->getNumPoints() > 3)
							{
								OGRPolygon * OgrPoly = new OGRPolygon;
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid())
								{
									Building->addGeometryDirectly(OgrPoly);
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
				std::cout << "Avancement etape 1 : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
				continue;
			}

			OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);

			ListEnveloppes.push_back(Enveloppe);
			NameBuildings.push_back(obj->getId());

			if(BuildingsFootprints == nullptr)
				BuildingsFootprints = Enveloppe;
			else
			{
				OGRMultiPolygon * tmp = BuildingsFootprints;
				BuildingsFootprints = (OGRMultiPolygon *)tmp->Union(Enveloppe);
				delete tmp;
			}
		}

		cpt++;
		std::cout << "Ouverture des batiments en entree : " << cpt << "/" << model->getCityObjectsRoots().size() << " batiments traites.\r" << std::flush;
	}
	std::cout << std::endl;

	Names->resize(BuildingsFootprints->getNumGeometries(), "null");

	//On veut assigner le nom de bâtiment initial à chaque sous bâtiment que l'on vient de créer, avec une numérotation permettant de les distinguer les uns des autres
	for(int i = 0; i < ListEnveloppes.size(); ++i)
	{
		cpt = 0;
		OGRMultiPolygon * CurrentMP = ListEnveloppes.at(i);
		for(int j = 0; j < BuildingsFootprints->getNumGeometries(); ++j)
		{
			if(Names->at(j) != "null")//Le nom est déjà rempli, inutile de faire les tests suivants
				continue;

			OGRPolygon* CurrentGeo = (OGRPolygon*) BuildingsFootprints->getGeometryRef(j);
			if(CurrentMP->Intersects(CurrentGeo))
			{
				++ cpt;
				Names->at(j) = NameBuildings.at(i) + "_" + std::to_string(cpt);
			}
		}
	}

	return BuildingsFootprints;
}

////////////////////////////////////////////////////////////////////////////////

/**
* @brief Traite un fichier CityGML afin de définir chaque objet 3D isolé comme étant un bâtiment : ressort le CityModel contenant tous ces nouveaux bâtiments
* @param Tile : Contient les données du CityGML ouvert
*/
citygml::CityModel* SplitBuildingsFromCityGML(vcity::Tile* Tile, std::vector<TextureCityGML*>* TexturesList)
{
	citygml::CityModel* model = Tile->getCityModel();
	citygml::CityModel* ModelOut = new citygml::CityModel;

	int cpt = 0;

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
							OGRLinearRing * OgrRing = new OGRLinearRing;

							for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								OgrRing->addPoint(Point.x, Point.y, Point.z);

							OgrRing->closeRings();

							if(OgrRing->getNumPoints() > 3)
							{
								OGRPolygon * OgrPoly = new OGRPolygon;
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid())
								{
									Building->addGeometryDirectly(OgrPoly);
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
				std::cout << "Avancement : " << cpt << " / " << model->getCityObjectsRoots().size() << ".\r" << std::flush;
				continue;
			}

			/*std::cout << obj->getId() << std::endl;
			SaveGeometrytoShape("Polygons.shp", Building);
			int a;
			std::cin >> a;*/

			OGRMultiPolygon * Enveloppe = GetEnveloppe(Building);

			delete Building;

			std::vector<citygml::Geometry*> ListRoofs;
			std::vector<citygml::Geometry*> ListWalls;

			std::string NameBuilding = obj->getId();

			for(int i = 0; i < Enveloppe->getNumGeometries(); ++i)
			{
				std::string Name = NameBuilding + "_" + std::to_string(i);
				citygml::Geometry* Roof = new citygml::Geometry(Name+"_RoofGeometry", citygml::GT_Roof, 2);
				citygml::Geometry* Wall = new citygml::Geometry(Name+"_WallGeometry", citygml::GT_Wall, 2);
				ListRoofs.push_back(Roof);
				ListWalls.push_back(Wall);
			}

			for(citygml::CityObject* object : obj->getChildren())
			{
				if(object->getType() == citygml::COT_RoofSurface)
				{
					for(citygml::Geometry* Geometry : object->getGeometries())
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
						{
							for(int i = 0; i < Enveloppe->getNumGeometries(); ++i)
							{
								OGRGeometry * Building = Enveloppe->getGeometryRef(i); //Batiment de référence que l'on cherche à remplir d'objets CityGML

								bool PolyIsInBati = true;

								for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								{
									OGRPoint* P = new OGRPoint(Point.x, Point.y);

									if(!P->Intersects(Building)) //Si un point ne se retrouve pas dans Building, alors le polygon correspondant ne doit pas lui être associé.
									{
										PolyIsInBati = false;
										delete P;
										break;
									}
									delete P;
								}
								if(PolyIsInBati)
								{
									ListRoofs.at(i)->addPolygon(PolygonCityGML->Clone());

									if(PolygonCityGML->getTexture() == nullptr)
										continue;

									//Remplissage de ListTextures
									std::string Url = PolygonCityGML->getTexture()->getUrl();
									citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

									TexturePolygonCityGML Poly;

									Poly.Id = PolygonCityGML->getId();
									Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
									Poly.TexUV = PolygonCityGML->getTexCoords();

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
									break;
								}
							}
						}
					}
				}
				else if(object->getType() == citygml::COT_WallSurface)
				{
					for(citygml::Geometry* Geometry : object->getGeometries())
					{
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
						{
							for(int i = 0; i < Enveloppe->getNumGeometries(); ++i)
							{
								OGRGeometry * Building = Enveloppe->getGeometryRef(i); //Batiment de référence que l'on cherche à remplir d'objets CityGML
								bool PolyIsInBati = true;
								for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
								{
									OGRPoint* P = new OGRPoint(Point.x, Point.y);
									if(P->Distance(Building)> Precision_Vect) //Si un point ne se retrouve pas dans Building, alors le polygon correspondant ne doit pas lui être associé. Distance au lieu de intersection à cause des imprécisions 
										//entre certains points de Wall par rapport à l'emprise au sol définie par les Roof
									{
										PolyIsInBati = false;
										delete P;
										break;
									}
									delete P;
								}
								if(PolyIsInBati) //Si tous les points du polygone sont dans Building, il faut l'ajouter à celui ci
								{
									ListWalls.at(i)->addPolygon(PolygonCityGML->Clone());

									if(PolygonCityGML->getTexture() == nullptr)
										continue;

									//Remplissage de ListTextures
									std::string Url = PolygonCityGML->getTexture()->getUrl();
									citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

									TexturePolygonCityGML Poly;

									Poly.Id = PolygonCityGML->getId();
									Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
									Poly.TexUV = PolygonCityGML->getTexCoords();

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
									break;
								}
							}
						}
					}
				}
			}

			for(int i = 0; i < Enveloppe->getNumGeometries(); ++i)
			{
				std::string Name = NameBuilding + "_" + std::to_string(i);
				citygml::CityObject* BuildingCO = new citygml::Building(Name);
				citygml::CityObject* RoofCO = new citygml::RoofSurface(Name+"_Roof");
				citygml::CityObject* WallCO = new citygml::WallSurface(Name+"_Wall");

				RoofCO->addGeometry(ListRoofs.at(i));
				ModelOut->addCityObject(RoofCO);
				BuildingCO->insertNode(RoofCO);
				WallCO->addGeometry(ListWalls.at(i));
				ModelOut->addCityObject(WallCO);
				BuildingCO->insertNode(WallCO);

				ModelOut->addCityObject(BuildingCO);
				ModelOut->addCityObjectAsRoot(BuildingCO);
			}

			delete Enveloppe;
		}

		++cpt;
		std::cout << "Avancement : " << cpt << " / " << model->getCityObjectsRoots().size() << ".\r" << std::flush;
	}
	std::cout << std::endl;
	return ModelOut;

	/*std::vector<std::string> Names;

	OGRMultiPolygon * BuildingsFootprints = GetBuildingsFootprintsFromCityModel(model, &Names); //Permet de définir l'existence des bâtiments recherchés, il faut maintenant leur assigner des CityObject complets (Wall + Roof + ...)

	//Maintenant qu'on a la liste des bâtiments désirés, il va falloir assigner à chacun les Wall et Roof qui correspondent en reparcourant
	// tous les bâtiments et en regardant quels sont les polygons se situant au même niveau que les emprises au sol sauvegardées dans BuildingsFootprints

	for(int i = 0; i < BuildingsFootprints->getNumGeometries(); ++i)
	{
	OGRGeometry * Building = BuildingsFootprints->getGeometryRef(i); //Batiment de référence que l'on cherche à remplir d'objets CityGML

	std::string Name = Names.at(i);
	citygml::CityObject* BuildingCO = new citygml::Building(Name);
	citygml::CityObject* RoofCO = new citygml::RoofSurface(Name+"_Roof");
	citygml::Geometry* Roof = new citygml::Geometry(Name+"_RoofGeometry", citygml::GT_Roof, 2);
	citygml::CityObject* WallCO = new citygml::WallSurface(Name+"_Wall");
	citygml::Geometry* Wall = new citygml::Geometry(Name+"_WallGeometry", citygml::GT_Wall, 2);

	for(citygml::CityObject* obj : model->getCityObjectsRoots())
	{
	if(obj->getType() == citygml::COT_Building)
	{
	for(citygml::CityObject* object : obj->getChildren())
	{
	if(object->getType() == citygml::COT_RoofSurface)
	{
	for(citygml::Geometry* Geometry : object->getGeometries())
	{
	for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
	{
	bool PolyIsInBati = true;

	for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
	{
	OGRPoint* P = new OGRPoint(Point.x, Point.y);

	if(!P->Intersects(Building)) //Si un point ne se retrouve pas dans Building, alors le polygon correspondant ne doit pas lui être associé.
	{
	PolyIsInBati = false;
	break;
	}
	}
	if(PolyIsInBati)
	{
	Roof->addPolygon(PolygonCityGML);

	if(PolygonCityGML->getTexture() == nullptr)
	continue;

	//Remplissage de ListTextures
	std::string Url = PolygonCityGML->getTexture()->getUrl();
	citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

	TexturePolygonCityGML Poly;

	Poly.Id = PolygonCityGML->getId();
	Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
	Poly.TexUV = PolygonCityGML->getTexCoords();

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
	}
	}
	}
	else if(object->getType() == citygml::COT_WallSurface)
	{
	for(citygml::Geometry* Geometry : object->getGeometries())
	{
	for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
	{
	bool PolyIsInBati = true;
	for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
	{
	OGRPoint* P = new OGRPoint(Point.x, Point.y);
	if(P->Distance(Building)> Precision_Vect) //Si un point ne se retrouve pas dans Building, alors le polygon correspondant ne doit pas lui être associé. Distance au lieu de intersection à cause des imprécisions 
	//entre certains points de Wall par rapport à l'emprise au sol définie par les Roof
	{
	PolyIsInBati = false;
	delete P;
	break;
	}
	delete P;
	}
	if(PolyIsInBati) //Si tous les points du polygone sont dans Building, il faut l'ajouter à celui ci
	{
	Wall->addPolygon(PolygonCityGML);

	if(PolygonCityGML->getTexture() == nullptr)
	continue;

	//Remplissage de ListTextures
	std::string Url = PolygonCityGML->getTexture()->getUrl();
	citygml::Texture::WrapMode WrapMode = PolygonCityGML->getTexture()->getWrapMode();

	TexturePolygonCityGML Poly;

	Poly.Id = PolygonCityGML->getId();
	Poly.IdRing =  PolygonCityGML->getExteriorRing()->getId();
	Poly.TexUV = PolygonCityGML->getTexCoords();

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
	}
	}
	}
	}
	}
	}
	RoofCO->addGeometry(Roof);
	ModelOut->addCityObject(RoofCO);
	BuildingCO->insertNode(RoofCO);
	WallCO->addGeometry(Wall);
	ModelOut->addCityObject(WallCO);
	BuildingCO->insertNode(WallCO);

	ModelOut->addCityObject(BuildingCO);
	ModelOut->addCityObjectAsRoot(BuildingCO);

	std::cout << "Generation des batiments : " << i+1 << "/" << BuildingsFootprints->getNumGeometries() << " batiments crees.\r" << std::flush;
	}
	std::cout << std::endl;*/

	return ModelOut;
}

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Ouvre un fichier CityGML de MNT, un Shapefile contenant un ou plusieurs polygones et ne conserve que le MNT se trouvant à l'intérieur. Les polygones du MNT ne sont pas découpés, on n'aura donc pas forme exacte du Shapefile en sortie.
* @param Tile : Contient les données du CityGML ouvert
* @param Shapefile : Contient le ou les polygones définissant la zone de MNT à conserver
* @param TexturesList : Contient la liste des textures liées aux polygones du CityModel de sortie
*/
citygml::CityModel* CutMNTwithShapefile(vcity::Tile* Tile, OGRDataSource* ShapeFile, std::vector<TextureCityGML*>* TexturesList)
{
	OGRLayer* Layer = ShapeFile->GetLayer(0);

	OGRMultiPolygon* Polygons = new OGRMultiPolygon;

	OGRFeature *Feature;
	Layer->ResetReading();

	while((Feature = Layer->GetNextFeature()) != NULL)
	{
		OGRGeometry* Geometry = Feature->GetGeometryRef();

		if(Geometry->getGeometryType() == wkbPolygon || Geometry->getGeometryType() == wkbPolygon25D)
			Polygons->addGeometry(Geometry);
		else if(Geometry->getGeometryType() == wkbMultiPolygon || Geometry->getGeometryType() == wkbMultiPolygon25D)
		{
			for(int i = 0; i < ((OGRMultiPolygon*)Geometry)->getNumGeometries(); ++i)
				Polygons->addGeometry(((OGRMultiPolygon*)Geometry)->getGeometryRef(i));
		}
	}

	citygml::CityModel* MNT_Out = new citygml::CityModel;
	citygml::CityModel* Model = Tile->getCityModel();

	int cpt1 = -1;

	for(citygml::CityObject* obj : Model->getCityObjectsRoots())
	{
		++cpt1;

		std::cout << "Avancement : " << cpt1 << " / " << Model->getCityObjectsRoots().size() << std::endl;

		if(obj->getType() == citygml::COT_TINRelief || obj->getType() == citygml::COT_WaterBody)
		{
			std::string Name = obj->getId();
			citygml::CityObject* TIN_CO;
			if(obj->getType() == citygml::COT_TINRelief)
				TIN_CO = new citygml::TINRelief(Name);
			else if(obj->getType() == citygml::COT_WaterBody)
				TIN_CO = new citygml::WaterBody(Name);

			citygml::Geometry* TIN_Geo = new citygml::Geometry(Name + "_TINGeometry", citygml::GT_Unknown, 2);

			for(citygml::Geometry* Geometry : obj->getGeometries())
			{
				for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
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

					std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

					OGRLinearRing * OgrRing = new OGRLinearRing;
					for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
						OgrRing->addPoint(Point.x, Point.y, Point.z);

					if(PolygonCityGML->getTexture()->getType() == "GeoreferencedTexture") //Ce sont des coordonnées géoréférences qu'il faut convertir en coordonnées de texture standard
					{
						/*double A, B, C ,D; //Voir fr.wikipedia.org/wiki/World_file : Taille pixel, rotation, retournement //Pour faire une conversion propre.
						double offset_x;
						double offset_y;

						std::string path = PathFolder + "/" + PolygonCityGML->getTexture()->getUrl().substr(0, PolygonCityGML->getTexture()->getUrl().find_last_of('.'))+".jgw";
						std::cout << path << std::endl;
						std::ifstream fichier(path, std::ios::in);

						if(fichier)
						{
						fichier >> A >> B >> C >> D >> offset_x >> offset_y;
						fichier.close();
						}
						std::cout << A << " " << B << " " << C << " " << D << " " << offset_x << " " << offset_y << std::endl;*/


						//////////////////////////////// MARCHE POUR DES TEXTURES 4096x4096 avec un D négatif (données de LYON)
						int i = 0;
						for(TVec2f UV:TexUV)
						{
							UV.x = UV.x/4095; 
							UV.y = 1 + UV.y/4095;//Car D est négatif
							TexUV.at(i) = UV;
							++i;
						}
					}

					OgrRing->closeRings();
					if(OgrRing->getNumPoints() > 3)
					{
						OGRPolygon * OgrPoly = new OGRPolygon;
						OgrPoly->addRingDirectly(OgrRing);
						if(!OgrPoly->IsValid())
							continue;
						if(OgrPoly->Intersects(Polygons))
						{
							TIN_Geo->addPolygon(PolygonCityGML);
							if(HasTexture)
							{
								TexturePolygonCityGML Poly;

								Poly.Id = PolygonCityGML->getId();
								Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
								Poly.TexUV = TexUV;

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
						}
						delete OgrPoly;
					}
					else
						delete OgrRing;
				}
			}
			if(TIN_Geo->getPolygons().size() > 0)
			{
				TIN_CO->addGeometry(TIN_Geo);
				MNT_Out->addCityObject(TIN_CO);
				MNT_Out->addCityObjectAsRoot(TIN_CO);
			}
		}
	}

	++cpt1;

	std::cout << "Avancement : " << cpt1 << " / " << Model->getCityObjectsRoots().size() << std::endl;

	return MNT_Out;
}