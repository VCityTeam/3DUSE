#include <algorithm>
#include "OGRGDALtools.hpp"

/**
* @brief Recupere l'enveloppe d'une geometry en faisant une succession d'unions sur tous les polygones
* @param MP Ensemble de polygones sur lequel on va generer une enveloppe
*/
OGRMultiPolygon * GetEnveloppe(OGRMultiPolygon * MP)
{
    if (MP->IsEmpty())
        return nullptr;

    OGRGeometry* ResUnion = new OGRMultiPolygon;

    ResUnion = MP->UnionCascaded();

    //On travaille avec des OGRMultiPolygon pour avoir un format universel, il faut donc transformer la geometry en collection.
    if (ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon25D)//La geometry est en fait un ensemble de polygons : plusieurs baitments
    {
        OGRMultiPolygon * GeoCollection = (OGRMultiPolygon*)(ResUnion);

        return GeoCollection;		//////////// Ignore le retrait des interior ring plats

        OGRMultiPolygon * MultiPolygonRes = new OGRMultiPolygon;

        for (int i = 0; i < GeoCollection->getNumGeometries(); ++i)//Pour enlever les aretes parasites formant des interior ring "plats"
        {
            OGRGeometry * Geometry = GeoCollection->getGeometryRef(i);

            if (Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon || Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon25D)
            {
                OGRPolygon * Poly = dynamic_cast<OGRPolygon*>(Geometry);
                OGRPolygon * PolyRes = new OGRPolygon;

                PolyRes->addRing(Poly->getExteriorRing());

                for (int j = 0; j < Poly->getNumInteriorRings(); ++j)
                {
                    const OGRLinearRing * IntRing = Poly->getInteriorRing(j);

                    if (IntRing->get_Area() > 0.1) //Pour enlever les aretes parasites formant des interior ring "plats"
                        PolyRes->addRingDirectly(dynamic_cast<OGRLinearRing*>(IntRing->clone()));
                }

                MultiPolygonRes->addGeometryDirectly(PolyRes);
            }
        }

        return MultiPolygonRes;
    }
    else if (ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon25D)//La geometry est en fait un seul polygon : un seul batiment
    {
        OGRMultiPolygon * GeoCollection = new OGRMultiPolygon;
        GeoCollection->addGeometryDirectly(ResUnion);
        return GeoCollection;
    }

    return nullptr;
}

/**
* @brief ProjectPointOnPolygon3D : prend un point 2D Point et calcule sa coordonnee Z en partant du principe qu'il est coplanaire a Polygon
* @param Point : point que l'on veut extruder en 3D
* @param Polygon : polygon qui definit le plan sur lequel vient se poser Point
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

    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe a 1 tant que C n'est pas correctement rempli
    for (int i = 1; i < Ring->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une repetition du premier
    {
        if (test == 0)
        {
            B.x = Ring->getX(i);
            B.y = Ring->getY(i);
            B.z = Ring->getZ(i);

            if (A.x != B.x || A.y != B.y)
            {
                ++test;// A est bien different de B
                AB = B - A;
            }
        }
        else if (test == 1)
        {
            C.x = Ring->getX(i);
            C.y = Ring->getY(i);
            C.z = Ring->getZ(i);

            /*if ((C.x - A.x) / (B.x - A.x) != (C.y - A.y) / (B.y - A.y))
            {
                ++test;// C n'est pas aligné avec A et B => A B C forment bien un plan
                -AC = C - A;
                break;
            }*/

            AC = C - A;

            if (AB.x == 0 && AC.x != 0)
            {
                ++test;// C n'est pas aligne avec A et B => A B C forment bien un plan
                break;
            }
            if (AB.y == 0 && AC.y != 0)
            {
                ++test;// C n'est pas aligne avec A et B => A B C forment bien un plan
                break;
            }
            if (AB.x != 0 && AB.y != 0 && AC.x / AB.x != AC.y / AB.y)
            {
                ++test;// C n'est pas aligne avec A et B => A B C forment bien un plan
                break;
            }
        }
    }

    if (test != 2)
    {
        std::cout << "Erreur lors de la creation du plan. \n";
        return nullptr;
    }

    TVec3d M; // <=> Point
    M.x = Point->getX();
    M.y = Point->getY();

    double s, t;

    //t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
    //s = (M.x - A.x - t * AC.x) / AB.x;

    if (AB.x != 0)
    {
        t = (A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y);
        s = (M.x - A.x - t * AC.x) / AB.x;
    }
    else //AB.x = 0 donc AC.x ne peut pas etre egal a 0 non plus (car A et B ont ete choisis pour etre distints)
    {
        t = (A.x * AB.y - A.y * AB.x + AB.x * M.y - AB.y * M.x) / (AB.x * AC.y - AB.y * AC.x);
        s = (M.y - A.y - t * AC.y) / AB.y;
    }

    M.z = A.z + s * AB.z + t * AC.z;

    return new OGRPoint(M.x, M.y, M.z);
}

/**
* @brief Calcule la distance de Hausdorff unidirectionnelle entre un nuage de points et un ensemble de triangles
* @param GeoPoints Correspond a la geometrie contenant le nuage de points et que l'on va projeter sur la seconde geometrie
* @param Geo Correspond aux triangles sur lesquels seront projetes les points
*/
double Hausdorff(OGRMultiPolygon * GeoPoints, OGRMultiPolygon * Geo)
{
    std::vector<OGRPoint *> Points;

    for (int i = 0; i < GeoPoints->getNumGeometries(); ++i)
    {
        OGRPolygon * Poly = (OGRPolygon *)GeoPoints->getGeometryRef(i);
        OGRLinearRing * Ring = Poly->getExteriorRing();
        for (int j = 0; j < Ring->getNumPoints(); ++j)
        {
            OGRPoint * Point = new OGRPoint;
            Ring->getPoint(j, Point);
            Points.push_back(Point);
        }
    }

    for (int i = 0; i < Geo->getNumGeometries(); ++i) //On parcourt tous les polygones pour transformer les eventuels rectangles (ou autres polygones convexes a 4 cates) en deux triangles : temporaire, ne fonctionne que pour ces polygones precis
    {
        OGRPolygon * Poly = (OGRPolygon *)Geo->getGeometryRef(i);
        OGRLinearRing * Ring = Poly->getExteriorRing();
        if (Ring->getNumPoints() == 5)
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
    for (size_t i = 0; i < Points.size(); ++i)
    {
        int J = -1;
        double D_1 = 10000;
        OGRPoint * OgrP0 = Points.at(i);
        TVec3d P0(OgrP0->getX(), OgrP0->getY(), OgrP0->getZ());
        for (int j = 0; j < Geo->getNumGeometries(); ++j)
        {
            double D_2;
            OGRPolygon * PolyTriangle = (OGRPolygon *)Geo->getGeometryRef(j);
            OGRLinearRing * Triangle = PolyTriangle->getExteriorRing();
            if (Triangle->getNumPoints() > 4 || Triangle->get_Area() < 0.01)//Si la geometry courante n'est pas un triangle
            {
                continue;
            }
            OGRPoint * OgrP1 = new OGRPoint; //Point du triangle
            OGRPoint * OgrP2 = new OGRPoint;
            OGRPoint * OgrP3 = new OGRPoint;
            Triangle->getPoint(0, OgrP1);
            Triangle->getPoint(1, OgrP2);
            Triangle->getPoint(2, OgrP3);

            TVec3d P1(OgrP1->getX(), OgrP1->getY(), OgrP1->getZ());
            TVec3d P2(OgrP2->getX(), OgrP2->getY(), OgrP2->getZ());
            TVec3d P3(OgrP3->getX(), OgrP3->getY(), OgrP3->getZ());

            delete OgrP1;
            delete OgrP2;
            delete OgrP3;

            TVec3d P1P0 = P0 - P1; //Vecteur P1P0
            TVec3d P2P0 = P0 - P2;
            TVec3d P3P0 = P0 - P3;

            double nP1P0 = P1P0.length(); //Norme du vecteur P1P0
            double nP2P0 = P2P0.length();
            double nP3P0 = P3P0.length();

            if (nP1P0 == 0 || nP2P0 == 0 || nP3P0 == 0) // Si le point P0 est confondu avec l'un des points du triangle
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

            double cosa = (P1P0.x * Np.x + P1P0.y * Np.y + P1P0.z * Np.z) / (nP1P0 * nNp); //Calcul du cosinus de langle a entre Np et P1P0

            double nP0P0_ = nP1P0 * cosa;
            TVec3d P0P0_(-nP0P0_ * Np.x / nNp, -nP0P0_ * Np.y / nNp, -nP0P0_ * Np.z / nNp); //Vecteur P0P0_, P0_ etant le projete de P0 sur le plan du triangle

            TVec3d P0_(P0.x + P0P0_.x, P0.y + P0P0_.y, P0.z + P0P0_.z); // Position du projete de P0 sur le plan du triangle

            double s, t;//Coordonnees barycentriques du point P0_ par rapport au point P1 et aux vecteurs P1P2 et P1P3

            t = (P1.y * P1P2.x - P1.x * P1P2.y + P1P2.y * P0_.x - P1P2.x * P0_.y) / (P1P2.y * P1P3.x - P1P2.x * P1P3.y);
            s = (P0_.x - P1.x - t * P1P3.x) / P1P2.x;

            if (t >= 0 && s >= 0 && t + s <= 1)//Le projete est dans le triangle
            {
                D_2 = nP0P0_;
                if (D_1 > D_2)
                {
                    J = j;
                    D_1 = D_2;
                }
            }
            else//Le projete est en dehors du triangle
            {
                //On va donc le comparer aux trois arretes du triangle en le projetant a nouveau sur celles-ci
                TVec3d P0_P1 = P1 - P0_;//Vecteur P0_P1
                TVec3d P0_P2 = P2 - P0_;
                TVec3d P0_P3 = P3 - P0_;
                double nP0_P1 = P0_P1.length();//Norme du vecteur P0_P1
                double nP0_P2 = P0_P2.length();

                //Sur P1P2 :
                TVec3d Temp(P0_P2.y * P0_P1.z - P0_P2.z * P0_P1.y, P0_P2.z * P0_P1.x - P0_P2.x * P0_P1.z, P0_P2.x * P0_P1.y - P0_P2.y * P0_P1.x);
                TVec3d R(Temp.y * P1P2.z - Temp.z * P1P2.y, Temp.z * P1P2.x - Temp.x * P1P2.z, Temp.x * P1P2.y - Temp.y * P1P2.x); //Direction de P0_ -> P0__
                double nR = R.length();

                double cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z) / (nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

                double nP0_P0__ = nP0_P1 * cosg;
                TVec3d P0_P0__(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__etant le projete de P0_ sur l'arrete courante du triangle
                TVec3d P0__(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projete de P0_ sur l'arrete du triangle

                double u = (P0__.x - P1.x) / (P1P2.x); //Position relative de P0__ sur le segment P1P2

                if (u <= 0)
                    D_2 = nP1P0; //P0 est le plus proche de P1
                else if (u >= 1)
                    D_2 = nP2P0; //P0 est le plus proche de P2
                else
                    D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

                if (D_1 > D_2)
                {
                    J = j;
                    D_1 = D_2;
                }

                //Sur P1P3 :
                Temp = TVec3d(P0_P3.y * P0_P1.z - P0_P3.z * P0_P1.y, P0_P3.z * P0_P1.x - P0_P3.x * P0_P1.z, P0_P3.x * P0_P1.y - P0_P3.y * P0_P1.x);
                R = TVec3d(Temp.y * P1P3.z - Temp.z * P1P3.y, Temp.z * P1P3.x - Temp.x * P1P3.z, Temp.x * P1P3.y - Temp.y * P1P3.x); //Direction de P0_ -> P0__
                nR = R.length();

                cosg = (P0_P1.x * R.x + P0_P1.y * R.y + P0_P1.z * R.z) / (nP0_P1 * nR); //Calcul du cosinus de langle g entre R et P0_P1

                nP0_P0__ = nP0_P1 * cosg;
                P0_P0__ = TVec3d(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__etant le projete de P0_ sur l'arrete courante du triangle
                P0__ = TVec3d(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projete de P0_ sur l'arrete du triangle

                u = (P0__.x - P1.x) / (P1P3.x); //Position relative de P0__ sur le segment P1P3

                if (u <= 0)
                    D_2 = nP1P0; //P0 est le plus proche de P1
                else if (u >= 1)
                    D_2 = nP3P0; //P0 est le plus proche de P3
                else
                    D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

                if (D_1 > D_2)
                {
                    J = j;
                    D_1 = D_2;
                }

                //Sur P2P3 :
                Temp = TVec3d(P0_P3.y * P0_P2.z - P0_P3.z * P0_P2.y, P0_P3.z * P0_P2.x - P0_P3.x * P0_P2.z, P0_P3.x * P0_P2.y - P0_P3.y * P0_P2.x);
                R = TVec3d(Temp.y * P2P3.z - Temp.z * P2P3.y, Temp.z * P2P3.x - Temp.x * P2P3.z, Temp.x * P2P3.y - Temp.y * P2P3.x); //Direction de P0_ -> P0__
                nR = R.length();

                cosg = (P0_P2.x * R.x + P0_P2.y * R.y + P0_P2.z * R.z) / (nP0_P2 * nR); //Calcul du cosinus de langle g entre R et P0_P2

                nP0_P0__ = nP0_P2 * cosg;
                P0_P0__ = TVec3d(nP0_P0__ * R.x / nR, nP0_P0__ * R.y / nR, nP0_P0__ * R.z / nR); //Vecteur P0_P0__, P0__etant le projete de P0_ sur l'arrete courante du triangle
                P0__ = TVec3d(P0_.x + P0_P0__.x, P0_.y + P0_P0__.y, P0_.z + P0_P0__.z); // Position du projete de P0_ sur l'arrete du triangle

                u = (P0__.x - P2.x) / (P2P3.x); //Position relative de P0__ sur le segment P2P3

                if (u <= 0)
                    D_2 = nP2P0; //P0 est le plus proche de P2
                else if (u >= 1)
                    D_2 = nP3P0; //P0 est le plus proche de P3
                else
                    D_2 = sqrt(nP0_P0__ * nP0_P0__ + nP0P0_ * nP0P0_); //P0 est le plus proche de P0__

                if (D_1 > D_2)
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
        if (D_1 > D)
            D = D_1;
    }
    //int a;
    //std::cin >> a;
    return D;
}

/**
* @brief Calcule la distance de Hausdorff entre deux batiments composes de triangles : methode basee sur "3D Distance from a Point to a Triangle", Mark W.Jones, 1995 (DistancePointTriangle.pdf).
* @param Geo1 Geometrie correspondant au premier batiment
* @param Geo2 Geometrie correspondant au second batiment
*/
double DistanceHausdorff(OGRMultiPolygon * Geo1, OGRMultiPolygon * Geo2)
{
    double D12 = 0;//Distance de Geo1 a Geo2
    double D21 = 0;//Distance de Geo2 a Geo1

    D12 = Hausdorff(Geo1, Geo2);
    D21 = Hausdorff(Geo2, Geo1);

    return std::max(D12, D21);
}

/**
* @brief Projette un polygone 2D Poly2D sur le plan forme par un polygone 3D Poly3D
* @param Poly2D Polygone 2D
* @param Poly3D Polygone 3D
*/
OGRPolygon* ProjectPolyOn3DPlane(OGRPolygon* Poly2D, OGRPolygon * Poly3D)
{
    OGRLinearRing * Ring2D = Poly2D->getExteriorRing();
    OGRLinearRing * Ring3D = Poly3D->getExteriorRing();

    //Il faut prendre trois points A,B,C non alignes de Ring3D pour pouvoir etablir l'equation parametrique du plan 3D forme par ce Polygone
    OGRPoint * A = new OGRPoint;
    OGRPoint * B = new OGRPoint;
    OGRPoint * C = new OGRPoint;

    TVec3d AB;
    TVec3d AC;
    TVec3d AM;

    Ring3D->getPoint(0, A);
    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe a 1 tant que C n'est pas correctement rempli
    for (int i = 1; i < Ring3D->getNumPoints(); ++i)
    {
        if (test == 0)
        {
            Ring3D->getPoint(i, B);
            if (A != B)
            {
                AB.x = B->getX() - A->getX();
                AB.y = B->getY() - A->getY();
                AB.z = B->getZ() - A->getZ();
                ++test;
            }
        }
        else if (test == 1)
        {
            Ring3D->getPoint(i, C);
            if ((C->getX() - A->getX()) / (B->getX() - A->getX()) != (C->getY() - A->getY()) / (B->getY() - A->getY())) //C n'est pas aligne avec A et B
            {
                AC.x = C->getX() - A->getX();
                AC.y = C->getY() - A->getY();
                AC.z = C->getZ() - A->getZ();
                ++test;
                break;
            }
        }
    }
    if (test != 2)
    {
        std::cout << "Erreur lors de la creation du plan. \n";
        delete A;
        delete B;
        delete C;
        return nullptr;
    }

    OGRLinearRing * Ring = new OGRLinearRing;

    for (int i = 0; i < Ring2D->getNumPoints(); ++i)
    {
        OGRPoint * M = new OGRPoint;
        Ring2D->getPoint(i, M);
        double s, t;

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
    for (int i = 0; i < Poly2D->getNumGeometries(); ++i)
    {
        OGRPolygon* Poly = (OGRPolygon*)Poly2D->getGeometryRef(i);
        OGRPolygon* Res = ProjectPolyOn3DPlane(Poly, Poly3D);
        MultiPolygon->addGeometryDirectly(Res);
    }
    return MultiPolygon;
}

/**
* @brief ChangePointsOrderForNormal : Modifie l'orientation du polygone pour que sa normale soit orientee vers le haut (pour MNT et Roof).
* @param Ring : Contient les points formant le polygone
* @param Tex : Contient les coordonnees de textures liees aux points de Ring, il faut egalement modifier leur ordre si on veut conserver l'information de texture
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

    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe a 1 tant que C n'est pas correctement rempli
    for (int i = 1; i < Ring->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une repetition du premier
    {
        if (test == 0)
        {
            B.x = Ring->getX(i);
            B.y = Ring->getY(i);
            B.z = Ring->getZ(i);

            if (A.x != B.x || A.y != B.y)
            {
                ++test;// A est bien different de B
                AB = B - A;
            }
        }
        else if (test == 1)
        {
            C.x = Ring->getX(i);
            C.y = Ring->getY(i);
            C.z = Ring->getZ(i);

            if ((C.x - A.x) / (B.x - A.x) != (C.y - A.y) / (B.y - A.y))
            {
                ++test;// C n'est pas aligne avec A et B => A B C forment bien un plan
                AC = C - A;
                break;
            }
        }
    }

    double NormZ = AB.x * AC.y - AB.y * AC.x;

    if (NormZ >= 0) //Le polygone est bien oriente pour que sa normale soit vers le haut, il n'y a donc rien a changer.
        return;

    OGRLinearRing* RingTmp = (OGRLinearRing*)Ring->clone();
    std::vector<TVec2f> TexTmp;// = *Tex;

    TexTmp.insert(TexTmp.end(), Tex->begin(), Tex->end());

    Tex->clear();
    delete Ring;
    Ring = new OGRLinearRing;

    for (std::size_t i = 0; i < TexTmp.size(); ++i)
    {
        Tex->push_back(TexTmp.at(TexTmp.size() - 1 - i));
        Ring->addPoint(RingTmp->getX((int)TexTmp.size() - 1 - i), RingTmp->getY((int)TexTmp.size() - 1 - i), RingTmp->getZ((int)TexTmp.size() - 1 - i));
    }

    delete RingTmp;
}

/**
* @brief Decoupe un polygon 3D GMLPoly suivant un second polygon 2D BuildingShp. Il faut que le polygon decoupe soit encore en 3d.
* @param GMLPoly represente le premier polygon 3D (issu du fichier CityGML)
* @param BuildingShp represente le second polygon 2D qui va decouper GMLPoly
* @param TexUV represente les coordonnees de texture en entree correspondant au GMLPoly
* @param TexUVout represente les coordonnees de texture en sortie pour la Geometry resultat.
*/
OGRGeometry * CutPolyGMLwithShape(OGRPolygon* GMLPoly, OGRPolygon* BuildingShp, std::vector<TVec2f> *TexUV, std::vector<std::vector<TVec2f>>* TexUVout)
{
    //SaveGeometrytoShape("A_GMLPoly.shp", GMLPoly);
    //SaveGeometrytoShape("A_BuildingShp.shp", BuildingShp);

    OGRGeometry* Inter = GMLPoly->Intersection(BuildingShp);

    //On va parcourir chaque point P de Inter et calculer sa position dans GMLPoly afin de calculer sa coordonnee z
    //On commence par recuperer trois points A, B et C de GMLPoly non alignes pour obtenir l'equation parametrique du plan forme par ce polygon
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

    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe a 1 tant que C n'est pas correctement rempli
    for (int i = 1; i < GMLRing->getNumPoints() - 1; ++i) //Pas besoin de regarder le dernier point qui est une repetition du premier
    {
        if (test == 0)
        {
            B.x = GMLRing->getX(i);
            B.y = GMLRing->getY(i);
            B.z = GMLRing->getZ(i);

            if (A.x != B.x || A.y != B.y)
            {
                ++test;// A est bien different de B
                AB = B - A;
                uvB = TexUV->at(i);
                uvAB = uvB - uvA;
            }
        }
        else if (test == 1)
        {
            C.x = GMLRing->getX(i);
            C.y = GMLRing->getY(i);
            C.z = GMLRing->getZ(i);

            if ((C.x - A.x) / (B.x - A.x) != (C.y - A.y) / (B.y - A.y))
            {
                ++test;// C n'est pas aligne avec A et B => A B C forment bien un plan
                AC = C - A;
                uvC = TexUV->at(i);
                uvAC = uvC - uvA;
                break;
            }
        }
    }

    if (AB.x == 0) //Pour le calcul de s et t, cela pose probleme donc on intervertit B et C pour avoir un AB.x != 0. En effet, AB.x et AC.x ne peuvent tous deux etre egaux a 0 sinon le triangle serait plat.
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

    if (test != 2)
    {
        std::cout << "Erreur lors de la creation du plan. \n";
        delete Inter;
        return nullptr;
    }

    OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(Inter);
    if (PolyInter != nullptr)
    {
        //SaveGeometrytoShape("A_PolyInter.shp", PolyInter);
        if (PolyInter->get_Area() == GMLPoly->get_Area())
        {
            delete Inter;
            TexUVout->push_back(*TexUV);
            OGRPolygon* PolyTemp = new OGRPolygon(*GMLPoly);
            return PolyTemp;
            //return GMLPoly->clone(); //GMLPoly est inclu dans BuildingShp, il n'y a pas besoin de le modifier
        }
        OGRPolygon* ResPoly = new OGRPolygon;
        OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
        OGRLinearRing* ResExtRing = new OGRLinearRing;

        std::vector<TVec2f> uvPolyInter;

        for (int i = 0; i < InterExtRing->getNumPoints(); ++i)
        {
            OGRPoint* P = new OGRPoint;
            InterExtRing->getPoint(i, P);

            TVec3d M; // <=> P
            M.x = P->getX();
            M.y = P->getY();

            float s, t;

            t = (float)((A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y));
            s = (float)((M.x - A.x - t * AC.x) / AB.x);

            M.z = A.z + s * AB.z + t * AC.z;

            ResExtRing->addPoint(M.x, M.y, M.z);

            uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquees sans deformation.

            delete P;
        }

        ChangePointsOrderForNormal(ResExtRing, &uvPolyInter);

        ResPoly->addRingDirectly(ResExtRing);

        for (int r = 0; r < PolyInter->getNumInteriorRings(); ++r)
        {
            //std::cout << "INTERIOR RING" << std::endl;

            OGRLinearRing* InterIntRing = PolyInter->getInteriorRing(r);
            OGRLinearRing* ResIntRing = new OGRLinearRing;

            for (int i = 0; i < InterIntRing->getNumPoints(); ++i)
            {
                OGRPoint* P = new OGRPoint;
                InterIntRing->getPoint(i, P);

                TVec3d M;
                M.x = P->getX();
                M.y = P->getY();

                float s, t;

                t = (float)((A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y));
                s = (float)((M.x - A.x - t * AC.x) / AB.x);

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
    else //Si l'intersection ne represente pas un simple Polygon, il faut rechercher si c'est une GeometryCollection qui en contient, afin de pouvoir ensuite les recuperer.
    {
        //std::cout << "Pas Polygon !! " << Inter->getGeometryName() << std::endl;

        OGRGeometryCollection* GC_Inter = dynamic_cast<OGRGeometryCollection*>(Inter);
        if (GC_Inter != nullptr)
        {
            OGRMultiPolygon* ResMultiPoly = new OGRMultiPolygon;
            for (int j = 0; j < GC_Inter->getNumGeometries(); ++j)
            {
                OGRPolygon* PolyInter = dynamic_cast<OGRPolygon*>(GC_Inter->getGeometryRef(j));
                if (PolyInter != nullptr)
                {
                    OGRPolygon* ResPoly = new OGRPolygon;
                    OGRLinearRing* InterExtRing = PolyInter->getExteriorRing();
                    OGRLinearRing* ResExtRing = new OGRLinearRing;

                    std::vector<TVec2f> uvPolyInter;

                    for (int i = 0; i < InterExtRing->getNumPoints(); ++i)
                    {
                        OGRPoint* P = new OGRPoint;
                        InterExtRing->getPoint(i, P);

                        TVec3d M; // <=> P
                        M.x = P->getX();
                        M.y = P->getY();

                        float s, t;

                        t = (float)((A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y));
                        s = (float)((M.x - A.x - t * AC.x) / AB.x);

                        M.z = A.z + s * AB.z + t * AC.z;

                        ResExtRing->addPoint(M.x, M.y, M.z);

                        uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquees sans deformation.

                        delete P;
                    }

                    ChangePointsOrderForNormal(ResExtRing, &uvPolyInter);

                    ResPoly->addRingDirectly(ResExtRing);

                    for (int r = 0; r < PolyInter->getNumInteriorRings(); ++r)
                    {
                        //std::cout << "INTERIOR RING2" << std::endl;

                        OGRLinearRing* InterIntRing = PolyInter->getInteriorRing(r);
                        OGRLinearRing* ResIntRing = new OGRLinearRing;

                        for (int i = 0; i < InterIntRing->getNumPoints(); ++i)
                        {
                            OGRPoint* P = new OGRPoint;
                            InterIntRing->getPoint(i, P);

                            TVec3d M;
                            M.x = P->getX();
                            M.y = P->getY();

                            float s, t;

                            t = (float)((A.y * AB.x - A.x * AB.y + AB.y * M.x - AB.x * M.y) / (AB.y * AC.x - AB.x * AC.y));
                            s = (float)((M.x - A.x - t * AC.x) / AB.x);

                            M.z = A.z + s * AB.z + t * AC.z;

                            ResIntRing->addPoint(M.x, M.y, M.z);

                            uvPolyInter.push_back(TVec2f(uvA.x + s * uvAB.x + t * uvAC.x, uvA.y + s * uvAB.y + t * uvAC.y)); //On part du principe que les textures sont appliquees sans deformation.

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
* @brief Calcul les coordonnees de texture UV d'un point par rapport a un polygone qui lui est coplanaire.
* @param Poly Contient les coordonnees des points du polygone
* @param UVs Contient les coordonnees de texture des points du polygone
* @param Point Contient le point dont on desire calculer les coordonnees de texture
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

    int test = 0;//Vaut 0 tant que B n'est pas correctement rempli, puis passe a 1 tant que C n'est pas correctement rempli
    for (std::size_t i = 1; i < Poly->size(); ++i) //Le premier point n'est pas repete a la fin
    {
        if (test == 0)
        {
            B = Poly->at(i);

            if (A != B)
            {
                ++test;// A est bien different de B
                AB = B - A;
                uvB = UVs->at(i);
                uvAB = uvB - uvA;
            }
        }
        else if (test == 1)
        {
            C = Poly->at(i);

            if ((C.x - A.x) / (B.x - A.x) != (C.y - A.y) / (B.y - A.y) || (C.z - A.z) / (B.z - A.z) != (C.y - A.y) / (B.y - A.y) || (C.x - A.x) / (B.x - A.x) != (C.z - A.z) / (B.z - A.z))
            {
                ++test;// C n'est pas aligne avec A et B => A B C forment bien un plan
                AC = C - A;
                uvC = UVs->at(i);
                uvAC = uvC - uvA;
                break;
            }
        }
    }

    if (AB.x == 0 && AB.y == 0 && AC.x == 0 && AC.y == 0) // Pour les polygones de murs tenant sur un seul point (x,y) (ces polygones sont inutiles ?)
        return TVec2f(0, 0);

    float s, t;

    if (AB.x != 0)
    {
        t = (float)((A.z * AB.x - A.x * AB.z + AB.z * Point.x - AB.x * Point.z) / (AB.z * AC.x - AB.x * AC.z));
        s = (float)((Point.x - A.x - t * AC.x) / AB.x);
    }
    else if (AB.y != 0)
    {
        t = (float)((A.z * AB.y - A.y * AB.z + AB.z * Point.y - AB.y * Point.z) / (AB.z * AC.y - AB.y * AC.z));
        s = (float)((Point.y - A.y - t * AC.y) / AB.y);
    }
    else if (AB.z != 0)
    {
        t = (float)((A.x * AB.z - A.z * AB.x + AB.x * Point.z - AB.z * Point.x) / (AB.x * AC.z - AB.z * AC.x));
        s = (float)((Point.z - A.z - t * AC.z) / AB.z);
    }
    else
    {
        t = (float)((A.y * AB.z - A.z * AB.y + AB.y * Point.z - AB.z * Point.y) / (AB.y * AC.z - AB.z * AC.y));
        s = (float)((Point.z - A.z - t * AC.z) / AB.z);
    }

    if (AB.z == 0 && AC.z == 0)//Pour les batiments remarquables qui ne sont pas en LOD2 et qui ont des murs horizontaux, pour eviter les -1.#IND
    {
        t = (float)((A.y * AB.x - A.x * AB.y + AB.y * Point.x - AB.x * Point.y) / (AB.y * AC.x - AB.x * AC.y));
        s = (float)((Point.x - A.x - t * AC.x) / AB.x);
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
* @brief GetLineStringsFromPolygon lit un ou plusieurs polygons et extraits les aretes (deux points) regroupes par Ring
* @param Polygon
* @return Retourne un OGRMultiLineString par Ring : un LineString est compose de seulement deux points
*/
std::vector<OGRMultiLineString*> GetLineStringsFromPolygon(OGRPolygon* Polygon)
{
    std::vector<OGRMultiLineString*> ListRing;

    OGRLinearRing* Ring = Polygon->getExteriorRing();
    OGRMultiLineString* MultiLS = new OGRMultiLineString;

    for (int j = 0; j < Ring->getNumPoints() - 1; ++j)
    {
        OGRLineString* LS = new OGRLineString();
        LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
        LS->addPoint(Ring->getX(j + 1), Ring->getY(j + 1), Ring->getZ(j + 1));
        MultiLS->addGeometryDirectly(LS);
    }
    ListRing.push_back(MultiLS);

    for (int k = 0; k < Polygon->getNumInteriorRings(); ++k)
    {
        Ring = Polygon->getInteriorRing(k);
        delete MultiLS;
        MultiLS = new OGRMultiLineString;

        for (int j = 0; j < Ring->getNumPoints() - 1; ++j)
        {
            OGRLineString* LS = new OGRLineString();
            LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
            LS->addPoint(Ring->getX(j + 1), Ring->getY(j + 1), Ring->getZ(j + 1));
            MultiLS->addGeometryDirectly(LS);
        }
        ListRing.push_back(MultiLS);
    }

    return ListRing;
}
std::vector<OGRMultiLineString*> GetLineStringsFromMultiPolygon(OGRGeometryCollection* MultiPolygon)
{
    std::vector<OGRMultiLineString*> ListRing;

    for (int i = 0; i < MultiPolygon->getNumGeometries(); ++i)
    {
        OGRPolygon* Polygon = dynamic_cast<OGRPolygon*>(MultiPolygon->getGeometryRef(i));
        if (Polygon == nullptr)
            continue;

        OGRLinearRing* Ring = Polygon->getExteriorRing();
        OGRMultiLineString* MultiLS = new OGRMultiLineString;

        for (int j = 0; j < Ring->getNumPoints() - 1; ++j)
        {
            OGRLineString* LS = new OGRLineString();
            LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
            LS->addPoint(Ring->getX(j + 1), Ring->getY(j + 1), Ring->getZ(j + 1));
            MultiLS->addGeometryDirectly(LS);
        }
        ListRing.push_back(MultiLS);

        for (int k = 0; k < Polygon->getNumInteriorRings(); ++k)
        {
            Ring = Polygon->getInteriorRing(k);
            delete MultiLS;
            MultiLS = new OGRMultiLineString;

            for (int j = 0; j < Ring->getNumPoints() - 1; ++j)
            {
                OGRLineString* LS = new OGRLineString();
                LS->addPoint(Ring->getX(j), Ring->getY(j), Ring->getZ(j));
                LS->addPoint(Ring->getX(j + 1), Ring->getY(j + 1), Ring->getZ(j + 1));
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
    if (Polygon == nullptr)
        return  ListPoints;

    OGRLinearRing* ExtRing = Polygon->getExteriorRing();

    OGRMultiPoint* PointsExtRing = new OGRMultiPoint;

    for (int i = 0; i < ExtRing->getNumPoints(); ++i)
    {
        OGRPoint* P = new OGRPoint;
        ExtRing->getPoint(i, P);
        PointsExtRing->addGeometryDirectly(P);
    }
    ListPoints.push_back(PointsExtRing);

    for (int j = 0; j < Polygon->getNumInteriorRings(); ++j)
    {
        OGRMultiPoint* PointsIntRing = new OGRMultiPoint;

        OGRLinearRing* IntRing = Polygon->getInteriorRing(j);
        for (int i = 0; i < IntRing->getNumPoints(); ++i)
        {
            OGRPoint* P = new OGRPoint;
            ExtRing->getPoint(i, P);
            PointsIntRing->addGeometryDirectly(P);
        }
        ListPoints.push_back(PointsIntRing);
    }

    return ListPoints;
}
