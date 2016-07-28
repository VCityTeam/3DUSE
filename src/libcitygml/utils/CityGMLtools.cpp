#include "CityGMLtools.hpp"
#include "utils/OGRGDAL_Utils/OGRGDALtools.hpp"

/**
* @brief Projette les toits du CityObject selectionne sur le plan (xy)
* @param obj CityObject selectionne
* @param FootPrint un multiPolygon, le resultat de la projection
* @param heightmax Enregistre le Zmax des murs du batiment
* @param heightmin Enregistre le Zmin des murs du batiment
*/
void GetFootprint(citygml::CityObject* obj, OGRMultiPolygon * FootPrint, double * heightmax, double * heightmin)
{
    if (obj->getType() == citygml::COT_RoofSurface) //Si surface de toit : COT_RoofSurface COT_WallSurface
    {
        for (citygml::Geometry* Geom : obj->getGeometries())
        {
            for (citygml::Polygon* Poly : Geom->getPolygons())
            {
                OGRPolygon * OgrPoly = new OGRPolygon;
                OGRLinearRing * OgrRing = new OGRLinearRing;
                for (TVec3d Vertices : Poly->getExteriorRing()->getVertices())
                {
                    OgrRing->addPoint(Vertices.x, Vertices.y);

                    if (Vertices.z > *heightmax)
                        *heightmax = Vertices.z;
                    //std::cout << " (x,y) = (" << Vertices.x<< "," << Vertices.y<< ")" << std::endl;
                }
                OgrRing->closeRings();
                if (OgrRing->getNumPoints() > 3)//Le polygone ne sera cree qu'a partir de 4 points
                {
                    OgrPoly->addRingDirectly(OgrRing);
                    if (OgrPoly->IsValid())
                        FootPrint->addGeometryDirectly(OgrPoly); // on recupere le polygone
                }
            }
        }
    }
    else if (obj->getType() == citygml::COT_WallSurface)//Remplissage de la hauteur min des murs (correspondant au "sol" du batiment)
    {
        for (citygml::Geometry* Geom : obj->getGeometries())
        {
            for (citygml::Polygon* Poly : Geom->getPolygons())
            {
                for (TVec3d Vertices : Poly->getExteriorRing()->getVertices())
                {
                    if (Vertices.z < *heightmin || *heightmin == -1)
                        *heightmin = Vertices.z;
                }
            }
        }
    }
    for (citygml::CityObject* Obj : obj->getChildren())//Pour descendre dans le batiment jusqu'a arriver aux Wall/Roofs
    {
        GetFootprint(Obj, FootPrint, heightmax, heightmin);
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Convertit un LOD1 contenu dans un MultiPolygon en Cityobject CityGML.
* @param name Nom du Cityobject CityGML creee
* @param Geometry MultiPolygon contient les emprises au sol a convertir
* @param heightmin Donne la valeur de z minimale pour le LOD1
* @param heightmax Donne la valeur de z maximale pour le LOD1
*/
citygml::CityObject* ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin)
{
    citygml::CityObject* BuildingCO = new citygml::Building("LOD1_" + name);
    citygml::CityObject* WallCO = new citygml::WallSurface("LOD1_" + name + "_Wall");
    citygml::CityObject* RoofCO = new citygml::RoofSurface("LOD1_" + name + "_Roof");
    //citygml::CityObject* GroundCO = new citygml::GroundSurface("LOD1_" + name + "_Ground");

    for (int i = 0; i < Enveloppe->getNumGeometries(); ++i)
    {
        const OGRPolygon * Poly = dynamic_cast<const OGRPolygon *>(Enveloppe->getGeometryRef(i));
        if (!Poly || !Poly->IsValid())
            continue;

        citygml::Geometry* Wall = new citygml::Geometry(name + "_Wall_" + std::to_string(i), citygml::GT_Wall, 1);
        citygml::Geometry* Roof = new citygml::Geometry(name + "_Roof_" + std::to_string(i), citygml::GT_Roof, 1);

        citygml::Polygon * PolyRoof = new citygml::Polygon(name + "_PolyRoof_" + std::to_string(i));
        citygml::LinearRing * RingRoof = new citygml::LinearRing(name + "_RingRoof_" + std::to_string(i), true);

        const OGRLinearRing * ExtRing = Poly->getExteriorRing();

        for (int j = 0; j < ExtRing->getNumPoints() - 1; ++j)//On s'arrete a size - 1 car le premier point est deja repete en derniere position
        {
            citygml::Polygon * PolyWall = new citygml::Polygon(name + "_PolyWall_" + std::to_string(i) + "_" + std::to_string(j));
            citygml::LinearRing * RingWall = new citygml::LinearRing(name + "_RingWall_" + std::to_string(i) + "_" + std::to_string(j), true);

            OGRPoint * point = new OGRPoint;
            ExtRing->getPoint(j, point);
            double x1 = point->getX();
            double y1 = point->getY();
            delete point;

            RingRoof->addVertex(TVec3d(x1, y1, *heightmax));

            point = new OGRPoint;
            ExtRing->getPoint(j + 1, point);
            double x2 = point->getX();
            double y2 = point->getY();
            delete point;

            RingWall->addVertex(TVec3d(x1, y1, *heightmin));
            RingWall->addVertex(TVec3d(x2, y2, *heightmin));
            RingWall->addVertex(TVec3d(x2, y2, *heightmax));
            RingWall->addVertex(TVec3d(x1, y1, *heightmax));

            PolyWall->addRing(RingWall);
            Wall->addPolygon(PolyWall);
        }
        PolyRoof->addRing(RingRoof);
        Roof->addPolygon(PolyRoof);

        WallCO->addGeometry(Wall);
        RoofCO->addGeometry(Roof);

        //std::cout << "Avancement creation LOD1 : " << i+1 << "/" << Geometry->getNumGeometries() << "\r" << std::flush;
    }

    BuildingCO->insertNode(WallCO);
    BuildingCO->insertNode(RoofCO);

    return BuildingCO;
}

/**
* @brief Convertit un LOD0 contenu dans un MultiPolygon en geometry CityGML.
* @param name Nom de la geometry CityGML creee
* @param Geometry MultiPolygon contient les emprises au sol a convertir en objet CityGML
* @param heightmin Permet de situer les LOD0 dans l'espace
*/
citygml::Geometry* ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin)
{
    citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
    for (int i = 0; i < Geometry->getNumGeometries(); ++i)
    {
        citygml::Polygon * Poly = new citygml::Polygon("Polygon");
        citygml::LinearRing * Ring = new citygml::LinearRing("ExteriorRing", true);

        if (Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon && Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon25D)
            continue;

        OGRPolygon * Polygon = dynamic_cast<OGRPolygon*>(Geometry->getGeometryRef(i)->clone());

        OGRLinearRing * ExtRing = Polygon->getExteriorRing();

        for (int j = 0; j < ExtRing->getNumPoints(); ++j)
        {
            OGRPoint * point = new OGRPoint;
            ExtRing->getPoint(j, point);
            Ring->addVertex(TVec3d(point->getX(), point->getY(), *heightmin));
        }
        Poly->addRing(Ring);
        for (int k = 0; k < Polygon->getNumInteriorRings(); ++k)
        {
            citygml::LinearRing * IntRingCityGML = new citygml::LinearRing("InteriorRing", false);//False pour signifier que le linearring correspond a un interior ring
            OGRLinearRing * IntRing = Polygon->getInteriorRing(k);

            for (int j = 0; j < IntRing->getNumPoints(); ++j)
            {
                OGRPoint * point = new OGRPoint;
                IntRing->getPoint(j, point);
                IntRingCityGML->addVertex(TVec3d(point->getX(), point->getY(), *heightmin));
            }
            Poly->addRing(IntRingCityGML);
        }
        Geom->addPolygon(Poly);

        delete Polygon;
    }
    return Geom;
}

/**
* @brief Genere le LOD0 du batiment contenu dans obj
* @param obj batiment courant
* @param Enveloppe resultat de la fonction : un multipolygon correspondant au footprint de obj
* @param heightmax Hauteur max des murs du batiment
* @param heightmin Hauteur min des murs du batiment
*/
void generateLOD0fromLOD2(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin)
{
    *heightmax = 0;
    *heightmin = -1;

    OGRMultiPolygon * Footprint = new OGRMultiPolygon;
    GetFootprint(obj, Footprint, heightmax, heightmin);

    if (!Footprint->IsEmpty())
        *Enveloppe = GetEnveloppe(Footprint);
}

/**
* @brief Convertit un OGRPolygon* en citygml::Polygon*
* @param OGRPoly OGRPolygon* a convertir.
* @param Name Nom du Polygon CityGML a retourner.
*/
citygml::Polygon * ConvertOGRPolytoGMLPoly(OGRPolygon* OGRPoly, std::string Name)
{
    OGRLinearRing * ExtRing = OGRPoly->getExteriorRing();

    if (ExtRing == nullptr)
        return nullptr;

    citygml::Polygon * Poly = new citygml::Polygon(Name + "_Poly");
    citygml::LinearRing * Ring = new citygml::LinearRing(Name + "_Ring", true);

    for (int j = 0; j < ExtRing->getNumPoints() - 1; ++j)//On s'arrete a size - 1 car le premier point est deja repete en derniere position
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

    for (int i = 0; i < OGRPoly->getNumInteriorRings(); ++i)
    {
        citygml::LinearRing * IntRingGML = new citygml::LinearRing(Name + "_IntRing_" + std::to_string(i), false);

        OGRLinearRing * IntRing = OGRPoly->getInteriorRing(i);

        for (int j = 0; j < IntRing->getNumPoints() - 1; ++j)//On s'arrete a size - 1 car le premier point est deja repete en derniere position
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
////////////////////////////////////////////////////////////////////////////////
