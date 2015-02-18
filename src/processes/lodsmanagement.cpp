#include "lodsmanagement.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////

/**
* @brief Projette les toits du CityObject sélectionné sur le plan (xy)
* @param obj CityObject sélectionné
* @param FootPrint un multiPolygon, le résultat de la projection
* @param heightmax Enregistre le Zmax des murs du bâtiment
* @param heightmin Enregistre le Zmin des murs du bâtiment
*/
void GetFootprint(citygml::CityObject* obj, OGRMultiPolygon * FootPrint, double * heightmax, double * heightmin)
{
    if(obj->getType() == citygml::COT_RoofSurface) //Si surface de toit : COT_RoofSurface COT_WallSurface
    {
        for(citygml::Geometry* Geom : obj->getGeometries())
        {
            for(citygml::Polygon* Poly : Geom->getPolygons())
            {
                OGRPolygon * OgrPoly = new OGRPolygon;
                OGRLinearRing * OgrRing = new OGRLinearRing;
                for(TVec3d Vertices : Poly->getExteriorRing()->getVertices())
                {
                    OgrRing->addPoint(Vertices.x, Vertices.y);

                    if(Vertices.z > *heightmax)
                        *heightmax = Vertices.z;
                    //std::cout << " (x,y) = (" << Vertices.x<< "," << Vertices.y<< ")" << std::endl;
                }
                OgrRing->closeRings();
                if(OgrRing->getNumPoints() > 3)//Le polygone ne sera créé qu'à partir de 4 points
                {
                    OgrPoly->addRingDirectly(OgrRing);
                    if(OgrPoly->IsValid())
                        FootPrint->addGeometryDirectly(OgrPoly); // on récupere le polygone
                }
            }
        }
    }
    else if(obj->getType() == citygml::COT_WallSurface)//Remplissage de la hauteur min des murs (correspondant au "sol" du bâtiment)
    {
        for(citygml::Geometry* Geom : obj->getGeometries())
        {
            for(citygml::Polygon* Poly : Geom->getPolygons())
            {
                for(TVec3d Vertices : Poly->getExteriorRing()->getVertices())
                {
                    if(Vertices.z < *heightmin || *heightmin == -1)
                        *heightmin = Vertices.z;
                }
            }
        }
    }
    for(citygml::CityObject* Obj : obj->getChildren())//Pour descendre dans le bâtiment jusqu'à arriver aux Wall/Roofs
    {
        GetFootprint(Obj, FootPrint, heightmax, heightmin);
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
* @brief Récupère l'enveloppe d'une geometry en faisant une succession d'unions sur tous les polygones
* @param MP Ensemble de polygones sur lequel on va générer une enveloppe
*/
OGRMultiPolygon * GetEnveloppe(OGRMultiPolygon * MP)
{
    //std::cout << "Mise en place de l'union des polygons" << std::endl;

    OGRGeometry* ResUnion = new OGRMultiPolygon;

    ResUnion = MP->UnionCascaded();

    //On travaille avec des OGRMultiPolygon pour avoir un format universel, il faut donc transformer la geometry en collection.
    if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbMultiPolygon25D)//La geometry est en fait un ensemble de geometry : plusieurs bâitments
    {
        OGRMultiPolygon * GeoCollection = (OGRMultiPolygon*)(ResUnion);

		//return GeoCollection;		//////////// Ignore le retrait des interior ring plats

        OGRMultiPolygon * MultiPolygonRes = new OGRMultiPolygon;

        for(int i = 0; i < GeoCollection->getNumGeometries(); ++i)//Pour enlever les arêtes parasites formant des interior ring "plats"
        {
            OGRGeometry * Geometry = GeoCollection->getGeometryRef(i);

            if(Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon || Geometry->getGeometryType() == OGRwkbGeometryType::wkbPolygon25D)
            {
                OGRPolygon * Poly = dynamic_cast<OGRPolygon*>(Geometry);
                OGRPolygon * PolyRes = new OGRPolygon;

                PolyRes->addRing(Poly->getExteriorRing());

                for(int j = 0; j < Poly->getNumInteriorRings(); ++j)
                {
                    const OGRLinearRing * IntRing = Poly->getInteriorRing(j);

                    if(IntRing->get_Area() > 0.1) //Pour enlever les arêtes parasites formant des interior ring "plats"
                        PolyRes->addRingDirectly(dynamic_cast<OGRLinearRing*>(IntRing->clone()));
                }

                MultiPolygonRes->addGeometryDirectly(PolyRes);
            }
        }

        return MultiPolygonRes;
    }
    else if(ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon || ResUnion->getGeometryType() == OGRwkbGeometryType::wkbPolygon25D)//La geometry est en fait un seul polygon : un seul bâtiment
    {
        OGRMultiPolygon * GeoCollection = new OGRMultiPolygon;
        GeoCollection->addGeometryDirectly(ResUnion);
        return GeoCollection;
    }

    return nullptr;
}

/**
* @brief Convertit un LOD1 contenu dans un MultiPolygon en Cityobject CityGML.
* @param name Nom du Cityobject CityGML créée
* @param Geometry MultiPolygon contient les emprises au sol à convertir
* @param heightmin Donne la valeur de z minimale pour le LOD1
* @param heightmax Donne la valeur de z maximale pour le LOD1
*/
citygml::CityObject* ConvertLOD1ToCityGML(std::string name, OGRMultiPolygon * Enveloppe, double * heightmax, double * heightmin)
{
    citygml::CityObject* BuildingCO = new citygml::Building("LOD1_" + name);
    citygml::CityObject* WallCO = new citygml::WallSurface("LOD1_" + name + "_Wall");
    citygml::CityObject* RoofCO = new citygml::RoofSurface("LOD1_" + name + "_Roof");
    //citygml::CityObject* GroundCO = new citygml::GroundSurface("LOD1_" + name + "_Ground");

    for(int i = 0; i < Enveloppe->getNumGeometries(); ++i)
    {
        const OGRPolygon * Poly = dynamic_cast<const OGRPolygon *>(Enveloppe->getGeometryRef(i));
        if(!Poly || !Poly->IsValid())
            continue;

        citygml::Geometry* Wall = new citygml::Geometry(name + "_Wall_" + std::to_string(i), citygml::GT_Wall, 1);
        citygml::Geometry* Roof = new citygml::Geometry(name + "_Roof_" + std::to_string(i), citygml::GT_Roof, 1);

        citygml::Polygon * PolyRoof = new citygml::Polygon(name + "_PolyRoof_" + std::to_string(i));
        citygml::LinearRing * RingRoof = new citygml::LinearRing(name + "_RingRoof_" + std::to_string(i), true);

        const OGRLinearRing * ExtRing = Poly->getExteriorRing();

        for(int j = 0; j < ExtRing->getNumPoints() - 1; ++j)//On s'arrête à size - 1 car le premier point est déjà répété en dernière position
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
            ExtRing->getPoint(j+1, point);
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
* @param name Nom de la geometry CityGML créée
* @param Geometry MultiPolygon contient les emprises au sol à convertir en objet CityGML
* @param heightmin Permet de situer les LOD0 dans l'espace
*/
citygml::Geometry* ConvertLOD0ToCityGML(std::string name, OGRMultiPolygon * Geometry, double * heightmin)
{
    citygml::Geometry* Geom = new citygml::Geometry(name + "_lod0", citygml::GT_Ground, 0);
    for(int i = 0; i < Geometry->getNumGeometries(); ++i)
    {
        citygml::Polygon * Poly = new citygml::Polygon("Polygon");
        citygml::LinearRing * Ring = new citygml::LinearRing("ExteriorRing", true);

        if(Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon && Geometry->getGeometryRef(i)->getGeometryType() != OGRwkbGeometryType::wkbPolygon25D)
            continue;

        OGRPolygon * Polygon =  dynamic_cast<OGRPolygon*>(Geometry->getGeometryRef(i)->clone());

        OGRLinearRing * ExtRing = Polygon->getExteriorRing();

        for(int j = 0; j < ExtRing->getNumPoints(); ++j)
        {
            OGRPoint * point = new OGRPoint;
            ExtRing->getPoint(j, point);
            Ring->addVertex(TVec3d(point->getX(), point->getY(), *heightmin));
        }
        Poly->addRing(Ring);
        for(int k = 0; k < Polygon->getNumInteriorRings(); ++k)
        {
            citygml::LinearRing * IntRingCityGML = new citygml::LinearRing("InteriorRing", false);//False pour signifier que le linearring correspond à un interior ring
            OGRLinearRing * IntRing = Polygon->getInteriorRing(k);

            for(int j = 0; j < IntRing->getNumPoints(); ++j)
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
* @brief Génère le LOD0 du bâtiment contenu dans obj
* @param obj bâtiment courant
* @param Enveloppe résultat de la fonction : un multipolygon correspondant au footprint de obj
* @param heightmax Hauteur max des murs du bâtiment
* @param heightmin Hauteur min des murs du bâtiment
*/
void generateLOD0fromLOD2(citygml::CityObject* obj, OGRMultiPolygon ** Enveloppe, double * heightmax, double * heightmin)
{
    *heightmax = 0;
    *heightmin = -1;

    OGRMultiPolygon * Footprint = new OGRMultiPolygon;
    GetFootprint(obj, Footprint, heightmax, heightmin);

    *Enveloppe = GetEnveloppe(Footprint);
}
////////////////////////////////////////////////////////////////////////////////
