#include "FlatRoof.hpp"

#include "gui/osg/osgScene.hpp"
#include "src/processes/ExportToShape.hpp"

#include <QDir>

void DetectionToitsPlats(std::string path, float minArea, float slopeFactor)
{
	vcity::Tile* tile = new vcity::Tile(path);
	citygml::CityModel * model = tile->getCityModel();

	OGRMultiPolygon* toitsPlats = new OGRMultiPolygon;//Carte contenant uniquement les toits plats
	OGRMultiPolygon* toits = new OGRMultiPolygon;//Carte de tous les toits.

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
							const std::vector<TVec3f>& norms = PolygonCityGML->getNormals();

							OGRPolygon * OgrPoly = new OGRPolygon;
							OGRLinearRing * OgrRing = new OGRLinearRing;

							for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
							{
								OgrRing->addPoint(Point.x, Point.y, Point.z);
							}

							OgrRing->closeRings();

							if(OgrRing->getNumPoints() > 3)//Vérification qu'on a bien un triangle
							{
								OgrPoly->addRingDirectly(OgrRing);
								if(OgrPoly->IsValid())//Vérification que notre triangle n'est pas dégénéré
								{
									toits->addGeometryDirectly(OgrPoly);
									TVec3f norm = norms.at(0);
									if(norm.z >= slopeFactor)//On test la normal à notre triangle
									{
										toitsPlats->addGeometry(OgrPoly);
									}
								}
							}
						}
					}
				}
			}
		}
	}
	OGRGeometry* toitsGobalUnion = toits->UnionCascaded();
	OGRGeometry* Union = toitsPlats->UnionCascaded();
	OGRGeometryCollection* UnionMP = (OGRGeometryCollection*)Union;
	OGRMultiPolygon* ToitsRes = new OGRMultiPolygon;
	for(int i = 0; i < UnionMP->getNumGeometries(); ++i)
	{
		OGRPolygon* poly = dynamic_cast<OGRPolygon*>(UnionMP->getGeometryRef(i));
		if(poly == nullptr)
			continue;
		if(poly->get_Area() > minArea)
			ToitsRes->addGeometry(poly);
	}

	QDir outputDir("./SkylineOutput/");
	if(!outputDir.exists("./SkylineOutput/"))
	{
		outputDir.mkpath(outputDir.absolutePath());
	}

	SaveGeometrytoShape("SkylineOutput/Toits.shp", toits);
	SaveGeometrytoShape("SkylineOutput/ToitsUnion.shp", toitsGobalUnion);
	SaveGeometrytoShape("SkylineOutput/ToitsPlats.shp", toitsPlats);
	SaveGeometrytoShape("SkylineOutput/ToitsPlatsUnion.shp", Union);
	SaveGeometrytoShape("SkylineOutput/ToitsPlatsUnionFiltre.shp", ToitsRes);

	delete UnionMP;
	delete ToitsRes;
	delete toits;
	delete toitsPlats;
	delete tile;
}
