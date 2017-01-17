#include "FlatRoof.hpp"

#include "gui/osg/osgScene.hpp"
#include "src/utils/OGRGDAL_Utils/OGRGDALtoShpWriter.hpp"

#include <QDir>

void DetectionToitsPlats(std::string path, float minArea, float slopeFactor)
{
	vcity::Tile* tile = new vcity::Tile(path);
	citygml::CityModel * model = tile->getCityModel();

  //Carte contenant uniquement les toits plats
	OGRMultiPolygon* toitsPlats = new OGRMultiPolygon;
  //Carte de tous les toits.
	OGRMultiPolygon* toits = new OGRMultiPolygon;

	for(citygml::CityObject* obj : model->getCityObjectsRoots())
	{
		if(obj->getType() == citygml::COT_Building)
		{
      //On parcourt les objets (Wall, Roof, ...) du batiment
			for(citygml::CityObject* object : obj->getChildren())
			{
				if(object->getType() == citygml::COT_RoofSurface)
				{
          // Pour chaque geometrie
					for(citygml::Geometry* Geometry : object->getGeometries())
					{
            // Pour chaque polygone
						for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
						{
							const std::vector<TVec3f>& norms = PolygonCityGML->getNormals();

							OGRPolygon * OgrPoly = new OGRPolygon;
							OGRLinearRing * OgrRing = new OGRLinearRing;

							for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
							{
								OgrRing->addPoint(Point.x, Point.y, Point.z);
							}

							OgrRing->closeRings();

              //Verification qu'on a bien un triangle
							if(OgrRing->getNumPoints() > 3)
							{
								OgrPoly->addRingDirectly(OgrRing);
                // Verification que notre triangle n'est pas degenere
								if(OgrPoly->IsValid())
								{
									toits->addGeometryDirectly(OgrPoly);
									TVec3f norm = norms.at(0);
                  // On test la normal a notre triangle
									if(norm.z >= slopeFactor)
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
