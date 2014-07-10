#include "BatimentShape.hpp"

BatimentShape::BatimentShape()
{
	this->Geometry = NULL;
	this->ID = "";
}

BatimentShape::BatimentShape(geos::geom::Geometry * Geo, std::string ID)
{
	this->Geometry = Geo;
	this->ID = ID;
}