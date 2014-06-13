#include "BatimentShape.hpp"

BatimentShape::BatimentShape()
{
	this->Geometry = NULL;
	this->ID = "";
	this->Name = "";
}

BatimentShape::BatimentShape(geos::geom::Geometry * Geo, std::string ID, std::string Name)
{
	this->Geometry = Geo;
	this->ID = ID;
	this->Name = Name;
}