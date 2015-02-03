#ifndef CITYGMLFUSION_H
#define CITYGMLFUSION_H

#include "citygml.hpp"

void FusionTiles();
void FusionLODs(citygml::CityModel * City1, citygml::CityModel * City2);

#endif // CITYGMLFUSION_H
