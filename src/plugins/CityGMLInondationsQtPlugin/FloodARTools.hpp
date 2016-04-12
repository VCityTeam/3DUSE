#ifndef _FLOODAR_TOOLS_
#define _FLOODAR_TOOLS_

#include "export/exportCityGML.hpp"
#include <QFileDialog>

namespace FloodAR
{
	std::vector<TextureCityGML*> getTexturesList(citygml::CityModel*, QFileInfo, QFileInfo);
	void cutASC(std::string filePath, std::string outputDir, int tileSizeX, int tileSizeY);
	void cutPicture(std::string filename, int width, int height);
	void ASCtoWater(std::string filePath, bool polygonsImport, float prec, bool tempImport, std::string creaDate, std::string termDate);
	void ASCtoTerrain(std::string filePath1, bool fusion, std::string filePath2, bool addTextures, std::string texturesPath);
}

#endif
