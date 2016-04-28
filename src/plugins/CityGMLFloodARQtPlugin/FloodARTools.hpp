#ifndef _FLOODAR_TOOLS_
#define _FLOODAR_TOOLS_

#include "export/exportCityGML.hpp"
#include <QFileDialog>

namespace FloodAR
{
  std::vector<TextureCityGML*> getTexturesList(citygml::CityModel*, QFileInfo, QFileInfo);
  void cutASC(std::string filePath, std::string workingDir, int tileSizeX, int tileSizeY, bool isTerrain);
  void cutPicture(std::string filename, std::string workingDir, int width, int height);
  void ASCtoWaterAuto(std::string workingDir, float prec, std::string startingDate);
  void ASCtoTerrain(std::string filePath1, bool fusion, std::string filePath2, bool addTextures, std::string texturesPath);
}

#endif
