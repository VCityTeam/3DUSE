#ifndef _FLOODAR_TOOLS_
#define _FLOODAR_TOOLS_

#include "export/exportCityGML.hpp"
#include <QFileDialog>

namespace FloodAR
{
  std::vector<TextureCityGML*> getTexturesList(citygml::CityModel*, QFileInfo, QFileInfo);
  void cutASC(std::string filePath, std::string workingDir, int tileSizeX, int tileSizeY, bool isTerrain);
  void cutPicture(std::string filename, std::string workingDir, int width, int height);
  void CutShapeFile(std::string workingDir, int tilesize_x, int tilesize_y, std::string filename);
  void ASCtoWaterAuto(std::string workingDir, float prec, std::string startingDate);
  void ASCtoTerrain(std::string workingDir);
  void ShapeExtrusion(std::string workingDir);
}

#endif
