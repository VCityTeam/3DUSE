#ifndef SUNLIGHTDETECTION_HPP
#define SUNLIGHTDETECTION_HPP

#include "vecs.hpp"

#include <QString>

class FileInfo;

void SunlightDetection(std::string fileDir, std::vector<FileInfo*> filenames, std::string sunpathFile, std::string startDate, std::string endDate, QString outputDir);

#endif // SUNLIGHTDETECTION_HPP
