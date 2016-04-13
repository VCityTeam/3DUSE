#ifndef IO_H
#define IO_H

#include <string>
#include <map>
#include <vector>
#include <QString>

#include "vecs.hpp"
#include "FileInfo.h"

// Forward declaration, class can be found in libcitygml utils library
struct Triangle;


void createOutputFolders(QString sOutputDir);

///
/// \brief initExportFile
/// \param filename
///
void createFileFolder(FileInfo* file, QString sOutputDir);

///
/// \brief exportLightningToCSV Export Sunlight informations for a given tile into a csv file.
/// \param vSunInfo vector holding informations about sunlight for a given triangle.
/// \param filename Name of the file.
///
void exportLightningToCSV(std::map<int,bool> sunInfo, Triangle* t, FileInfo *file, int iStartDate, int iEndDate, QString outputDir);


std::map<int,TVec3d> loadSunpathFile(std::string sunpathFile, int iStartDate, int iEndDate);

#endif // IO_H
