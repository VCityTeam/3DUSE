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

///
/// \brief createOutputFolders Prepare folder layout hierarchy:
///
///                             sOutputDir
///                                 |
///                           SunlightOutput
///                                 |
///                      ----------------------
///                      |                    |
///                    _BATI                _MNT
///                      |                    |
///                -------------           -------------
///                |            |           |          |
///        3670_10383  ... XXXX_XXXX   3670_10383  ... XXXX_XXXX
///            |
///     -------------------------
///     |                       |
///2016-01-01.csv     ...   2016-31-12.csv
///
///
/// This function creates SunlightOutput and _BATI, _MNT folders.
///
/// \param sOutputDir Path to output directory
///
void createOutputFolders(const QString &sOutputDir);

///
/// \brief createFileFolder Creates folder for a given file which will hold its computed sunlight infos
/// \param file File to create folder for
/// \param sOutputDir Full path to output directory
///
void createFileFolder(FileInfo* file, const QString &sOutputDir);

///
/// \brief exportLightningToCSV Export Sunlight results of a given triangle into a csv.
/// \param sunInfo map containing sunlight information (int : datetime, bool : sunny)
/// \param t Triangle to export sunlight for
/// \param file File which holds the triangle
/// \param iStartDate Start date of sunlight computation encoded as int
/// \param iEndDate End date of sunlight computation encoded as int
/// \param outputDir Full path to output directory
///
void exportLightningToCSV(std::map<int, bool> &sunInfo, Triangle* t, FileInfo *file, int iStartDate, int iEndDate, QString &outputDir);

///
/// \brief loadSunpathFile Loads file containing the path of the sun for a year
/// \param sunpathFile Full path to the file
/// \param iStartDate start date of sunlight computation encoded as int
/// \param iEndDate end date of sunlight computation encoded as int
/// \return
///
std::map<int,TVec3d> loadSunpathFile(std::string sunpathFile, int iStartDate, int iEndDate);

#endif // IO_H
