#ifndef SUNLIGHTDETECTION_HPP
#define SUNLIGHTDETECTION_HPP

#include "vecs.hpp"

#include <QString>

class FileInfo;

///
/// \brief SunlightDetection Function to compute sunlight on a given set of file on a predefined period.
/// \param fileDir Full path of directory containing the files to compute sunlight for.
/// \param filenames List of files to compute sunlight for.
/// \param sunpathFile full path to file describing sun path for a given year.
/// \param startDate start date of sunlight computation
/// \param endDate end date of sunlight computation
/// \param outputDir full path to output directory
///
void SunlightDetection(std::string fileDir, std::vector<FileInfo*> filenames, std::string sunpathFile, std::string startDate, std::string endDate, QString outputDir);

#endif // SUNLIGHTDETECTION_HPP
