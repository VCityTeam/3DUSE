#ifndef FILEINFO_H
#define FILEINFO_H

#include <string>

enum fileType
{
    _BATI,
    _MNT,
    _DEFAULT
};

class FileInfo
{

public:
    FileInfo(std::string filepath);

    std::string WithGMLExtension();
    std::string WithPrevFolder();
    std::string WithPrevFolderAndGMLExtension();

    std::string m_filename;
    std::string m_filepath;
    fileType m_type;

};

#endif // FILEINFO_H
