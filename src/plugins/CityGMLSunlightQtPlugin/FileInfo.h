#ifndef FILEINFO_H
#define FILEINFO_H

#include <string>

enum fileType
{
    _BATI,
    _MNT,
    _DEFAULT
};


///
/// \brief The FileInfo class is an utilitary to manipulate files in the plugin
///
class FileInfo
{

public:
    ///
    /// \brief FileInfo Constructor of the class.
    /// \param filepath Full path of the file
    ///
    FileInfo(std::string filepath);

    ///
    /// \brief WithGMLExtension
    /// \return name of the file with .gml extension
    ///
    std::string WithGMLExtension();

    ///
    /// \brief WithPrevFolder
    /// \return name of the file with previous folder. (Ex : '_BATI/3670_10383')
    ///
    std::string WithPrevFolder();

    ///
    /// \brief WithPrevFolderAndGMLExtension
    /// \return name of the file with previous folder and .gml extension (Ex : '_BATI/3670_10383.gml')
    ///
    std::string WithPrevFolderAndGMLExtension();

    ///
    /// \brief m_filename name of the file
    ///
    std::string m_filename;

    ///
    /// \brief m_filepath full path of the file
    ///
    std::string m_filepath;

    ///
    /// \brief m_type Type of the file (_BATI or _MNT)
    ///
    fileType m_type;

};

#endif // FILEINFO_H
