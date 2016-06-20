#include "FileInfo.h"

FileInfo::FileInfo(std::string filepath)
{
    m_filepath = filepath;

    //Find filename
    size_t pos_lastfolder = filepath.find_last_of("/");
    size_t pos_extension = filepath.find(".gml");

    m_filename = filepath.substr(pos_lastfolder + 1, pos_extension - pos_lastfolder - 1);

    //Find type from path
    if (m_filepath.find("_BATI") != std::string::npos)
        m_type = fileType::_BATI;
    else if (m_filepath.find("_MNT") != std::string::npos)
        m_type = fileType::_MNT;

}

std::string FileInfo::WithGMLExtension()
{
    return m_filename.append(".gml");
}

std::string FileInfo::WithPrevFolder()
{
    if(m_type == fileType::_BATI)
        return "_BATI/" + m_filename;
    else if(m_type == fileType::_MNT)
        return "_MNT/" + m_filename;
    else
        return "";
}

std::string FileInfo::WithPrevFolderAndGMLExtension()
{
    std::string result = this->WithPrevFolder() + ".gml";
    return result;
}
