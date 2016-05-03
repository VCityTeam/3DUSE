// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __DATAPROFILE_HPP__
#define __DATAPROFILE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
#include "vecs.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The DataProfile class holds informations about a data set
////////////////////////////////////////////////////////////////////////////////
class   DataProfile
{
public:
    DataProfile();

    TVec3d m_bboxLowerBound;    ///< lower corner coordinates of data
    TVec3d m_bboxUpperBound;    ///< upper corner coordinates of data
    TVec3d m_offset;            ///< data offset (used to avoid big coordinates and
    float m_xStep;              ///< tile width
    float m_yStep;              ///< tile height
    int m_TileIdOriginX;        ///< tile id at the X origin
    int m_TileIdOriginY;        ///< tile id at the X origin
    // data origin : vec3
    // data offset : vec3
    // tile x size
    // tile y size
    // file prefix
    // file suffix
    // file ordering convention
    // srs -> proj string format ?

    // citygml
    // ign data

    std::string m_dataPathCityGML;  ///< default path for CityGML data
    int m_id;                   ///< id of the data profile
    std::string m_name;         ///< name of the data profile

private:
};
////////////////////////////////////////////////////////////////////////////////
// default data profile
DataProfile createDataProfileDefault();
DataProfile createDataProfileNone();
DataProfile createDataProfileParis();
DataProfile createDataProfileLyon();
DataProfile createDataProfileSablons();
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __DATAPROFILE_HPP__
