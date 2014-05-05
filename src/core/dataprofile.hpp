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
///
class DataProfile
{
public:
    DataProfile();

    TVec3d m_bboxLowerBound;    ///< lower corner coordinates of data
    TVec3d m_bboxUpperBound;    ///< upper corner coordinates of data
    TVec3d m_offset;            ///< data offset (used to avoid big coordinates and
    float m_xStep;              ///< tile width
    float m_yStep;              ///< tile height
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

    std::string m_dataPathCityGML;

private:
};
////////////////////////////////////////////////////////////////////////////////
// default data profile
DataProfile createDataProfileDefault();
DataProfile createDataProfileParis();
DataProfile createDataProfileLyon();
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __DATAPROFILE_HPP__
