#ifndef __EXPORTOBJ_HPP__
#define __EXPORTOBJ_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "exporter.hpp"
#include "citygml.hpp"
//#include <fstream> // MT 05/05/2014
#include <osgDB/fstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterOBJ class
/// Export OBJ
class ExporterOBJ : public Exporter
{
public:
    ExporterOBJ();

    void addFilter(citygml::CityObjectsType filter, const std::string& name);
    void resetFilters();

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(const CityModel& model, const std::string& fileName);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(const CityObject& obj, const std::string& fileName);

private:
    void exportCityObject(const CityObject& obj, citygml::CityObjectsType filter=COT_All);
    void exportMaterials(const std::string& filename);

    std::ofstream m_outFile;
    //std::vector<TVec3d> m_vertices;
    //std::vector<TVec3f> m_normals;
    //std::vector<unsigned int> m_indices;
    //int m_indexOffset;

    //std::vector<TVec3d> m_verticesWall, m_verticesRoof, m_verticesTerrain;
    //std::vector<TVec3f> m_normalsWall, m_normalsRoof, m_normalsTerrain;
    //std::vector<unsigned int> m_indicesWall, m_indicesRoof, m_indicesTerrain;
    //int m_indexOffsetWall, m_indexOffsetRoof, m_indexOffsetTerrain;

    std::vector<citygml::CityObjectsType> m_filters;
    std::vector<std::string> m_filterNames;
    std::map<citygml::CityObjectsType, int> m_filterOffsets;
    std::map<std::string, std::string> m_materials;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __EXPORTOBJ_HPP__
