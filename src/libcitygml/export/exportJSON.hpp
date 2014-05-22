#ifndef __JSON_EXPORT_HPP_
#define __JSON_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "export.hpp"
#include "citygml.hpp"
//#include <fstream> // MT 05/05/2014
#include <osgDB/fstream>
#include <map>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterJSON class
/// Export JSON
class ExporterJSON : public Export
{
public:
    ExporterJSON();

    void setBasePath(const std::string& basePath);

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel& model, const std::string& fileName, const std::string& id);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject& obj);

    void addFilter(CityObjectsType filter, const std::string& name);
    void resetFilters();

private:
    void exportFeature(CityObject& obj, CityObjectsType type);
    int getNbFeature(CityModel& model, CityObjectsType type) const;
    int getNbFaces(CityObject& obj, CityObjectsType type) const;
    int getNbTris(CityObject& obj) const;

    void openScope();
    void closeScope(bool comma = false);
    void indent();

    std::string m_id;
    std::string m_basePath;
    std::string m_fileName;
    std::ofstream m_outFile;
    int m_indentDepth;

    std::map<CityObjectsType, std::string> m_filters;

    bool m_needComma;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __JSON_EXPORT_HPP_
