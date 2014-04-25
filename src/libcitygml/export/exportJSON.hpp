#ifndef __JSON_EXPORT_HPP_
#define __JSON_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "export.hpp"
#include "citygml.hpp"
#include <fstream>
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

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel& model, const std::string& fileName, const std::string& id);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject& obj);

private:
    void exportWallsAndRoofs(CityObject& obj, CityObjectsType type);
    int getNbBldg(CityModel& model) const;
    int getNbFaces(CityObject& obj, CityObjectsType type) const;
    int getNbTris(CityObject& obj) const;

    void openScope();
    void closeScope(bool comma = false);
    void indent();

    std::string m_id;
    std::ofstream m_outFile;
    int m_indentDepth;

    bool m_needComma;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __JSON_EXPORT_HPP_
