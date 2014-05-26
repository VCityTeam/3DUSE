#ifndef __EXPORTOBJASSIMP_HPP__
#define __EXPORTOBJASSIMP_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "export.hpp"
#include "citygml.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterOBJAssimp class
/// Export OBJ
class ExporterOBJAssimp : public Export
{
public:
    ExporterOBJAssimp();

    void addFilter(citygml::CityObjectsType filter, const std::string& name);

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel& model, const std::string& fileName);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject& obj, const std::string& fileName);

private:
    void exportCityObject(CityObject& obj, citygml::CityObjectsType filter=COT_All);


};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __EXPORTOBJASSIMP_HPP__
