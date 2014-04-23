#ifndef __EXPORTOBJ_HPP__
#define __EXPORTOBJ_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "export.hpp"
#include "citygml.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterOBJ class
/// Export OBJ
class ExporterOBJ : public Export
{
public:
    ExporterOBJ();

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel* model, const std::string& fileName);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject* model, const std::string& fileName);

private:
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __EXPORTOBJ_HPP__
