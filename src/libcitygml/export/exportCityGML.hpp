#ifndef __CITYGML_EXPORT_HPP_
#define __CITYGML_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "export.hpp"
#include <libxml/tree.h>
#include "citygml.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The Exporter class
/// Export citygml
class ExporterCityGML : public Export
{
public:
    ExporterCityGML();

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel* model, const std::string& fileName);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject* model, const std::string& fileName);

private:
    xmlNodePtr exportCityObjectModelXml(const citygml::CityObject& obj);
    xmlNodePtr exportCityModelXml(const citygml::CityModel& model);
    xmlNodePtr exportEnvelopeXml(const citygml::Envelope& env);
    xmlNodePtr exportLinearRingXml(const citygml::LinearRing& ring);
    xmlNodePtr exportPolygonXml(const citygml::Polygon& poly);
    xmlNodePtr exportRoofXml(const citygml::RoofSurface& geom);
    xmlNodePtr exportGeometryXml(const citygml::Geometry& geom, const std::string& nodeType);
    xmlNodePtr exportGeometryXml(const citygml::Geometry& geom);
    xmlNodePtr exportBuildingPart();
    xmlNodePtr exportBuildingInstallationXml(const citygml::BuildingInstallation& bldg);
    xmlNodePtr exportBuildingXml(const citygml::Building& bldg);
    xmlNodePtr exportSurfaceXml(const citygml::CityObject& srf, const std::string& nodeType, xmlNodePtr parent);
    xmlNodePtr exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent);
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_EXPORT_HPP_
