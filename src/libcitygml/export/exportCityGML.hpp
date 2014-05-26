#ifndef __CITYGML_EXPORT_HPP_
#define __CITYGML_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "exporter.hpp"
#include <libxml/tree.h>
#include "citygml.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The Exporter class
/// Export citygml
class ExporterCityGML : public Exporter
{
public:
    ExporterCityGML();

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(const CityModel& model, const std::string& fileName);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(const std::vector<CityObject*>& objs, const std::string& fileName);

private:
    xmlNodePtr exportCityObjectModelXml(const std::vector<CityObject*>& objs);
    xmlNodePtr exportCityModelXml(const citygml::CityModel& model);
    xmlNodePtr exportEnvelopeXml(const citygml::Envelope& env, xmlNodePtr parent);
    xmlNodePtr exportLinearRingXml(const citygml::LinearRing& ring, xmlNodePtr parent);
    xmlNodePtr exportPolygonXml(const citygml::Polygon& poly, xmlNodePtr parent);
    xmlNodePtr exportGeometryGenericXml(const citygml::Geometry& geom, const std::string& nodeType, xmlNodePtr parent);
    xmlNodePtr exportGeometryXml(const citygml::Geometry& geom, xmlNodePtr parent);
    xmlNodePtr exportCityObjetGenericXml(const citygml::CityObject& obj, const std::string &nodeType, xmlNodePtr parent);
    xmlNodePtr exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent);
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_EXPORT_HPP_
