#ifndef __CITYGML_EXPORT_HPP_
#define __CITYGML_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "exporter.hpp"
#include <libxml/tree.h>
#include "citymodel.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The Exporter class
/// Export citygml
/// Can be used with one step by writing a citymodel or an array of cityobjects
/// or can be used by appending cityobjects
class ExporterCityGML : public Exporter
{
public:
    ExporterCityGML(const std::string& filename);
    ~ExporterCityGML();

    /// \bief
    void initExport(bool createCityModelRootNode=true);

    void endExport();

    /// \brief exportCityModel
    /// \param model
    void exportCityModel(const CityModel& model);

    /// \brief exportCityObject
    /// \param objs
    void exportCityObject(const std::vector<const CityObject*>& objs);

    void appendCityObject(const std::vector<const CityObject*>& objs);

    void appendCityObject(const CityObject& obj);

private:
    xmlNodePtr exportCityObjectModelXml(const std::vector<const CityObject*>& objs);
    xmlNodePtr exportCityModelXml(const citygml::CityModel& model);
    xmlNodePtr exportEnvelopeXml(const citygml::Envelope& env, xmlNodePtr parent);
    xmlNodePtr exportLinearRingXml(const citygml::LinearRing& ring, xmlNodePtr parent);
    xmlNodePtr exportPolygonXml(const citygml::Polygon& poly, xmlNodePtr parent);
    xmlNodePtr exportGeometryGenericXml(const citygml::Geometry& geom, const std::string& nodeType, xmlNodePtr parent);
    xmlNodePtr exportGeometryXml(const citygml::Geometry& geom, xmlNodePtr parent);
    xmlNodePtr exportCityObjetGenericXml(const citygml::CityObject& obj, const std::string &nodeType, xmlNodePtr parent);
    xmlNodePtr exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent);

    std::string m_fileName;
    xmlDocPtr m_doc;
    xmlNodePtr m_root_node;
    xmlNodePtr m_citymodel_node;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_EXPORT_HPP_
