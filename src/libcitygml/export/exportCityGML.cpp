////////////////////////////////////////////////////////////////////////////////
#include "exportCityGML.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterCityGML::ExporterCityGML()
{

}
////////////////////////////////////////////////////////////////////////////////
void ExporterCityGML::exportCityModel(const CityModel& model, const std::string& fileName)
{
    xmlDocPtr doc = NULL;       /* document pointer */
    xmlNodePtr root_node = NULL;

    LIBXML_TEST_VERSION;

    /*
     * Creates a new document, a node and set it as a root node
     */
    doc = xmlNewDoc(BAD_CAST "1.0");
    root_node = exportCityModelXml(model);
    xmlDocSetRootElement(doc, root_node);

    /*
     * Dumping document to stdio or file
     */
    xmlSaveFormatFileEnc(fileName.c_str(), doc, "UTF-8", 1);

    /*free the document */
    //xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    xmlCleanupParser();

    /*
     * this is to debug memory for regression tests
     */
    xmlMemoryDump();
}
////////////////////////////////////////////////////////////////////////////////
void ExporterCityGML::exportCityObject(const std::vector<CityObject*>& models, const std::string& fileName)
{
    xmlDocPtr doc = NULL;       /* document pointer */
    xmlNodePtr root_node = NULL;

    LIBXML_TEST_VERSION;

    /*
     * Creates a new document, a node and set it as a root node
     */
    doc = xmlNewDoc(BAD_CAST "1.0");
    root_node = exportCityObjectModelXml(models);
    xmlDocSetRootElement(doc, root_node);

    /*
     * Dumping document to stdio or file
     */
    xmlSaveFormatFileEnc(fileName.c_str(), doc, "UTF-8", 1);

    /*free the document */
    //xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    xmlCleanupParser();

    /*
     * this is to debug memory for regression tests
     */
    xmlMemoryDump();
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportEnvelopeXml(const citygml::Envelope& env, xmlNodePtr parent)
{
    xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST "gml:Envelope", NULL);

    std::stringstream ss;
    TVec3d vec = env.getLowerBound();
    ss << std::fixed << vec.x << " " << vec.y << " " << vec.z;
    xmlNewChild(res, NULL, BAD_CAST "gml:lowerCorner", BAD_CAST ss.str().c_str());
    //ss.clear();
    ss.str(std::string());
    vec = env.getUpperBound();
    ss << std::fixed << vec.x << " " << vec.y << " " << vec.z;
    xmlNewChild(res, NULL, BAD_CAST "gml:upperCorner", BAD_CAST ss.str().c_str());

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportLinearRingXml(const citygml::LinearRing& ring, xmlNodePtr parent)
{
    xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST "gml:LinearRing", NULL);
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST ring.getId().c_str());

    std::stringstream ss;
    for(const TVec3d& v : ring.getVertices())
    {
        ss << std::fixed << v.x << " " << v.y << " " << v.z << " ";
    }
    const TVec3d& v = ring.getVertices().front();
    ss << std::fixed << v.x << " " << v.y << " " << v.z;
    xmlNewChild(res, NULL, BAD_CAST "gml:posList", BAD_CAST ss.str().c_str());

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportPolygonXml(const citygml::Polygon& poly, xmlNodePtr parent)
{
    xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST "gml:Polygon", NULL);
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST poly.getId().c_str());

    const std::vector<citygml::LinearRing*>& lrings = poly.getInteriorRings();
    std::vector<citygml::LinearRing*>::const_iterator it = lrings.begin();
    for(; it != lrings.end(); ++it)
    {
        xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:interior", NULL);
        exportLinearRingXml(**it, node);
    }

    if(poly.getExteriorRing())
    {
        xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:exterior", NULL);
        exportLinearRingXml(*poly.getExteriorRing(), node);
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportGeometryGenericXml(const citygml::Geometry& geom, const std::string& nodeType, xmlNodePtr parent)
{
    xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL);
    //xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST bldg.getId().c_str());
    /*xmlNodePtr node1 = */xmlNewChild(res, NULL, BAD_CAST "gml:name", BAD_CAST geom.getId().c_str());
    xmlNodePtr node2 = xmlNewChild(res, NULL, BAD_CAST "bldg:lod3MultiSurface", NULL); // geom.getLOD();
    xmlNodePtr node3 = xmlNewChild(node2, NULL, BAD_CAST "gml:surfaceMember", NULL);

    const std::vector<citygml::Polygon*>& polys = geom.getPolygons();
    std::vector<citygml::Polygon*>::const_iterator it = polys.begin();
    for(; it != polys.end(); ++it)
    {
        exportPolygonXml(**it, node3);
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportGeometryXml(const citygml::Geometry& geom, xmlNodePtr parent)
{
    switch(geom.getType())
    {
    case citygml::GT_Unknown:
        return exportGeometryGenericXml(geom, "bldg:Unknown", parent);
        break;
    case citygml::GT_Roof:
        return exportGeometryGenericXml(geom, "bldg:Roof", parent);
        break;
    case citygml::GT_Wall:
        return exportGeometryGenericXml(geom, "bldg:Wall", parent);
        break;
    case citygml::GT_Ground:
        return exportGeometryGenericXml(geom, "bldg:Ground", parent);
        break;
    case citygml::GT_Closure:
        return exportGeometryGenericXml(geom, "bldg:Closure", parent);
        break;
    case citygml::GT_Floor:
        return exportGeometryGenericXml(geom, "bldg:Floor", parent);
        break;
    case citygml::GT_InteriorWall:
        return exportGeometryGenericXml(geom, "bldg:InteriorWall", parent);
        break;
    case citygml::GT_Ceiling:
        return exportGeometryGenericXml(geom, "bldg:Ceiling", parent);
        break;
    default:
        break;
    }

    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportCityObjetGenericXml(const citygml::CityObject& obj, const std::string& nodeType, xmlNodePtr parent)
{
    xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL);
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST obj.getId().c_str());

    /*for(const auto& child : obj.getChildren())
    {
        exportCityObjetXml(child, res);
    }*/

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent)
{
    xmlNodePtr res = NULL;

    std::cout << "type : " << obj.getTypeAsString() << std::endl;

    switch(obj.getType())
    {
    case citygml::COT_GenericCityObject:
        break;
    case citygml::COT_Building:
        res = exportCityObjetGenericXml(obj, "bldg:Building", parent);
        break;
    case citygml::COT_Room:
        res = exportCityObjetGenericXml(obj, "bldg:Room", parent);
        break;
    case citygml::COT_BuildingInstallation:
        res = exportCityObjetGenericXml(obj, "bldg:BuildingInstallation", parent);
        break;
    case citygml::COT_BuildingFurniture:
        res = exportCityObjetGenericXml(obj, "bldg:BuildingFurniture", parent);
        break;
    case citygml::COT_Door:
        res = exportCityObjetGenericXml(obj, "bldg:Door", parent);
        break;
    case citygml::COT_Window:
        res = exportCityObjetGenericXml(obj, "bldg:Window", parent);
        break;
    case citygml::COT_CityFurniture:
        res = exportCityObjetGenericXml(obj, "bldg:CityFurniture", parent);
        break;
    case citygml::COT_Track:
        res = exportCityObjetGenericXml(obj, "bldg:Track", parent);
        break;
    case citygml::COT_Road:
        res = exportCityObjetGenericXml(obj, "bldg:Road", parent);
        break;
    case citygml::COT_Railway:
        res = exportCityObjetGenericXml(obj, "bldg:Railway", parent);
        break;
    case citygml::COT_Square:
        res = exportCityObjetGenericXml(obj, "bldg:Square", parent);
        break;
    case citygml::COT_PlantCover:
        res = exportCityObjetGenericXml(obj, "bldg:PlantCover", parent);
        break;
    case citygml::COT_SolitaryVegetationObject:
        res = exportCityObjetGenericXml(obj, "bldg:SolitaryVegetationObject", parent);
        break;
    case citygml::COT_WaterBody:
        res = exportCityObjetGenericXml(obj, "bldg:WaterBody", parent);
        break;
    case citygml::COT_TINRelief:
        res = exportCityObjetGenericXml(obj, "bldg:TINRelief", parent);
        break;
    case citygml::COT_LandUse:
        res = exportCityObjetGenericXml(obj, "bldg:LandUse", parent);
        break;
    case citygml::COT_Tunnel:
        res = exportCityObjetGenericXml(obj, "bldg:Tunnel", parent);
        break;
    case citygml::COT_Bridge:
        res = exportCityObjetGenericXml(obj, "bldg:Bridge", parent);
        break;
    case citygml::COT_BridgeConstructionElement:
        res = exportCityObjetGenericXml(obj, "bldg:BridgeConstructionElement", parent);
        break;
    case citygml::COT_BridgeInstallation:
        res = exportCityObjetGenericXml(obj, "bldg:BridgeInstallation", parent);
        break;
    case citygml::COT_BridgePart:
        res = exportCityObjetGenericXml(obj, "bldg:BridgePart", parent);
        break;
    case citygml::COT_BuildingPart:
        res = exportCityObjetGenericXml(obj, "bldg:BuildingPart", parent);
        break;
    case citygml::COT_WallSurface:
        res = exportCityObjetGenericXml(obj, "bldg:WallSurface", parent);
        break;
    case citygml::COT_RoofSurface:
        res = exportCityObjetGenericXml(obj, "bldg:RoofSurface", parent);
        break;
    case citygml::COT_GroundSurface:
        res = exportCityObjetGenericXml(obj, "bldg:GroundSurface", parent);
        break;
    case citygml::COT_ClosureSurface:
        res = exportCityObjetGenericXml(obj, "bldg:ClosureSurface", parent);
        break;
    case citygml::COT_FloorSurface:
        res = exportCityObjetGenericXml(obj, "bldg:FloorSurface", parent);
        break;
    case citygml::COT_InteriorWallSurface:
        res = exportCityObjetGenericXml(obj, "bldg:InteriorWallSurface", parent);
        break;
    case citygml::COT_CeilingSurface:
        res = exportCityObjetGenericXml(obj, "bldg:CeilingSurface", parent);
        break;
     default:
        break;
    }

    /*citygml::AttributesMap::const_iterator it = obj.getAttributes().begin();
    while(it != obj.getAttributes().end())
    {
        std::string attr = "bldg:" + it->first;
        xmlNewChild(res, NULL, BAD_CAST attr.c_str(), BAD_CAST it->second.c_str());

        if(m_temporalExport == true)
        {
            int year;
            if(it->first == "yearOfConstruction")
            {
                std::stringstream ss(it->second);
                ss >> year;
                if(m_date.date().year() < year)
                {
                    // discard
                    return NULL;
                }
            }
            else if(it->first == "yearOfDemolition")
            {
                std::stringstream ss(it->second);
                ss >> year;
                if(m_date.date().year() > year)
                {
                    // discard
                    return NULL;
                }
            }
        }

        ++it;
    }*/

    for(auto& geom : obj.getGeometries())
    {
        if(res) exportGeometryXml(*geom, res);
        else exportGeometryXml(*geom, parent);
    }

    for(auto& child : obj.getChildren())
    {
        if(res) exportCityObjetXml(*child, res);
        else exportCityObjetXml(*child, parent);
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportCityModelXml(const citygml::CityModel& model)
{
    xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "core:CityModel");

    // write envelope (bouned by)
    xmlNodePtr nodeEnv = xmlNewChild(root, NULL, BAD_CAST "gml:boundedBy", NULL);
    exportEnvelopeXml(model.getEnvelope(), nodeEnv);

    // do objects
    for(const citygml::CityObject* obj : model.getCityObjectsRoots())
    {
        xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "core:cityObjectMember", NULL);
        exportCityObjetXml(*obj, node);
    }

    return root;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr ExporterCityGML::exportCityObjectModelXml(const std::vector<CityObject*>& objs)
{
    xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "core:CityModel");

    // write envelope (bouned by)
    xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "gml:boundedBy", NULL);
    Envelope env;
    for(const CityObject* obj : objs) // compute enveloppe
    {
        env.merge(obj->getEnvelope());
    }
    exportEnvelopeXml(env, node);

    // do objects
    for(const CityObject* obj : objs)
    {
        xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "core:cityObjectMember", NULL);
        exportCityObjetXml(*obj, node);
    }

    return root;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
