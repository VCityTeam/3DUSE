////////////////////////////////////////////////////////////////////////////////
#include "export.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
Exporter::Exporter()
    : m_temporalExport(false), m_date()
{

}
////////////////////////////////////////////////////////////////////////////////
void Exporter::exportCityModel(CityModel* model, const std::string& fileName)
{
    xmlDocPtr doc = NULL;       /* document pointer */
    xmlNodePtr root_node = NULL;

    LIBXML_TEST_VERSION;

    /*
     * Creates a new document, a node and set it as a root node
     */
    doc = xmlNewDoc(BAD_CAST "1.0");
    root_node = exportCityModelXml(*model);
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
void Exporter::exportCityObject(CityObject* model, const std::string& fileName)
{
    xmlDocPtr doc = NULL;       /* document pointer */
    xmlNodePtr root_node = NULL;

    LIBXML_TEST_VERSION;

    /*
     * Creates a new document, a node and set it as a root node
     */
    doc = xmlNewDoc(BAD_CAST "1.0");
    root_node = exportCityObjectModelXml(*model);
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
void Exporter::setTemporalExport(bool param)
{
    m_temporalExport = param;
}
////////////////////////////////////////////////////////////////////////////////
void Exporter::setDate(const QDateTime& date)
{
    m_date = date;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportEnvelopeXml(const citygml::Envelope& env)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "gml:Envelope");

    std::stringstream ss;
    TVec3d vec = env.getLowerBound();
    ss << vec.x << " " << vec.y << " " << vec.z;
    xmlNewChild(res, NULL, BAD_CAST "gml:lowerCorner", BAD_CAST ss.str().c_str());
    //ss.clear();
    ss.str(std::string());
    vec = env.getUpperBound();
    ss << vec.x << " " << vec.y << " " << vec.z;
    xmlNewChild(res, NULL, BAD_CAST "gml:upperCorner", BAD_CAST ss.str().c_str());

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportLinearRingXml(const citygml::LinearRing& ring)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "gml:LinearRing");
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST ring.getId().c_str());

    const std::vector<TVec3d>& vertices = ring.getVertices();

    std::stringstream ss;
    std::vector<TVec3d>::const_iterator it = vertices.begin();
    for(; it != vertices.end(); ++it)
    {
        ss << it->x << " " << it->y << " " << it->z << " ";
    }
    it = vertices.begin();
    ss << it->x << " " << it->y << " " << it->z;
    xmlNewChild(res, NULL, BAD_CAST "gml:posList", BAD_CAST ss.str().c_str());

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportPolygonXml(const citygml::Polygon& poly)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "gml:Polygon");
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST poly.getId().c_str());

    const std::vector<citygml::LinearRing*>& lrings = poly.getInteriorRings();
    std::vector<citygml::LinearRing*>::const_iterator it = lrings.begin();
    for(; it != lrings.end(); ++it)
    {
        /*xmlNodePtr node = */xmlNewChild(res, NULL, BAD_CAST "gml:interior", NULL);
        xmlAddChild(res, exportLinearRingXml(**it));
    }

    if(poly.getExteriorRing())
    {
        xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:exterior", NULL);
        xmlAddChild(node, exportLinearRingXml(*poly.getExteriorRing()));
    }

    /*xmlNodePtr node = xmlNewNode(NULL, BAD_CAST "gml:LinearRing");
    //xmlNewProp(node, BAD_CAST "gml:id", BAD_CAST ring.getId().c_str());
    std::stringstream ss;
    const std::vector<TVec3d>& vertices = poly.getVertices();
    std::vector<TVec3d>::const_iterator itv = vertices.begin();
    for(; itv != vertices.end(); ++itv)
    {
        ss << itv->x << " " << itv->y << " " << itv->z;
    }
    xmlNewChild(node, NULL, BAD_CAST "gml:posList", BAD_CAST ss.str().c_str());
    xmlAddChild(res, node);*/

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportRoofXml(const citygml::RoofSurface& /*geom*/)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "Roof");


    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportGeometryXml(const citygml::Geometry& geom, const std::string& nodeType)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST nodeType.c_str());
    //xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST bldg.getId().c_str());
    /*xmlNodePtr node1 = */xmlNewChild(res, NULL, BAD_CAST "gml:name", BAD_CAST geom.getId().c_str());
    xmlNodePtr node2 = xmlNewChild(res, NULL, BAD_CAST "bldg:lod3MultiSurface", NULL); // geom.getLOD();
    xmlNodePtr node3 = xmlNewChild(node2, NULL, BAD_CAST "gml:surfaceMember", NULL);

    const std::vector<citygml::Polygon*>& polys = geom.getPolygons();
    std::vector<citygml::Polygon*>::const_iterator it = polys.begin();
    for(; it != polys.end(); ++it)
    {
        xmlAddChild(node3, exportPolygonXml(**it));
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportGeometryXml(const citygml::Geometry& geom)
{
    switch(geom.getType())
    {
    case citygml::GT_Unknown:
        return exportGeometryXml(geom, "bldg:Unknown");
        break;
    case citygml::GT_Roof:
        return exportGeometryXml(geom, "bldg:Roof");
        break;
    case citygml::GT_Wall:
        return exportGeometryXml(geom, "bldg:Wall");
        break;
    case citygml::GT_Ground:
        return exportGeometryXml(geom, "bldg:Ground");
        break;
    case citygml::GT_Closure:
        return exportGeometryXml(geom, "bldg:Closure");
        break;
    case citygml::GT_Floor:
        return exportGeometryXml(geom, "bldg:Floor");
        break;
    case citygml::GT_InteriorWall:
        return exportGeometryXml(geom, "bldg:InteriorWall");
        break;
    case citygml::GT_Ceiling:
        return exportGeometryXml(geom, "bldg:Ceiling");
        break;
    default:
        break;
    }

    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportBuildingPart()
{
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportBuildingInstallationXml(const citygml::BuildingInstallation& bldg)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "bldg:Buildinginstallation");
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST bldg.getId().c_str());

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportBuildingXml(const citygml::Building& bldg)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "bldg:Building");
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST bldg.getId().c_str());

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportSurfaceXml(const citygml::CityObject& srf, const std::string& nodeType, xmlNodePtr parent=NULL)
{
    xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "bldg:boundedBy");
    xmlNodePtr res = xmlNewChild(root, NULL, BAD_CAST nodeType.c_str(), NULL);
    //xmlNodePtr res = xmlNewNode(NULL, BAD_CAST nodeType.c_str());
    xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST srf.getId().c_str());

    if(parent)
    {
        xmlAddChild(parent, root);
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent=NULL)
{
    xmlNodePtr res = NULL;

    switch(obj.getType())
    {
    case citygml::COT_GenericCityObject:
        break;
    case citygml::COT_Building:
        res = exportBuildingXml(static_cast<const citygml::Building&>(obj));
        break;
    case citygml::COT_Room:
        res = exportSurfaceXml(obj, "bldg:Room", parent);
        break;
    case citygml::COT_BuildingInstallation:
        //res = exportBuildingInstallationXml(static_cast<const citygml::BuildingInstallation&>(obj));
        res = exportSurfaceXml(obj, "bldg:BuildingInstallation", parent);
        break;
    case citygml::COT_BuildingFurniture:
        res = exportSurfaceXml(obj, "bldg:BuildingFurniture", parent);
        break;
    case citygml::COT_Door:
        res = exportSurfaceXml(obj, "bldg:Door", parent);
        break;
    case citygml::COT_Window:
        res = exportSurfaceXml(obj, "bldg:Window", parent);
        break;
    case citygml::COT_CityFurniture:
        res = exportSurfaceXml(obj, "bldg:CityFurniture", parent);
        break;
    case citygml::COT_Track:
        res = exportSurfaceXml(obj, "bldg:Track", parent);
        break;
    case citygml::COT_Road:
        res = exportSurfaceXml(obj, "bldg:Road", parent);
        break;
    case citygml::COT_Railway:
        res = exportSurfaceXml(obj, "bldg:Railway", parent);
        break;
    case citygml::COT_Square:
        res = exportSurfaceXml(obj, "bldg:Square", parent);
        break;
    case citygml::COT_PlantCover:
        res = exportSurfaceXml(obj, "bldg:PlantCover", parent);
        break;
    case citygml::COT_SolitaryVegetationObject:
        res = exportSurfaceXml(obj, "bldg:SolitaryVegetationObject", parent);
        break;
    case citygml::COT_WaterBody:
        res = exportSurfaceXml(obj, "bldg:WaterBody", parent);
        break;
    case citygml::COT_TINRelief:
        res = exportSurfaceXml(obj, "bldg:TINRelief", parent);
        break;
    case citygml::COT_LandUse:
        res = exportSurfaceXml(obj, "bldg:LandUse", parent);
        break;
    case citygml::COT_Tunnel:
        res = exportSurfaceXml(obj, "bldg:Tunnel", parent);
        break;
    case citygml::COT_Bridge:
        res = exportSurfaceXml(obj, "bldg:Bridge", parent);
        break;
    case citygml::COT_BridgeConstructionElement:
        res = exportSurfaceXml(obj, "bldg:BridgeConstructionElement", parent);
        break;
    case citygml::COT_BridgeInstallation:
        res = exportSurfaceXml(obj, "bldg:BridgeInstallation", parent);
        break;
    case citygml::COT_BridgePart:
        res = exportSurfaceXml(obj, "bldg:BridgePart", parent);
        break;
    case citygml::COT_BuildingPart:
        res = exportSurfaceXml(obj, "bldg:BuildingPart", parent);
        break;
    case citygml::COT_WallSurface:
        res = exportSurfaceXml(obj, "bldg:WallSurface", parent);
        break;
    case citygml::COT_RoofSurface:
        res = exportSurfaceXml(obj, "bldg:RoofSurface", parent);
        break;
    case citygml::COT_GroundSurface:
        res = exportSurfaceXml(obj, "bldg:GroundSurface", parent);
        break;
    case citygml::COT_ClosureSurface:
        res = exportSurfaceXml(obj, "bldg:ClosureSurface", parent);
        break;
    case citygml::COT_FloorSurface:
        res = exportSurfaceXml(obj, "bldg:FloorSurface", parent);
        break;
    case citygml::COT_InteriorWallSurface:
        res = exportSurfaceXml(obj, "bldg:InteriorWallSurface", parent);
        break;
    case citygml::COT_CeilingSurface:
        res = exportSurfaceXml(obj, "bldg:CeilingSurface", parent);
        break;
     default:
        break;
    }

    citygml::AttributesMap::const_iterator it = obj.getAttributes().begin();
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
    }

    const citygml::CityObjects& objs = obj.getChildren();
    citygml::CityObjects::const_iterator ito = objs.begin();
    for(; ito != objs.end(); ++ito)
    {
        //xmlAddChild(res, exportCityObjetXml(**ito, res));
        exportCityObjetXml(**ito, res);
    }

    const std::vector<citygml::Geometry*>& geoms = obj.getGeometries();
    std::vector<citygml::Geometry*>::const_iterator itg = geoms.begin();
    for(; itg != geoms.end(); ++itg)
    {
        xmlAddChild(res, exportGeometryXml(**itg));
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportCityModelXml(const citygml::CityModel& model)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "core:CityModel");

    // write envelope (bouned by)
    xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:boundedBy", NULL);
    xmlAddChild(node, exportEnvelopeXml(model.getEnvelope()));

    // do objects
    node = xmlNewChild(res, NULL, BAD_CAST "core:cityObjectMember", NULL);
    const citygml::CityObjects& objs = model.getCityObjectsRoots();
    citygml::CityObjects::const_iterator it = objs.begin();
    for(; it != objs.end(); ++it)
    {
        xmlNodePtr objnode = exportCityObjetXml(**it, node);
        if(objnode)
        xmlAddChild(node, objnode);
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
xmlNodePtr Exporter::exportCityObjectModelXml(const citygml::CityObject& obj)
{
    xmlNodePtr res = xmlNewNode(NULL, BAD_CAST "core:CityModel");

    // write envelope (bouned by)
    xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:boundedBy", NULL);
    xmlAddChild(node, exportEnvelopeXml(obj.getEnvelope()));

    // do objects
    node = xmlNewChild(res, NULL, BAD_CAST "core:cityObjectMember", NULL);
    const citygml::CityObjects& objs = obj.getChildren();
    citygml::CityObjects::const_iterator it = objs.begin();
    for(; it != objs.end(); ++it)
    {
        xmlAddChild(node, exportCityObjetXml(**it, node));
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
