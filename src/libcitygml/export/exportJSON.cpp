////////////////////////////////////////////////////////////////////////////////
#include "exportJSON.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterJSON::ExporterJSON()
    : m_indentDepth(0)
{

}
////////////////////////////////////////////////////////////////////////////////
/*void exportNode()
{
    QString json;
    json += QString::fromUtf8("\"nbFaces\":") + QString::number(nbFaces) + QString::fromUtf8(",");
    json += QString::fromUtf8("\"min\":[") + util::trimmedBy(QString::number(min.x, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(min.y, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(min.z, 'f')) + QString::fromUtf8("],");
    json += QString::fromUtf8("\"max\":[") + util::trimmedBy(QString::number(max.x, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(max.y, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(max.z, 'f')) + QString::fromUtf8("],");

    json += QString::fromUtf8("\"listIndices\":[");
    for (unsigned int i = 0; i<listIndices->size(); i++)
    {
        json += QString::number(listIndices->at(i));
        if (i+1 < listIndices->size()) json += QString::fromUtf8(",");
    }
    json += QString::fromUtf8("],");

    json += QString::fromUtf8("\"listGeometrie\":[");
    for (unsigned int i = 0; i<listGeom->size(); i++)
    {
        json += util::trimmedBy(QString::number(listGeom->at(i).x, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listGeom->at(i).y, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listGeom->at(i).z, 'f'));
        if (i+1 < listGeom->size()) json += QString::fromUtf8(",");
    }
    json += QString::fromUtf8("],");

    json += QString::fromUtf8("\"listNormal\":[");
    for (unsigned int i = 0; i<listNormalByFaces->size(); i++)
    {
        json += util::trimmedBy(QString::number(listNormalByFaces->at(i).x, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listNormalByFaces->at(i).y, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listNormalByFaces->at(i).z, 'f'));
        if (i+1 < listNormalByFaces->size()) json += QString::fromUtf8(",");
    }
    json += QString::fromUtf8("]");

    return json;
}*/
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportCityModel(CityModel& model, const std::string& fileName, const std::string& id)
{
    m_id = id;
    model.computeEnvelope();

    m_outFile.open(fileName+".json");
    m_outFile << std::fixed;
    openScope(); // global scope
    indent(); m_outFile << "\"id\":\"" << id << "\",\n";
    indent(); m_outFile << "\"nbBldg\":" << getNbBldg(model) << ",\n";
    TVec3d p = model.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = model.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    indent(); m_outFile << "\"listBldg\":";
    openScope(); // listBldg scope
    for(CityObject* obj : model.getCityObjectsRoots())
        if(obj && obj->getType() == COT_Building) exportCityObject(*obj);
    closeScope();  // listBldg scope
    closeScope(); // global scope
    m_outFile.close();
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportCityObject(CityObject& obj)
{
    indent(); m_outFile << "\"" << obj.getId() << "\":";
    openScope(); // bldg scope
    TVec3d p = obj.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = obj.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";

    indent(); m_outFile << "\"walls\":";
    openScope(); // walls scope
    //indent(); m_outFile << "\"nbFace\":" << getNbFaces(obj, COT_WallSurface) << ",\n";
    m_needComma = false;
    exportWallsAndRoofs(obj, COT_WallSurface);
    m_outFile << "\n";
    closeScope(true);  // walls scope

    indent(); m_outFile << "\"roofs\":";
    openScope(); // roofs scope
    indent(); m_outFile << "\"nbFace\":" << getNbFaces(obj, COT_RoofSurface) << ",\n";
    m_needComma = false;
    exportWallsAndRoofs(obj, COT_RoofSurface);
    m_outFile << "\n";
    closeScope(true);  // roofs scope

    //indent(); m_outFile << "\"listGeometries\":[" << 10 << "],\n";
    //indent(); m_outFile << "\"listNormals\":[" << 10 << "],\n";
    //indent(); m_outFile << "\"listUVs\":[" << 10 << "],\n";
    indent(); m_outFile << "\"semantique\":";
    openScope(); // semantique scope
    int nb = obj.getAttributes().size();
    int i = 0;
    for(auto& attr : obj.getAttributes())
    {
        indent(); m_outFile << "\"" << attr.first << "\":\"" << attr.second << "\"";
        if(++i != nb)
            m_outFile << ",";
        m_outFile << "\n";
    }
    closeScope(); // semantique scope

    closeScope(true);  // bldg scope
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportWallsAndRoofs(CityObject& obj, CityObjectsType type)
{
    if(obj.getType() == type)
    {
        std::string texture;
        for(Geometry* geom : obj.getGeometries())
        {
            for(Polygon* poly : geom->getPolygons())
            {
                if(poly->getTexture())
                {
                    if(!texture.empty() && texture != poly->getTexture()->getUrl())
                        std::cout << "Multiple textures !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                    texture = poly->getTexture()->getUrl();
                }
            }
        }

        if(!texture.empty())
        {
            if(m_needComma)
            {
                m_outFile << ",\n";
            }
            else
            {
                m_needComma = true;
            }
            indent(); m_outFile << "\"" << obj.getId() << "\":";
            openScope(); // feature scope
            //indent(); m_outFile << "\"nbTri\":" << getNbTris(obj) << ",\n";

            indent(); m_outFile << "\"listGeometries\":[";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    //for(const auto& vertex : poly->getVertices())
                    for(int i=0; i<poly->getVertices().size(); ++i)
                    {
                        const TVec3d& vertex = poly->getVertices()[i];
                        m_outFile << vertex.x << "," << vertex.y << "," << vertex.z;
                        if(i != poly->getVertices().size()-1) m_outFile << ",";
                    }
                }
            }
            m_outFile << "],\n";

            indent(); m_outFile << "\"listNormals\":[";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    //for(const auto& normal : poly->getNormals())
                    for(int i=0; i<poly->getNormals().size(); ++i)
                    {
                        const TVec3f& normal = poly->getNormals()[i];
                        m_outFile << normal.x << "," << normal.y << "," << normal.z;
                        if(i != poly->getNormals().size()-1) m_outFile << ",";
                    }
                }
            }
            m_outFile << "],\n";

            indent(); m_outFile << "\"listUVs\":[";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    /*if(poly->getTexture())
                    {
                        if(!texture.empty() && texture != poly->getTexture()->getUrl())
                            std::cout << "Multiple textures !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                        texture = poly->getTexture()->getUrl();
                    }*/

                    //for(const auto& uv : poly->getTexCoords())
                    for(int i=0; i<poly->getTexCoords().size(); ++i)
                    {
                        const TVec2f& uv = poly->getTexCoords()[i];
                        m_outFile << uv.x << "," << uv.y;
                        if(i != poly->getTexCoords().size()-1) m_outFile << ",";
                    }
                }
            }
            m_outFile << "],\n";

            indent(); m_outFile << "\"listIndices\":[";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    //for(auto index : poly->getIndices())
                    for(int i=0; i<poly->getIndices().size(); ++i)
                    {
                        unsigned int index = poly->getIndices()[i];
                        m_outFile << index;
                        if(i != poly->getIndices().size()-1) m_outFile << ",";
                    }
                }
            }
            m_outFile << "],\n";

            indent(); m_outFile << "\"texture\":\"" << "EXPORT_" << m_id.substr(0,4) << "-" <<  m_id.substr(5,5) << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";
            //if(!texture.empty())
            //    m_outFile << "\"texture\":\"" << "EXPORT_" << m_id.substr(0,4) << "-" <<  m_id.substr(5,5) << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";
            //else
            //    m_outFile << "\"texture\":\"\"\n";
            //closeScope(); // feature scope
            --m_indentDepth;indent();m_outFile << "}";
        }
    }

    for(CityObject* child : obj.getChildren())
    {
        if(child) exportWallsAndRoofs(*child, type);
    }
}
////////////////////////////////////////////////////////////////////////////////
int ExporterJSON::getNbBldg(CityModel& model) const
{
    int nb = 0;
    for(CityObject* obj : model.getCityObjectsRoots())
    {
        if(obj && obj->getType() == COT_Building)
        {
            ++nb;
        }
    }

    return nb;
}
////////////////////////////////////////////////////////////////////////////////
int ExporterJSON::getNbFaces(CityObject& obj, CityObjectsType type) const
{
    int nb = 0;
    if(obj.getType() == type)
    {
        ++nb;
    }

    for(citygml::CityObject* child : obj.getChildren())
    {
        nb += getNbFaces(*child, type);
    }

    return nb;
}
////////////////////////////////////////////////////////////////////////////////
int ExporterJSON::getNbTris(CityObject& obj) const
{
    int nb = 0;
    for(Geometry* geom : obj.getGeometries())
    {
        for(Polygon* poly : geom->getPolygons())
        {
            nb += poly->getIndices().size()/3;
        }
    }
    return nb;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::openScope()
{
    m_outFile << "{\n";
    ++m_indentDepth;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::closeScope(bool comma)
{
    --m_indentDepth;
    indent();
    if(comma)
        m_outFile << "},\n";
    else
        m_outFile << "}\n";
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::indent()
{
    for(int i=0; i<m_indentDepth; ++i)
    {
        m_outFile << "    ";
    }
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
