// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "exportJSON.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterJSON::ExporterJSON()
    : m_indentDepth(0), m_genTexCoords(false), m_offsetX(0.0), m_offsetY(0.0), m_tileSizeX(0.0), m_tileSizeY(0.0)
{

}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::setBasePath(const std::string& basePath)
{
    m_basePath = basePath;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::setOffset(double offsetX, double offsetY)
{
    m_offsetX = offsetX;
    m_offsetY = offsetY;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::setTileSize(double tileSizeX, double tileSizeY)
{
    m_tileSizeX = tileSizeX;
    m_tileSizeY = tileSizeY;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportCityModel(CityModel& model, const std::string& fileName, const std::string& id)
{
    m_id = id;
    model.computeEnvelope();

    TVec3d p;

    // bldg
    #if 0
    addFilter(COT_WallSurface, "walls");
    addFilter(COT_RoofSurface, "roofs");
    m_outFile.open(m_basePath + "building/" + fileName + ".json");
    m_outFile << std::fixed;
    openScope(); // global scope
    indent(); m_outFile << "\"id\":\"" << id << "\",\n";
    indent(); m_outFile << "\"nbBldg\":" << getNbFeature(model, COT_Building) << ",\n";
    p = model.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = model.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    indent(); m_outFile << "\"listBldg\":";
    openScope(); // listBldg scope
    for(CityObject* obj : model.getCityObjectsRoots())
        if(obj && obj->getType() == COT_Building) exportCityObject(*obj);
    m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";
    closeScope();  // listBldg scope
    closeScope(); // global scope
    m_outFile.close();
    resetFilters();
    #endif

    // terrain
    #if 0
    addFilter(COT_TINRelief, "terrain");
    m_outFile.open(m_basePath + "terrain/" + fileName + ".json");
    m_outFile << std::fixed;
    openScope(); // global scope
    indent(); m_outFile << "\"id\":\"" << id << "\",\n";
    indent(); m_outFile << "\"nbTerrain\":" << getNbFeature(model, COT_TINRelief) << ",\n";
    p = model.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = model.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    indent(); m_outFile << "\"listTerrain\":";
    openScope(); // listTerrain scope
    for(CityObject* obj : model.getCityObjectsRoots())
        if(obj && obj->getType() == COT_TINRelief) exportCityObject(*obj);
    m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";
    closeScope();  // listTerrain scope
    closeScope(); // global scope
    m_outFile.close();
    resetFilters();
    #endif

    // terrain geo ref test
    #if 0
    std::cout << "offset : " << m_offsetX << ", " << m_offsetY << std::endl;
    m_genTexCoords = true;
    addFilter(COT_TINRelief, "terrain");
    m_outFile.open(m_basePath + "terrain/lod/" + fileName + ".json");
    m_outFile << std::fixed;
    openScope(); // global scope
    indent(); m_outFile << "\"id\":\"" << id << "\",\n";
    indent(); m_outFile << "\"nbTerrain\":" << getNbFeature(model, COT_TINRelief) << ",\n";
    p = model.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = model.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    indent(); m_outFile << "\"listTerrain\":";
    openScope(); // listTerrain scope
    for(CityObject* obj : model.getCityObjectsRoots())
        if(obj && obj->getType() == COT_TINRelief) exportCityObject(*obj);
    m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";
    closeScope();  // listTerrain scope
    closeScope(); // global scope
    m_outFile.close();
    resetFilters();
    #endif

    // bldg geo ref test
    #if 1
    m_genTexCoords = true;
    addFilter(COT_WallSurface, "walls");
    addFilter(COT_RoofSurface, "roofs");
	std::cout << m_basePath + "building/" + fileName + ".json" << std::endl;
    m_outFile.open(m_basePath + "building/" + fileName + ".json");
    m_outFile << std::fixed;
    openScope(); // global scope
    indent(); m_outFile << "\"id\":\"" << id << "\",\n";
    indent(); m_outFile << "\"nbBldg\":" << getNbFeature(model, COT_Building) << ",\n";
    p = model.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = model.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    indent(); m_outFile << "\"listBldg\":";
    openScope(); // listBldg scope
    for(CityObject* obj : model.getCityObjectsRoots())
        if(obj && obj->getType() == COT_Building) exportCityObject(*obj);
    m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";
    closeScope();  // listBldg scope
    closeScope(); // global scope
    m_outFile.close();
    resetFilters();
    #endif
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportCityObject(CityObject& obj)
{
    indent(); m_outFile << "\"" << obj.getId() << "\":";
    openScope(); // obj scope
    TVec3d p = obj.getEnvelope().getLowerBound();
    indent(); m_outFile << "\"min\":[" << p.x << "," << p.y << "," << p.z << "],\n";
    p = obj.getEnvelope().getUpperBound();
    indent(); m_outFile << "\"max\":[" << p.x << "," << p.y << "," << p.z << "],\n";

    for(const auto& it : m_filters)
    {
        indent(); m_outFile << "\"" << it.second << "\":";
        openScope(); // scope
        //indent(); m_outFile << "\"nbFace\":" << getNbFaces(obj, it.first) << ",\n";
        m_needComma = false;
        exportFeature(obj, it.first);
        m_outFile << "\n";
        closeScope(true);  // scope
    }

    //indent(); m_outFile << "\"listGeometries\":[" << 10 << "],\n";
    //indent(); m_outFile << "\"listNormals\":[" << 10 << "],\n";
    //indent(); m_outFile << "\"listUVs\":[" << 10 << "],\n";
    indent(); m_outFile << "\"semantique\":";
    openScope(); // semantique scope
    int nb = obj.getAttributes().size();
    int i = 0;
    for(const auto& attr : obj.getAttributes())
    {
        indent(); m_outFile << "\"" << attr.first << "\":\"" << attr.second << "\"";
        if(++i != nb)
            m_outFile << ",";
        m_outFile << "\n";
    }
    closeScope(); // semantique scope

    closeScope(true);  // obj scope
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::addFilter(CityObjectsType filter, const std::string& name)
{
    m_filters[filter] = name;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::resetFilters()
{
    m_filters.clear();
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportFeature(CityObject& obj, CityObjectsType type)
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

        if(!texture.empty() || m_genTexCoords)
        {
            std::vector<TVec2f> genTexCoords;

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

            indent(); m_outFile << "\"listGeometries\":[ ";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    for(const auto& vertex : poly->getVertices())
                    {
                        m_outFile << vertex.x << "," << vertex.y << "," << vertex.z << ",";
                        if(m_genTexCoords)
                        {
                            //compute tex coords
                            TVec2f tc;

                            tc.x = (vertex.x-m_offsetX)/m_tileSizeX;
                            tc.y = (vertex.y-m_offsetY)/m_tileSizeY;

                            tc.y = 1.0f - tc.y;

                            genTexCoords.push_back(tc);
                        }
                    }
                }
            }
            m_outFile.seekp(-1, std::ios_base::cur);
            m_outFile << " ],\n";

            indent(); m_outFile << "\"listNormals\":[ ";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    for(const auto& normal : poly->getNormals())
                    {
                        m_outFile << normal.x << "," << normal.y << "," << normal.z << ",";
                    }
                }
            }
            m_outFile.seekp(-1, std::ios_base::cur);
            m_outFile << " ],\n";

            indent(); m_outFile << "\"listUVs\":[ ";
            if(!m_genTexCoords)
            {
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

                        for(const auto& uv : poly->getTexCoords())
                        {
                            m_outFile << uv.x << "," << uv.y << ",";
                        }
                    }
                }
            }
            else
            {
                for(const auto& uv : genTexCoords)
                {
                    m_outFile << uv.x << "," << uv.y << ",";
                }
            }
            m_outFile.seekp(-1, std::ios_base::cur);
            m_outFile << " ],\n";

            int offset = 0;
            indent(); m_outFile << "\"listIndices\":[ ";
            for(Geometry* geom : obj.getGeometries())
            {
                for(Polygon* poly : geom->getPolygons())
                {
                    for(auto index : poly->getIndices())
                    {
                        m_outFile << offset+index << ",";
                    }
                    offset += poly->getVertices().size();
                }
            }
            m_outFile.seekp(-1, std::ios_base::cur);
            m_outFile << " ],\n";

            indent();
            if(!m_genTexCoords)
            {
                m_outFile << "\"texture\":\"" << "EXPORT_" << m_id.substr(0,4) << "-" <<  m_id.substr(5,5) << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";
                //if(!texture.empty())
                //    m_outFile << "\"texture\":\"" << "EXPORT_" << m_id.substr(0,4) << "-" <<  m_id.substr(5,5) << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";
                //else
                //    m_outFile << "\"texture\":\"\"\n";
                //closeScope(); // feature scope
            }
            else
            {
                m_outFile << "\"texture\":\"" << "tiles/tile_" << m_id << "\"\n";
            }
            --m_indentDepth;indent();m_outFile << "}";
        }
    }

    for(CityObject* child : obj.getChildren())
    {
        if(child) exportFeature(*child, type);
    }
}
////////////////////////////////////////////////////////////////////////////////
int ExporterJSON::getNbFeature(CityModel& model, CityObjectsType type) const
{
    int nb = 0;
    for(CityObject* obj : model.getCityObjectsRoots())
    {
        if(obj && obj->getType() == type)
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
