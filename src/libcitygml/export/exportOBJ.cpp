////////////////////////////////////////////////////////////////////////////////
#include "exportOBJ.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterOBJ::ExporterOBJ()
{

}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::addFilter(citygml::CityObjectsType filter, const std::string& name)
{
    m_filters.push_back(filter);
    m_filterNames.push_back(name);
    m_filterOffsets[filter] = 1;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityModel(CityModel& model, const std::string& fileName)
{
    if(m_filters.size() == 0)
    {
        addFilter(COT_All, "");
    }

    for(int i=0; i<m_filters.size(); ++i)
    {
        m_outFile.open(fileName+m_filterNames[i]+".obj");
        m_outFile << std::fixed;
        m_outFile << "# CityGML test export\n\n";
        m_outFile << "o " << model.getId() << "\n\n";
        m_outFile << "mtllib " << fileName+m_filterNames[i] << ".mtl" << "\n\n";
        for(CityObject* obj : model.getCityObjectsRoots())
            if(obj) exportCityObject(*obj, m_filters[i]);
        m_outFile.close();
        exportMaterials(fileName+m_filterNames[i]+".mtl");
    }
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityObject(CityObject& obj, const std::string& fileName)
{
    if(m_filters.size() == 0)
    {
        addFilter(COT_All, "");
    }

    for(int i=0; i<m_filters.size(); ++i)
    {
        m_outFile.open(fileName+m_filterNames[i]+".obj");
        m_outFile << std::fixed;
        m_outFile << "# CityGML test export\n\n";
        m_outFile << "o " << obj.getId() << "\n\n";
        m_outFile << "mtllib " << fileName+m_filterNames[i] << ".mtl" << "\n\n";
        exportCityObject(obj, m_filters[i]);
        m_outFile.close();
        exportMaterials(fileName+m_filterNames[i]+".mtl");
    }

    /*exportCityObject(obj); // fill arrays

    m_outFile.open(fileName+"Wall.obj");
    m_outFile << "# CityGML test export\n\n";
    m_outFile << "o " << obj.getId() << "\n\n";
    for(const TVec3d& v : m_verticesWall)
    {
        m_outFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    m_outFile << "\n";
    for(const TVec3f& vn : m_normalsWall)
    {
        m_outFile << "vn " << vn.x << " " << vn.y << " " << vn.z << "\n";
    }
    m_outFile << "\n";
    for(int i=0; i<m_indicesWall.size(); i+=3)
    {
        m_outFile << "f " << m_indicesWall[i+0] << "//" << m_indicesWall[i+0] << " " <<
                             m_indicesWall[i+1] << "//" << m_indicesWall[i+1] << " " <<
                             m_indicesWall[i+2] << "//" << m_indicesWall[i+2] << "\n";
    }
    m_outFile << "\n";
    m_outFile.close();

    m_outFile.open(fileName+"Roof.obj");
    m_outFile << "# CityGML test export\n\n";
    m_outFile << "o " << obj.getId() << "\n\n";
    for(const TVec3d& v : m_verticesRoof)
    {
        m_outFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    m_outFile << "\n";
    for(const TVec3f& vn : m_normalsRoof)
    {
        m_outFile << "vn " << vn.x << " " << vn.y << " " << vn.z << "\n";
    }
    m_outFile << "\n";
    for(int i=0; i<m_indicesRoof.size(); i+=3)
    {
        m_outFile << "f " << m_indicesRoof[i+0] << "//" << m_indicesRoof[i+0] << " " <<
                             m_indicesRoof[i+1] << "//" << m_indicesRoof[i+1] << " " <<
                             m_indicesRoof[i+2] << "//" << m_indicesRoof[i+2] << "\n";
    }
    m_outFile << "\n";
    m_outFile.close();

    m_outFile.open(fileName+"Terrain.obj");
    m_outFile << "# CityGML test export\n\n";
    m_outFile << "o " << obj.getId() << "\n\n";
    for(const TVec3d& v : m_verticesTerrain)
    {
        m_outFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    m_outFile << "\n";
    for(const TVec3f& vn : m_normalsTerrain)
    {
        m_outFile << "vn " << vn.x << " " << vn.y << " " << vn.z << "\n";
    }
    m_outFile << "\n";
    for(int i=0; i<m_indicesTerrain.size(); i+=3)
    {
        m_outFile << "f " << m_indicesTerrain[i+0] << "//" << m_indicesTerrain[i+0] << " " <<
                             m_indicesTerrain[i+1] << "//" << m_indicesTerrain[i+1] << " " <<
                             m_indicesTerrain[i+2] << "//" << m_indicesTerrain[i+2] << "\n";
    }
    m_outFile << "\n";
    m_outFile.close();*/
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityObject(CityObject& obj, citygml::CityObjectsType filter)
{
    if(filter == COT_All || obj.getType() == filter)
    {
        int& offset = m_filterOffsets[filter];
        for(citygml::Geometry* geom : obj.getGeometries())
        {
            for(citygml::Polygon* poly : geom->getPolygons())
            {
                if(poly->getTexture())
                {
                    std::string mat = poly->getTexture()->getUrl();
                    mat = mat.substr(mat.find_last_of('/')+1);
                    mat = mat.substr(0, mat.find_last_of('.'));
                    m_outFile << "usemtl " << mat << "\n";
                    m_materials[mat] = poly->getTexture()->getUrl();
                }

                m_outFile << "g " << poly->getId() << "\n\n";

                for(const TVec3d& v : poly->getVertices())
                {
                    m_outFile << "v " << v.x-m_offsetX << " " << v.y-m_offsetY << " " << v.z << "\n";
                    //m_outFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
                }
                for(const TVec3f& vn : poly->getNormals())
                {
                    m_outFile << "vn " << vn.x << " " << vn.y << " " << vn.z << "\n";
                }
                //*
                for(const TVec2f& vt : poly->getTexCoords())
                {
                    m_outFile << "vt " << vt.x << " " << vt.y << "\n";
                }
                if(poly->getTexCoords().size() > 0)
                {
                    for(int i=0; i<poly->getIndices().size(); i+=3)
                    {
                        m_outFile << "f " << offset+poly->getIndices()[i+0] << "/" << offset+poly->getIndices()[i+0] << "/" << offset+poly->getIndices()[i+0] << " " <<
                                             offset+poly->getIndices()[i+1] << "/" << offset+poly->getIndices()[i+1] << "/" << offset+poly->getIndices()[i+1] << " " <<
                                             offset+poly->getIndices()[i+2] << "/" << offset+poly->getIndices()[i+2] << "/" << offset+poly->getIndices()[i+2] << "\n";
                    }
                }
                else
                {
                //*/
                    for(int i=0; i<poly->getIndices().size(); i+=3)
                    {
                        m_outFile << "f " << offset+poly->getIndices()[i+0] << "//" << offset+poly->getIndices()[i+0] << " " <<
                                             offset+poly->getIndices()[i+1] << "//" << offset+poly->getIndices()[i+1] << " " <<
                                             offset+poly->getIndices()[i+2] << "//" << offset+poly->getIndices()[i+2] << "\n";
                    }
                }
                //*/
                offset += poly->getVertices().size();
            }
        }
    }

    for(citygml::CityObject* child : obj.getChildren())
    {
        exportCityObject(*child, filter);
    }

    /*int* offset = nullptr;
    std::vector<TVec3d>* vertices = nullptr;
    std::vector<TVec3f>* normals = nullptr;
    std::vector<unsigned int>* indices = nullptr;
    switch(obj.getType())
    {
    case citygml::COT_WallSurface:
        offset = &m_indexOffsetWall;
        vertices = &m_verticesWall;
        normals = &m_normalsWall;
        indices = &m_indicesWall;
        break;
    case citygml::COT_RoofSurface:
        offset = &m_indexOffsetRoof;
        vertices = &m_verticesRoof;
        normals = &m_normalsRoof;
        indices = &m_indicesRoof;
        break;
    default:
        offset = &m_indexOffsetTerrain;
        vertices = &m_verticesTerrain;
        normals = &m_normalsTerrain;
        indices = &m_indicesTerrain;
        break;
    }
    for(citygml::Geometry* geom : obj.getGeometries())
    {
        for(citygml::Polygon* poly : geom->getPolygons())
        {
            vertices->insert(vertices->end(), poly->getVertices().begin(), poly->getVertices().end());
            normals->insert(normals->end(), poly->getNormals().begin(), poly->getNormals().end());
            for(int i=0; i<poly->getIndices().size(); ++i)
            {
                indices->push_back((*offset)+poly->getIndices()[i]);
            }
            *offset += poly->getIndices().size();
        }
    }

    for(citygml::CityObject* child : obj.getChildren())
    {
        exportCityObject(*child);
    }*/
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportMaterials(const std::string& filename)
{
    std::ofstream mat(filename);
    for(auto& it : m_materials)
    {
        mat << "newmtl " << it.first << "\n";
        mat << "Ka 1.000000 1.000000 1.000000\n";
        mat << "Kd 1.000000 1.000000 1.000000\n";
        mat << "Ks 0.000000 0.000000 0.000000\n";
        mat << "Tr 1.000000\n";
        mat << "illum 1\n";
        mat << "Ns 0.000000\n";
        mat << "map_Kd " << it.second << "\n\n";
    }
    mat.close();
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
