// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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
void ExporterOBJ::resetFilters()
{
    m_filterOffsets.clear();
    m_filters.clear();
    m_filterNames.clear();
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityObjects(const std::vector<CityObject*>& objs, const std::string& fileName)
{
    if(m_filters.size() == 0)
    {
        addFilter(COT_All, "");
    }

    for(size_t i=0; i<m_filters.size(); ++i)
    {
        m_outFile.open(fileName+m_filterNames[i]+".obj");
        m_outFile << std::fixed;
        m_outFile << "# CityGML test export\n\n";
        m_outFile << "o " << fileName << "\n\n";
        std::string mtllib = fileName+m_filterNames[i];
        mtllib = mtllib.substr(mtllib.find_last_of('/')+1);
        m_outFile << "mtllib " << mtllib << ".mtl" << "\n\n";
        for(const CityObject* obj : objs)
            if(obj) exportCityObject(*obj, m_filters[i]);
        m_outFile.close();
        exportMaterials(fileName+m_filterNames[i]+".mtl");
    }

    #if 0
    // split bldg (wall, roof)
    for(const CityObject* obj : objs)
    {
        if(obj)
        {
            resetFilters();
            addFilter(COT_WallSurface, "Wall");
            m_outFile.open(fileName+'_'+obj->getId()+'_'+m_filterNames[0]+".obj");
            m_outFile << std::fixed;
            m_outFile << "# CityGML test export\n\n";
            m_outFile << "o " << fileName << "\n\n";
            std::string mtllib = fileName+'_'+obj->getId()+'_'+m_filterNames[0];
            mtllib = mtllib.substr(mtllib.find_last_of('/')+1);
            m_outFile << "mtllib " << mtllib << ".mtl" << "\n\n";
            exportCityObject(*obj, m_filters[0]);
            m_outFile.close();
            exportMaterials(fileName+'_'+obj->getId()+'_'+m_filterNames[0]+".mtl");

            resetFilters();
            addFilter(COT_RoofSurface, "Roof");
            m_outFile.open(fileName+'_'+obj->getId()+'_'+m_filterNames[0]+".obj");
            m_outFile << std::fixed;
            m_outFile << "# CityGML test export\n\n";
            m_outFile << "o " << fileName << "\n\n";
            mtllib = fileName+'_'+obj->getId()+'_'+m_filterNames[0];
            mtllib = mtllib.substr(mtllib.find_last_of('/')+1);
            m_outFile << "mtllib " << mtllib << ".mtl" << "\n\n";
            exportCityObject(*obj, m_filters[0]);
            m_outFile.close();
            exportMaterials(fileName+'_'+obj->getId()+'_'+m_filterNames[0]+".mtl");
        }
    }
    #endif
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityObject(const CityObject& obj, const std::string& fileName)
{
    if(m_filters.size() == 0)
    {
        addFilter(COT_All, "");
    }

    for(size_t i=0; i<m_filters.size(); ++i)
    {
        // create out file
        m_outFile.open(fileName+m_filterNames[i]+".obj");
        m_outFile << std::fixed;
        m_outFile << "# CityGML test export\n\n";
        m_outFile << "o " << obj.getId() << "\n\n";
        std::string mtllib = fileName+m_filterNames[i];
        mtllib = mtllib.substr(mtllib.find_last_of('/')+1);
        m_outFile << "mtllib " << mtllib << ".mtl" << "\n\n";
        exportCityObject(obj, m_filters[i]); // write geometry
        m_outFile.close();
        exportMaterials(fileName+m_filterNames[i]+".mtl"); // write materials
    }
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportCityObject(const CityObject& obj, citygml::CityObjectsType filter)
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
                    m_outFile << "\nusemtl " << mat << "\n";
                    m_materials[mat] = poly->getTexture()->getUrl(); // add material to map, will be used by exportMaterials
                }

                m_outFile << "g " << poly->getId() << "\n\n";

                for(const TVec3d& v : poly->getVertices())
                {
                    m_outFile << "v " << v.x-m_offsetX << " " << v.y-m_offsetY << " " << v.z << "\n";
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
                if(poly->getTexCoords().size() > 0) // if there is texture coords
                {
                    for(size_t i=0; i<poly->getIndices().size(); i+=3)
                    {
                        m_outFile << "f " << offset+poly->getIndices()[i+0] << "/" << offset+poly->getIndices()[i+0] << "/" << offset+poly->getIndices()[i+0] << " " <<
                                             offset+poly->getIndices()[i+1] << "/" << offset+poly->getIndices()[i+1] << "/" << offset+poly->getIndices()[i+1] << " " <<
                                             offset+poly->getIndices()[i+2] << "/" << offset+poly->getIndices()[i+2] << "/" << offset+poly->getIndices()[i+2] << "\n";
                    }
                }
                else // else skip texture coords
                {
                //*/
                    for(size_t i=0; i<poly->getIndices().size(); i+=3)
                    {
                        m_outFile << "f " << offset+poly->getIndices()[i+0] << "//" << offset+poly->getIndices()[i+0] << " " <<
                                             offset+poly->getIndices()[i+1] << "//" << offset+poly->getIndices()[i+1] << " " <<
                                             offset+poly->getIndices()[i+2] << "//" << offset+poly->getIndices()[i+2] << "\n";
                    }
                }
                //*/
                offset += poly->getVertices().size(); // update offset
            }
        }
    }

    for(citygml::CityObject* child : obj.getChildren())
    {
        exportCityObject(*child, filter);
    }
}
////////////////////////////////////////////////////////////////////////////////
void ExporterOBJ::exportMaterials(const std::string& filename)
{
    std::ofstream mat(filename);
    int i = 0;
    for(auto& it : m_materials)
    {
        ++i;
        mat << "newmtl " << it.first << "\n";
        mat << "Ka 1.000000 1.000000 1.000000\n";
        mat << "Kd 1.000000 1.000000 1.000000\n";
        mat << "Ks 0.000000 0.000000 0.000000\n";
        mat << "Tr 1.000000\n";
        mat << "illum 1\n";
        mat << "Ns 0.000000\n";
        mat << "map_Kd " << it.second << "\n\n";
    }
    if(i==0)//mat est vide, il faut cependant écrire dans le mtl sinon Assimp ne peut ouvrir le fichier
    {
        mat << "newmtl No Material \n";
        mat << "Ka 1.000000 1.000000 1.000000\n";
        mat << "Kd 1.000000 1.000000 1.000000\n";
        mat << "Ks 0.000000 0.000000 0.000000\n";
        mat << "Tr 1.000000\n";
        mat << "illum 1\n";
        mat << "Ns 0.000000\n";
        mat << "map_Kd No Material \n\n";
    }
    mat.close();
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
