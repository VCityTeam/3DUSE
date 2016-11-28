// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014

////////////////////////////////////////////////////////////////////////////////

#include "exportJSON.hpp"
#include <sstream>

////////////////////////////////////////////////////////////////////////////////

namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterJSON::ExporterJSON()
   : m_indentDepth(0), m_genTexCoords(false), m_WorldTexCoords(false), m_offsetX(0.0), m_offsetY(0.0), m_tileSizeX(0.0), m_tileSizeY(0.0)
{
}

////////////////////////////////////////////////////////////////////////////////

void ExporterJSON::setPath(const std::string& Path)
{
   m_Path = Path;
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

   std::cout << "ID  = " << id << std::endl;
   std::cout << "Filename = " << fileName << std::endl;

   // bldg

#if 0 //1 : ce if est actif, les 0 sont inactifs
   std::cout << "bldg \n";
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

   for (CityObject* obj : model.getCityObjectsRoots())
      if (obj && obj->getType() == COT_Building) exportCityObject(*obj);

   if (getNbFeature(model, COT_Building) > 0)
      m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";

   closeScope();  // listBldg scope
   closeScope(); // global scope

   m_outFile.close();
   resetFilters();
#endif

   // terrain

#if 0
   std::cout << "terrain \n";
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

   for (CityObject* obj : model.getCityObjectsRoots())
      if (obj && obj->getType() == COT_TINRelief) exportCityObject(*obj);

   if (getNbFeature(model, COT_TINRelief) > 0)
      m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";

   closeScope();  // listTerrain scope
   closeScope(); // global scope
   m_outFile.close();
   resetFilters();
#endif

   // terrain avec fichiers world .jgw : TERRAIN POUR DONNEES DE LYON
#if 1 //0
   std::cout << "terrain \n";
   m_WorldTexCoords = true;
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

   for (CityObject* obj : model.getCityObjectsRoots())
      if (obj && obj->getType() == COT_TINRelief) exportCityObject(*obj);

   if (getNbFeature(model, COT_TINRelief) > 0)
      m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";

   closeScope();  // listTerrain scope
   closeScope(); // global scope
   m_outFile.close();
   resetFilters();
#endif

   // terrain geo ref test

#if 0 //Terrain avec m_genTexCoords = true //Utile pour buildJSON lod ?
   std::cout << "terrain geo ref test \n";
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

   for (CityObject* obj : model.getCityObjectsRoots())
      if (obj && obj->getType() == COT_TINRelief) exportCityObject(*obj);

   if (getNbFeature(model, COT_TINRelief) > 0)
      m_outFile.seekp(-2, std::ios_base::cur); m_outFile << "\n";

   closeScope();  // listTerrain scope
   closeScope(); // global scope
   m_outFile.close();
   resetFilters();
#endif

   // bldg geo ref test

#if 0 //Building avec m_genTexCoords = true
   std::cout << "bldg geo ref test \n";
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

   for (CityObject* obj : model.getCityObjectsRoots())
      if (obj && obj->getType() == COT_Building) exportCityObject(*obj);

   if (getNbFeature(model, COT_Building) > 0)
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

   for (const auto& it : m_filters)
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

   size_t nb = obj.getAttributes().size();

   size_t i = 0;

   for (const auto& attr : obj.getAttributes())
   {
      indent(); m_outFile << "\"" << attr.first << "\":\"" << attr.second << "\"";
      if (++i != nb)
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
   if (obj.getType() == type)
   {
      std::string texture;

      for (Geometry* geom : obj.getGeometries())
      {
         for (Polygon* poly : geom->getPolygons())
         {
            if (poly->getTexture())
            {
               if (!texture.empty() && texture != poly->getTexture()->getUrl())
                  std::cout << "Multiple textures !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

               texture = poly->getTexture()->getUrl();
            }
         }
      }

      if (!texture.empty() || m_genTexCoords)
      {
         std::vector<TVec2f> genTexCoords;

         if (m_needComma)
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

         double A, B, C, D; //Voir fr.wikipedia.org/wiki/World_file : Taille pixel, rotation, retournement
         double offset_x;
         double offset_y;

         if (m_WorldTexCoords) //Ouvrir le fichier .jgw
         {
            std::ifstream fichier(m_Path.substr(0, m_Path.find_last_of("/")) + "/" + texture.substr(0, texture.find_last_of('.')) + ".jgw", std::ios::in);

            //std::cout << "Open World File for texture : " << texture.substr(0, texture.find_last_of('.'))+".jgw" << std::endl;

            if (fichier)
            {
               fichier >> A >> B >> C >> D >> offset_x >> offset_y;
               fichier.close();
            }
            //std::cout << A << " " << B << " " << C << " " << D << " " << offset_x << " " << offset_y << std::endl;
         }

         indent(); m_outFile << "\"listGeometries\":[ ";

         for (Geometry* geom : obj.getGeometries())
         {
            for (Polygon* poly : geom->getPolygons())
            {
               for (const auto& vertex : poly->getVertices())
               {
                  m_outFile << vertex.x << "," << vertex.y << "," << vertex.z << ",";

                  if (m_genTexCoords)
                  {
                     //compute tex coords

                     TVec2f tc;
                     tc.x = (vertex.x - m_offsetX) / m_tileSizeX;
                     tc.y = (vertex.y - m_offsetY) / m_tileSizeY;
                     tc.y = 1.0f - tc.y;

                     genTexCoords.push_back(tc);
                  }
               }
            }
         }

         m_outFile.seekp(-1, std::ios_base::cur);
         m_outFile << " ],\n";

         indent(); m_outFile << "\"listNormals\":[ ";

         for (Geometry* geom : obj.getGeometries())
         {
            for (Polygon* poly : geom->getPolygons())
            {
               for (const auto& normal : poly->getNormals())
               {
                  m_outFile << normal.x << "," << normal.y << "," << normal.z << ",";
               }
            }
         }

         m_outFile.seekp(-1, std::ios_base::cur);
         m_outFile << " ],\n";

         indent(); m_outFile << "\"listUVs\":[ ";

         if (!m_genTexCoords)
         {
            for (Geometry* geom : obj.getGeometries())
            {
               for (Polygon* poly : geom->getPolygons())
               {
                  /*if(poly->getTexture())
                  {
                      if(!texture.empty() && texture != poly->getTexture()->getUrl())
                          std::cout << "Multiple textures !" << std::endl;

                      texture = poly->getTexture()->getUrl();
                  }*/

                  if (m_WorldTexCoords) //Generer les UV a partir du World File
                  {
                     for (const auto& vertex : poly->getVertices())
                     {
                        //compute tex coords

                        TVec2d tc;
                        tc.x = (vertex.x - offset_x) / 409.5; // ATTENTION : fonctionne uniquement pour textures 4096x4096 avec ratio de 0.1 (c'est le cas sur Lyon)
                        tc.y = (vertex.y - offset_y) / 409.5;
                        tc.y = 1 + tc.y; //Car D est negatif

                        m_outFile << tc.x << "," << tc.y << ",";
                     }
                  }
                  else
                  {
                     for (const auto& uv : poly->getTexCoords())
                     {
                        m_outFile << uv.x << "," << uv.y << ",";
                     }
                  }
               }
            }
         }
         else
         {
            for (const auto& uv : genTexCoords)
            {
               m_outFile << uv.x << "," << uv.y << ",";
            }
         }

         m_outFile.seekp(-1, std::ios_base::cur);
         m_outFile << " ],\n";

         size_t offset = 0;
         indent(); m_outFile << "\"listIndices\":[ ";

         for (Geometry* geom : obj.getGeometries())
         {
            for (Polygon* poly : geom->getPolygons())
            {
               for (auto index : poly->getIndices())
               {
                  m_outFile << offset + index << ",";
               }

               offset += poly->getVertices().size();
            }
         }
         m_outFile.seekp(-1, std::ios_base::cur);
         m_outFile << " ],\n";

         indent();

         if (!m_genTexCoords)
         {
            //m_outFile << "\"texture\":\"" << "EXPORT_" << m_id.substr(0,4) << "-" <<  m_id.substr(5,5) << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";
            m_outFile << "\"texture\":\"" << "output" << m_id << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";

            //if(!texture.empty())
            //    m_outFile << "\"texture\":\"" << "EXPORT_" << m_id.substr(0,4) << "-" <<  m_id.substr(5,5) << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";
            //else
            //    m_outFile << "\"texture\":\"\"\n";

            //closeScope(); // feature scope
         }
         else
         {
            //m_outFile << "\"texture\":\"" << "output" << m_id << "/" << texture.substr(0, texture.find_last_of('.')) << "\"\n";

            m_outFile << "\"texture\":\"" << "tiles/tile_" << m_id << "\"\n"; //BuildJSONLOD ???????
         }
         --m_indentDepth; indent(); m_outFile << "}";
      }
   }

   for (CityObject* child : obj.getChildren())
   {
      if (child) exportFeature(*child, type);
   }
}

////////////////////////////////////////////////////////////////////////////////

int ExporterJSON::getNbFeature(CityModel& model, CityObjectsType type) const
{
   int nb = 0;

   for (CityObject* obj : model.getCityObjectsRoots())
   {
      if (obj && obj->getType() == type)
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

   if (obj.getType() == type)
   {
      ++nb;
   }

   for (citygml::CityObject* child : obj.getChildren())
   {
      nb += getNbFaces(*child, type);
   }
   return nb;
}

////////////////////////////////////////////////////////////////////////////////

size_t ExporterJSON::getNbTris(CityObject& obj) const
{
   size_t nb = 0;

   for (Geometry* geom : obj.getGeometries())
   {
      for (Polygon* poly : geom->getPolygons())
      {
         nb += poly->getIndices().size() / 3;
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
   if (comma)
      m_outFile << "},\n";
   else
      m_outFile << "}\n";
}

////////////////////////////////////////////////////////////////////////////////

void ExporterJSON::indent()
{
   for (int i = 0; i < m_indentDepth; ++i)
   {
      m_outFile << "    ";
   }
}
////////////////////////////////////////////////////////////////////////////////

} // namespace citygml

////////////////////////////////////////////////////////////////////////////////
