// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __EXPORTOBJ_HPP__
#define __EXPORTOBJ_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "exporter.hpp"
#include "citygml.hpp"
#include "citygmlutils_export.h"

#ifdef WIN32
// Because of some "difficulties" a link time against the osgDB library (that
// seems to export some of its internal symbols, refer to
//  http://forum.openscenegraph.org/viewtopic.php?t=8099 )
// replace STL fstream with OSG fstream
#include <osgDB/fstream>
#else
#include <fstream>
#endif

////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterOBJ class
/// Export OBJ
class CITYGMLUTILS_EXPORT ExporterOBJ : public Exporter
{
public:
      ExporterOBJ();

      /// Add a filter on a type of object to produce an out file prepended with name containing only filter features
      /// \param filter Type to filter
      /// \param name Name to be prepended to output filename
      void addFilter(citygml::CityObjectsType filter, const std::string& name);

      /// Clear filters
      void resetFilters();

      /// \brief exportCityModel
      /// \param model
      /// \param fileName
      void exportCityObjects(const std::vector<CityObject*>& objs, const std::string& fileName);
      //void exportCityModel(const CityModel& model, const std::string& fileName);

      /// \brief exportCityObject
      /// \param model
      /// \param fileName
      void exportCityObject(const CityObject& obj, const std::string& fileName);

   private:
      void exportCityObject(const CityObject& obj, citygml::CityObjectsType filter=COT_All);
      void exportMaterials(const std::string& filename);

      std::ofstream m_outFile;
      //std::vector<TVec3d> m_vertices;
      //std::vector<TVec3f> m_normals;
      //std::vector<unsigned int> m_indices;
      //int m_indexOffset;

      //std::vector<TVec3d> m_verticesWall, m_verticesRoof, m_verticesTerrain;
      //std::vector<TVec3f> m_normalsWall, m_normalsRoof, m_normalsTerrain;
      //std::vector<unsigned int> m_indicesWall, m_indicesRoof, m_indicesTerrain;
      //int m_indexOffsetWall, m_indexOffsetRoof, m_indexOffsetTerrain;

      std::vector<citygml::CityObjectsType> m_filters;    ///< filters list
      std::vector<std::string> m_filterNames;             ///< filters name
      std::map<citygml::CityObjectsType, int> m_filterOffsets; ///< used to handle indices offset
      std::map<std::string, std::string> m_materials;
   };
   ////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __EXPORTOBJ_HPP__
