// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __EXPORTOBJ_HPP__
#define __EXPORTOBJ_HPP__

#include <fstream>
#include "libcitygml/export/exporter.hpp"
#include "libcitygml/citygml.hpp"
#include "citygmlutils_export.h"

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
      // FIXME: std::vector<TVec3d> m_vertices;
      // FIXME: std::vector<TVec3f> m_normals;
      // FIXME: std::vector<unsigned int> m_indices;
      // FIXME: int m_indexOffset;

      // FIXME: std::vector<TVec3d> m_verticesWall, m_verticesRoof, m_verticesTerrain;
      // FIXME: std::vector<TVec3f> m_normalsWall, m_normalsRoof, m_normalsTerrain;
      // FIXME: std::vector<unsigned int> m_indicesWall, m_indicesRoof, m_indicesTerrain;
      // FIXME: int m_indexOffsetWall, m_indexOffsetRoof, m_indexOffsetTerrain;

      std::vector<citygml::CityObjectsType> m_filters;    ///< filters list
      std::vector<std::string> m_filterNames;             ///< filters name
      std::map<citygml::CityObjectsType, size_t> m_filterOffsets; ///< used to handle indices offset
      std::map<std::string, std::string> m_materials;
   };
   ////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __EXPORTOBJ_HPP__
