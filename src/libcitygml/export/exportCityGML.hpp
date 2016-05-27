// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __CITYGML_EXPORT_HPP_
#define __CITYGML_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "exporter.hpp"
#include <libxml/tree.h>
#include "citygml_export.h"
#include "citymodel.hpp"
////////////////////////////////////////////////////////////////////////////////
struct TexturePolygonCityGML {
   std::vector<TVec2f> TexUV;
   std::string Id;
   std::string IdRing;
};
struct TextureCityGML {
   std::string Url;
   citygml::Texture::WrapMode Wrap;
   std::vector<TexturePolygonCityGML> ListPolygons;
};
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
   ////////////////////////////////////////////////////////////////////////////////
   /// \brief The Exporter class
   /// Export citygml
   ///
   /// Can be used with one step by writing a citymodel or an array of cityobjects
   /// or can be used by appending cityobjects
   ///
   /// Direct example :
   /// \code{.cpp}
   /// CityModel* model = ...
   /// ...
   /// citygml::ExporterCityGML exporter("file.gml");
   /// exporter.exportCityModel(*model);
   /// \endcode
   ///
   /// Incremental example :
   /// \code{.cpp}
   /// CityModel* model = ...
   /// ...
   /// citygml::ExporterCityGML exporter("file.gml");
   /// exporter.initExport();
   /// ...
   /// for(citygml::CityObject* Building : model->getCityObjectsRoots())
   /// {
   ///     ...
   ///     exporter.appendCityObject(*Building);
   ///     ...
   /// }
   /// ...
   /// exporter.endExport();
   /// \endcode
   class CITYGML_EXPORT ExporterCityGML : public Exporter
   {
   public:
      ExporterCityGML(const std::string& filename);
      virtual ~ExporterCityGML() override;

      /// Used for incremental export
      void initExport(bool createCityModelRootNode=true);

      /// Used to finish incremental export
      void endExport();

      /// Used to write envelope in incremental export
      void addEnvelope(const citygml::Envelope& env);

      /// \brief exportCityModel Export a complete CityModel
      /// \param model Model
      void exportCityModel(const CityModel& model);

      /// \brief exportCityModel Export a complete CityModel, with textures stored in ListTextures
      /// \param model Model
      /// \param ListTextures Textures List
      void exportCityModelWithListTextures(const CityModel& model, std::vector<TextureCityGML*>* ListTextures);

      /// \brief exportCityObject Export an array of CityObjects
      /// \param objs CityObjects
      void exportCityObject(const std::vector<const CityObject*>& objs);

      /// \brief exportCityObject Export an array of CityObjects, with textures stored in ListTextures
      /// \param objs CityObjects
      /// \param ListTextures Textures List
      void exportCityObjectWithListTextures(const std::vector<const CityObject*>& models, std::vector<TextureCityGML*>* ListTextures);

      /// Append an array of CityObjects (incremental export)
      void appendCityObject(const std::vector<const CityObject*>& objs);

      /// Append a CityObject (incremental export)
      void appendCityObject(const CityObject& obj);


   private:
      xmlNodePtr exportCityObjectModelXml(const std::vector<const CityObject*>& objs);
      xmlNodePtr exportCityModelXml(const citygml::CityModel& model);
      xmlNodePtr exportListTextures(xmlNodePtr root, std::vector<TextureCityGML*>* ListTextures);
      xmlNodePtr exportEnvelopeXml(const citygml::Envelope& env, xmlNodePtr parent);
      xmlNodePtr exportLinearRingXml(const citygml::LinearRing& ring, xmlNodePtr parent);
      xmlNodePtr exportPolygonXml(const citygml::Polygon& poly, xmlNodePtr parent);
      xmlNodePtr exportGeometryGenericXml(const citygml::Geometry& geom, const std::string& nodeType, xmlNodePtr parent);
      xmlNodePtr exportGeometryXml(const citygml::Geometry& geom, xmlNodePtr parent);
      xmlNodePtr exportCityObjetStateXml(const citygml::CityObjectState& state, const std::string &nodeType, xmlNodePtr parent);
      xmlNodePtr exportCityObjetTagXml(const citygml::CityObjectTag& tag, const std::string &nodeType, xmlNodePtr parent);
      xmlNodePtr exportCityObjetGenericXml(const citygml::CityObject& obj, const std::string &nodeType, xmlNodePtr parent, bool isSurface=false);
      xmlNodePtr exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent, bool rootLevel=false);
      xmlNodePtr exportPolygonAppearanceXml(const citygml::Polygon& poly, xmlNodePtr parent);

      std::string m_fileName;             ///< Out file name
      xmlDocPtr m_doc;                    ///< XML doc
      xmlNodePtr m_root_node;             ///w XML root node
      xmlNodePtr m_currentAppearence;     ///< current appearence node (used to group materials by Building)
   };
   ////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_EXPORT_HPP_
