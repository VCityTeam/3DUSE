// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __JSON_EXPORT_HPP_
#define __JSON_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include "exporter.hpp"
#include "citygml.hpp"
//#include <fstream> // MT 05/05/2014
#include <osgDB/fstream>
#include <map>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterJSON class
/// Export JSON, mostly for webgl project, using our json format
class ExporterJSON : public Exporter
{
public:
    ExporterJSON();

    /// Set exporter base path
    void setBasePath(const std::string& basePath);

    void setOffset(double offsetX, double offsetY);
    void setTileSize(double tileSizeX, double tileSizeY);

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel& model, const std::string& fileName, const std::string& id);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject& obj);

    /// Add a filter on a type of object to produce an out file prepended with name containing only filter features
    /// \param filter Type to filter
    /// \param name Name to be prepended to output filename
    void addFilter(CityObjectsType filter, const std::string& name);

    /// Clear filters
    void resetFilters();

private:
    void exportFeature(CityObject& obj, CityObjectsType type);
    int getNbFeature(CityModel& model, CityObjectsType type) const;
    int getNbFaces(CityObject& obj, CityObjectsType type) const;
    int getNbTris(CityObject& obj) const;

    // helper function for indent
    void openScope();
    void closeScope(bool comma = false);
    void indent();

    std::string m_id;
    std::string m_basePath;
    std::string m_fileName;
    std::ofstream m_outFile;
    int m_indentDepth;

    bool m_genTexCoords;            ///< Activate texture coord generation (like GeoRefTexture) ?
    double m_offsetX, m_offsetY;
    double m_tileSizeX, m_tileSizeY;

    std::map<CityObjectsType, std::string> m_filters;   ///< filters list

    bool m_needComma;               ///< flag used to handle commas
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __JSON_EXPORT_HPP_
