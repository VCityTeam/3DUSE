#ifndef __CITYGML_PARSERPARAMS_HPP__
#define __CITYGML_PARSERPARAMS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
// Parsing routines
// Parameters:
// objectsMask: a string describing the objects types that must or must not be parsed
//    examples: "All&~LandUse&~TINRelief" to parse everything but landuses and TIN reliefs
//              "Road&Railway" to parse only roads & railways
// minLOD: the minimal LOD that will be parsed
// maxLOD: the maximal LOD that will be parsed
// optimize: merge geometries & polygons that share the same appearance in the same object in order to reduce the global hierarchy
// pruneEmptyObjects: remove the objects which do not contains any geometrical entity
// tesselate: convert the interior & exteriors polygons to triangles
// destSRS: the SRS (WKT, EPSG, OGC URN, etc.) where the coordinates must be transformed, default ("") is no transformation
class ParserParams
{
public:
    ParserParams( void );

public:
    std::string objectsMask;
    unsigned int minLOD;
    unsigned int maxLOD;
    bool optimize;
    bool pruneEmptyObjects;
    bool tesselate;
    std::string destSRS;
    std::string m_basePath;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_PARSERPARAMS_HPP__
