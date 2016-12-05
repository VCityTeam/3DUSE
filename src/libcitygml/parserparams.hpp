/* -*-c++-*- libcitygml - Copyright (c) 2010 Joachim Pouderoux, BRGM
*
* This file is part of libcitygml library
* http://code.google.com/p/libcitygml
*
* libcitygml is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 2.1 of the License, or
* (at your option) any later version.
*
* libcitygml is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*/
////////////////////////////////////////////////////////////////////////////////
#ifndef __CITYGML_PARSERPARAMS_HPP__
#define __CITYGML_PARSERPARAMS_HPP__

#include <string>
#include "citygml_export.h"
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
#pragma warning(disable: 4251) // export problem on STL members
#endif

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
// m_basePath : base path used to find textures
class CITYGML_EXPORT ParserParams
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
    bool temporalImport;
    std::string destSRS;
    std::string m_basePath;
};

} // namespace citygml

#endif // __CITYGML_PARSERPARAMS_HPP__
