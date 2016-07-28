// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014, 2015
////////////////////////////////////////////////////////////////////////////////
#ifndef __OGRGDALTOSHPWRITER_HPP__
#define __OGRGDALTOSHPWRITER_HPP__

#include <string>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>
#include "vcitycore_export.h"

VCITYCORE_EXPORT void SaveGeometrytoShape( std::string name,
                                           const OGRGeometryCollection* G );

VCITYCORE_EXPORT void SaveGeometrytoShape( std::string name,
                                           const OGRGeometry* G );

#endif // __OGRGDALTOSHPWRITER_HPP__
