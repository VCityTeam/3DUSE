// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGGDAL_HPP__
#define __OSGGDAL_HPP__

#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>
#include <osg/Geode>

osg::ref_ptr<osg::Geode> buildOsgGDAL(OGRDataSource* poDS);

#endif // __OSGGDAL_HPP__
