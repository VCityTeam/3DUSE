// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __CHANGEDETECTION_HPP__
#define __CHANGEDETECTION_HPP__

#include <stdlib.h>
#ifdef _MSC_VER                // Inhibit dll-interface warnings concerning
# pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                         // including ogrsf_frmts.h on VCC++
#include <ogrsf_frmts.h>
#include "libcitygml/citygml.hpp"
#include "filters_export.h"

// Ad-hoc structure used to return RESult of change detection function
struct ChangeDetectionRes {

    typedef std::vector<std::string> CityGMLidAsStringVector;

    // Assume the buildings are ordered and thus an integer acts as
    // identifier. The following type can be used to describe a 
    // between a building (say at time period t1) and one or many buildings
    // (at time period t2) and thus encodes a correspondence between buildings
    typedef std::vector<std::vector<int> > BuildingCorrespondence;
    
    // Now assume that a building doesn't exist at time period t1 but gets
    // created at time period t2. Such building cannot appear in a
    // BuildingCorrespondence encoding the correspondence from past to future.
    // Such building can only exist in a correspondence timely oriented
    // backwards i.e. from future to past. Hence in order to completely
    // describe a correspondence between t1 and t2 one needs two
    // BuildingCorrespondence intstances working together:
    typedef std::pair< BuildingCorrespondence,
                       BuildingCorrespondence > BuildingFullCorrespondence;

    // Polygons representing the building footprints of first CityModel
    OGRMultiPolygon* EnveloppeCityU1;

    // Ditto for second CityModel
    OGRMultiPolygon* EnveloppeCityU2;

    // Footprints of destroyed buildings
    OGRMultiPolygon* BatiDetruits; 

    // Footprints of constructed buildings
    OGRMultiPolygon* BatiCrees;

    // Footprints of modified buildings of the first CityModel
    OGRMultiPolygon* BatiModifies1; 

    // Ditto for second Model
    OGRMultiPolygon* BatiModifies2;

    //Footprints of unchanged buildings
    OGRMultiPolygon* BatiInchanges;

    // CityGML identifiers (as obtained by cityGML::object.getId() method
    // and represented as std::string) of the (extracted buildings) of the
    // first model. 
    CityGMLidAsStringVector* BuildingID1;

    // Ditto but for the second model
    CityGMLidAsStringVector* BuildingID2;

    // The two way correspondance between the buildings of set 1 and the
    // buildings of set 2:
    BuildingFullCorrespondence* Compare;
};

FILTERS_EXPORT ChangeDetectionRes CompareTiles(
  std::string Folder,
  citygml::CityModel *City1,
  citygml::CityModel *City2
);

FILTERS_EXPORT void DumpIDCorrespondances( ChangeDetectionRes, int, int);

#endif // __CHANGEDETECTION_HPP__
