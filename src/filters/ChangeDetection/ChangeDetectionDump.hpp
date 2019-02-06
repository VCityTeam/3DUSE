// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef __CHANGEDETECTIONDUMP_HPP__
#define __CHANGEDETECTIONDUMP_HPP__

#include "filters_export.h"
#include "ChangeDetection.hpp"
#include <boost/property_tree/ptree.hpp>

FILTERS_EXPORT void DumpIDCorrespondancesJson(ChangeDetectionRes change,
                                              int time_stamp1,
                                              int time_stamp2,
                                              std::string filename,
                                              boost::property_tree::ptree& comment);

FILTERS_EXPORT void DumpIDCorrespondancesDebug(ChangeDetectionRes change,
                                               int time_stamp1,
                                               int time_stamp2);

#endif // __CHANGEDETECTIONDUMP_HPP__
