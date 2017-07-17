// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef CITYGMLCUT_H
#define CITYGMLCUT_H

#include "vecs.hpp"

#include <libxml/tree.h>

#include <map>
#include <set>

struct FOUR_PLANES
{
	TVec3d n[4];
	TVec3d p0[4];
};

typedef void (*fct_process_All_textureCoordinates)(xmlNodePtr, std::map<std::string, xmlNodePtr> *);
typedef xmlNodePtr (*fct_process_Building_ReliefFeature_textureCoordinates)(xmlNodePtr, xmlNodePtr);
typedef void (*fct_process_Building_ReliefFeature_boundingbox)(xmlNodePtr, bool *, double *, double *, double *, double *, double *, double *, std::set<std::string> *, xmlNodePtr, std::map<std::string, xmlNodePtr> *);
typedef void (*fct_process_Building_ReliefFeature_textures)(xmlNodePtr, std::set<std::string> *, std::string, std::string);

class CityGMLCut
{
public:
	int Run(char *xml_file_in, char *xml_file_out, double xmin, double ymin, double xmax, double ymax, bool verbose);

private:
};

#endif // CITYGMLCUT_H
