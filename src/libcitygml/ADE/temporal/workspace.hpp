// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef _WORKSPACE_HPP_
#define _WORKSPACE_HPP_

#include "version.hpp"

namespace temporal {
class Workspace {
public:
	Workspace(){};
	std::vector<Version*> versions;
	std::string name;
};
}
#endif
