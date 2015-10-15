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
