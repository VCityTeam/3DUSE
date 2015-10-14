#ifndef _WORKSPACE_HPP_
#define _WORKSPACE_HPP_

#include "Version.hpp"

namespace temporal {
class Workspace {
public:
	Workspace(){};
	std::vector<Version*> versions;
	std::string name;
};
}
#endif
