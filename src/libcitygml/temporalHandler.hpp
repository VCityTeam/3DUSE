#ifndef _TEMPORALHANDLER_HPP_
#define _TEMPORALHANDLER_HPP_

#include "ADE.hpp"

class TempHandler : public ADEHandler
{
public:
	TempHandler(citygml::CityGMLHandler* gmlHandler);
	void endElement(std::string);
protected:
	
};

#endif