#ifndef _TEMPORALHANDLER_HPP_
#define _TEMPORALHANDLER_HPP_

#include "ADE.hpp"

class TempHandler : public ADEHandler
{
public:
	TempHandler(void);
	TempHandler(citygml::CityGMLHandler* gmlHandler);
	void endElement(std::string);
private:
	//Adding to ADE register (template in ADE.hpp)
	static ADERegister<TempHandler> reg;	
};

#endif