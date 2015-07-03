#ifndef _TEMPORALHANDLER_HPP_
#define _TEMPORALHANDLER_HPP_

#include "ADE.hpp"

class TempHandler : public ADEHandler
{
public:
	TempHandler(void);
	TempHandler(citygml::CityGMLHandler* gmlHandler);
	void startElement(std::string, void*);
	void endElement(std::string);
protected:
	std::string getAttribute( void*, const std::string&, const std::string&);
//	std::string getGmlIdAttribute( void* attributes ) { return getAttribute( attributes, "gml:id", "" ); }
private:
	//Adding to ADE register (template in ADE.hpp)
	static ADERegister<TempHandler> reg;	
};

#endif