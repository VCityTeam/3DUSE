#ifndef _DOCUMENTHANDLER_HPP_
#define _DOCUMENTHANDLER_HPP_

#include "../ADE.hpp"


class DocumentHandler : public ADEHandler
{
protected :

public:
    DocumentHandler(void);
    DocumentHandler(citygml::CityGMLHandler* gmlHandler);
    void startElement(std::string, void*);
    void endElement(std::string);
    void endDocument();
};

#endif
