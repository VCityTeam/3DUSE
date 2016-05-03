#ifndef _DOCUMENTHANDLER_HPP_
#define _DOCUMENTHANDLER_HPP_

#include "../ADE.hpp"
#include "documentObject.hpp"
#include "reference.hpp"
#include "tag.hpp"

class DocumentHandler : public ADEHandler
{
protected :
    documentADE::DocumentObject* _currentDocument;
    documentADE::Reference* _currentReference;
    documentADE::Tag* _currentTag;

public:
    DocumentHandler(void);
    DocumentHandler(citygml::CityGMLHandler* gmlHandler);

    std::string getAttribute( void* attributes, const std::string& attname, const std::string& defvalue );
    std::string removeNamespace(std::string name);
    std::string getIDfromQuery(std::string query);

    void startElement(std::string name, void* attributes);
    void endElement(std::string);
    void endDocument();
};

#endif
