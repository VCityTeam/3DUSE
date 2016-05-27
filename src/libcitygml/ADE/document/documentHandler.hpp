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
    std::vector<documentADE::DocumentObject*> _documents;
    std::vector<documentADE::Reference*> _references;
private:
    void setAttributeValue(std::string name);
    void setDocumentAttributeValue(std::string name);

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
