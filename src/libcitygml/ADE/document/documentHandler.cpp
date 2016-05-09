#include "documentHandler.hpp"
#include "utils.hpp"
#include "citygml.hpp"

#include <libxml/parser.h>
#include <libxml/SAX.h>
#include <libxml/xlink.h>
#include <libxml/xpath.h>
#include "../../object.hpp"

DocumentHandler::DocumentHandler(void):ADEHandler()
{

}

DocumentHandler::DocumentHandler(citygml::CityGMLHandler* gHandler):ADEHandler(gHandler)
{

}

std::string DocumentHandler::getAttribute( void* attributes, const std::string& attname, const std::string& defvalue = "" )
{
    const xmlChar **attrs = (const xmlChar**)attributes;
    if ( !attrs ) return defvalue;
    for ( int i = 0; attrs[i] != 0; i += 2 ) {
        std::cout << "attr val: " << attrs[i] <<std::endl;
        if ( (const char*)( attrs[i] ) == attname ) return (const char*)( attrs[ i + 1 ] );
    }
    return defvalue;
}

std::string DocumentHandler::removeNamespace(std::string name)
{
    size_t pos = name.find_first_of( ":" );
    return name.substr( pos + 1 );
}

std::string DocumentHandler::getIDfromQuery(std::string query)
{
    size_t pos1 = query.find("//*[@id='");
    size_t pos2 = query.find("']",pos1);
    if (pos1!=std::string::npos && pos2!=std::string::npos)
    {
        return query.substr(pos1+9,pos2-(pos1+9));
    }
    if (query.find("#")==0)
    {
        return query.substr(1);
    }
    return "";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parsing routines

void DocumentHandler::startElement(std::string name, void* attributes)
{

    name = removeNamespace(name);

    if (name == "documentobject")
    {
       _currentDocument = new documentADE::DocumentObject(getGmlIdAttribute( attributes ));
        std::cout << "document object: " <<_currentDocument->getId()<< std::endl;
        citygml::CityModel** model = getModel();
        citygml::CityObject* cityObject = *getCurrentCityObject();
        (*model)->addCityObjectAsRoot(cityObject);
        _documents.push_back(_currentDocument);
    }
    else if (name == "reference")
    {
        _currentReference = new documentADE::Reference(getGmlIdAttribute( attributes ));
        std::cout << "reference: " <<_currentReference->getId()<< std::endl;
    }
    else if (name == "tag")
    {
       _currentTag = new documentADE::Tag(getGmlIdAttribute( attributes ));
        std::cout << "tag: " <<_currentTag->getId()<< std::endl;
    }
}
/******************************************************/
void DocumentHandler::endElement(std::string name)
{

    name = removeNamespace(name);

    if (name == "documentobject")
    {
    }
    else if (name == "reference")
    {
    }
}
/******************************************************/
void DocumentHandler::endDocument()
{
        citygml::CityModel** model = getModel();
        (*model)->setDocuments(_documents);
}
