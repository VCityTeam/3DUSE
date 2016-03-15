#include "documentHandler.hpp"
#include "utils.hpp"
#include "citygml.hpp"

#include <libxml/parser.h>
#include <libxml/SAX.h>
#include <libxml/xlink.h>
#include <libxml/xpath.h>

DocumentHandler::DocumentHandler(void):ADEHandler()
{

}

DocumentHandler::DocumentHandler(citygml::CityGMLHandler* gHandler):ADEHandler(gHandler)
{

}

void DocumentHandler::startElement(std::string,void*)
{

}
void DocumentHandler::endElement(std::string)
{

}

void DocumentHandler::endDocument()
{

}
