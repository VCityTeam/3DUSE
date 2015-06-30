#include "temporalHandler.hpp"
#include "utils.hpp"

TempHandler::TempHandler(void):ADEHandler()
{
}

TempHandler::TempHandler(citygml::CityGMLHandler* gHandler):ADEHandler(gHandler)
{
}

//Adding to ADE register (template in ADE.hpp)
ADERegister<TempHandler> TempHandler::reg("tmp");

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parsing routines

void TempHandler::endElement(std::string name)
{
	std::stringstream buffer;
	buffer << trim( getBuff()->str() );
	if (name=="tmp:creationDate")
	{
		citygml::CityObjectTag** currentTag = getCurrentTag();
		citygml::ParserParams* params = getParams();
		if(params->temporalImport && (*currentTag))
		{
			(*currentTag)->m_date = QDateTime::fromString(buffer.str().c_str(), Qt::ISODate);
			(*currentTag)->m_parent->checkTags();
		}
	}
}
