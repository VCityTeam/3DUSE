#include "temporalHandler.hpp"
#include "utils.hpp"
#include "citygml.hpp"

#include <libxml/parser.h>
#include <libxml/SAX.h>
#include <libxml/xlink.h>
#include <libxml/xpath.h>

TempHandler::TempHandler(void):ADEHandler()
{
	_currentTransaction=nullptr;
	_currentTransition=nullptr;
	_currentVersion=nullptr;
	_inFromTags=false;
	_inToTags=false;
}

TempHandler::TempHandler(citygml::CityGMLHandler* gHandler):ADEHandler(gHandler)
{
	_currentTransaction=nullptr;
	_currentTransition=nullptr;
	_currentVersion=nullptr;
	_inFromTags=false;
	_inToTags=false;
}

//Adding to ADE register (template in ADE.hpp)
ADERegister<TempHandler> TempHandler::reg("temp");

std::string TempHandler::getAttribute( void* attributes, const std::string& attname, const std::string& defvalue = "" )
{
	const xmlChar **attrs = (const xmlChar**)attributes;
	if ( !attrs ) return defvalue;
	for ( int i = 0; attrs[i] != 0; i += 2 ) 
		if ( (const char*)( attrs[i] ) == attname ) return (const char*)( attrs[ i + 1 ] );
	return defvalue;
}

std::string TempHandler::removeNamespace(std::string name)
{
	size_t pos = name.find_first_of( ":" );
	return name.substr( pos + 1 );
}

std::string TempHandler::getIDfromQuery(std::string query)
{
	size_t pos1 = query.find("//*[@id='");
	size_t pos2 = query.find("']",pos1);
	if (pos1!=std::string::npos && pos2!=std::string::npos)
	{
		return query.substr(pos1+9,pos2-(pos1+9));
	}
	throw "XLinkExpressionException";
	return "";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parsing routines

void TempHandler::startElement(std::string name, void* attributes)
{

	name = removeNamespace(name);

	if (name=="Version")
	{
		_currentVersion = new temporal::Version(getGmlIdAttribute( attributes ));
		if (getAttribute(attributes,"xlink:href","") != "")
		{
			_currentVersion->_isXlink = citygml::xLinkState::UNLINKED;
			_currentVersion->setAttribute( "xlink", getAttribute(attributes,"xlink:href",""), false );
		}
		else _versions.push_back(_currentVersion);
		if(_inFromTags) _currentTransition->setFrom(_currentVersion);
		if(_inToTags) _currentTransition->setTo(_currentVersion);
	}
	if (name == "versionMember")
	{
		pushCityObject( new citygml::GenericCityObject( getGmlIdAttribute( attributes ) ) );
		pushObject( *getCurrentCityObject() );
	}
	if (name == "VersionTransition")
	{
		_currentTransition = new temporal::VersionTransition(getGmlIdAttribute( attributes ));
		_transitions.push_back(_currentTransition);
	}
	if (name=="from")
	{
		_inFromTags = true;
	}
	if (name=="to")
	{
		_inToTags = true;
	}
	if (name=="Transaction")
	{
		temporal::Transaction* nTransaction = new temporal::Transaction(getGmlIdAttribute( attributes ));
		_currentTransaction = nTransaction;
	}
	if (name == "oldFeature")
	{
		pushCityObject( new citygml::GenericCityObject( getGmlIdAttribute( attributes ) ) );
		pushObject( *getCurrentCityObject() );
	}
	if (name == "newFeature")
	{
		pushCityObject( new citygml::GenericCityObject( getGmlIdAttribute( attributes ) ) );
		pushObject( *getCurrentCityObject() );
	}
}
/******************************************************/
void TempHandler::endElement(std::string name)
{

	name = removeNamespace(name);

	if (name=="validFrom")
	{
		citygml::Object** currentObject = getCurrentObject();
		std::stringstream buffer;
		buffer << trim(getBuff()->str());
		if ( *currentObject ) (*currentObject)->setAttribute( "validFrom", buffer.str(), false );
	}
	if (name=="validTo")
	{
		citygml::Object** currentObject = getCurrentObject();
		std::stringstream buffer;
		buffer << trim(getBuff()->str());
		if ( *currentObject ) (*currentObject)->setAttribute( "validTo", buffer.str(), false );
	}

	if (name=="Version")
	{
		_currentVersion = nullptr;
	}
	if (name == "VersionTransition")
	{
		_currentTransition = nullptr;
	}
	if (name=="Transaction")
	{
		if (_currentTransition)
		{
			_currentTransition->addTransaction(_currentTransaction);
		}
		_currentTransaction = nullptr;
	}
	if (name == "versionMember")
	{
		citygml::CityObject* tempCObj = *getCurrentCityObject();
		citygml::CityObject* child = tempCObj->getChild(0);
		_currentVersion->addMember(child);
		tempCObj->clearChildren();
		popCityObject();
		popObject();
		delete tempCObj;
	}
	if (name=="tag")
	{
		std::string tagWorkspace = "WORKSPACE=";
		std::stringstream rawBuffer;
		rawBuffer << trim(getBuff()->str());
		std::string buffer = rawBuffer.str();
		if (buffer.find(tagWorkspace)==0){
			std::string wName = buffer.substr(tagWorkspace.length());
			_workspaces[wName].versions.push_back(_currentVersion);
			_workspaces[wName].name=wName;
		}
		_currentVersion->addTag(buffer);
	}
	if (name=="reason")
	{
		std::stringstream rawBuff ;
		rawBuff << trim(getBuff()->str());
		_currentTransition->setReason(rawBuff.str());
	}
	if (name=="clonePredecessor")
	{
		std::stringstream rawBuff ;
		rawBuff << trim(getBuff()->str());
		std::string buffer = rawBuff.str();
		_currentTransition->setClone(buffer=="true");
	}
	if (name=="from")
	{
		_inFromTags = false;
	}
	if (name=="to")
	{
		_inToTags = false;
	}
	if (name=="type")
	{
		std::stringstream rawBuff ;
		rawBuff << trim(getBuff()->str());
		std::string buff = rawBuff.str();
		if (_currentTransaction)// we are in Transaction
		{
			if (buff=="insert") _currentTransaction->setType(temporal::TransactionValue::INSERT);
			if (buff=="delete") _currentTransaction->setType(temporal::TransactionValue::DEL);
			if (buff=="replace") _currentTransaction->setType(temporal::TransactionValue::REPLACE);
		}
		else // we are in VersionTransition
		{
			if (buff=="planned") _currentTransition->setType(temporal::TransitionValue::PLANNED);
			if (buff=="realized") _currentTransition->setType(temporal::TransitionValue::REALIZED);
			if (buff=="historical succession") _currentTransition->setType(temporal::TransitionValue::HISTORICAL_SUCCESSION);
			if (buff=="fork") _currentTransition->setType(temporal::TransitionValue::FORK);
			if (buff=="merge") _currentTransition->setType(temporal::TransitionValue::MERGE);
		}
	}
	if (name == "newFeature")
	{
		citygml::CityObject* tempCObj = *getCurrentCityObject();
		citygml::CityObject* child = tempCObj->getChild(0);
		_currentTransaction->setNewFeature(child);
		tempCObj->clearChildren();
		popCityObject();
		popObject();
		delete tempCObj;
	}
	if (name == "oldFeature")
	{
		citygml::CityObject* tempCObj = *getCurrentCityObject();
		citygml::CityObject* child = tempCObj->getChild(0);
		_currentTransaction->setOldFeature(child);
		tempCObj->clearChildren();
		popCityObject();
		popObject();
		delete tempCObj;
	}
}
/******************************************************/
void TempHandler::endDocument()
{
	for (temporal::VersionTransition* transition : _transitions)
	{
		if(transition->from()->_isXlink == citygml::xLinkState::UNLINKED)
		{
			try {
				std::string id = getIDfromQuery(transition->from()->getAttribute("xlink"));
				for ( temporal::Version* version : _versions)
				{
					if(id==version->getId()) transition->setFrom(version);
				}
			} catch (...) {std::cerr<<"ERROR: XLink expression not supported! : \""<<transition->from()->getAttribute("xlink")<<"\""<<std::endl;}
		}
		if(transition->to()->_isXlink == citygml::xLinkState::UNLINKED)
		{
			try {
				std::string id = getIDfromQuery(transition->to()->getAttribute("xlink"));
				for ( temporal::Version* version : _versions)
				{
					if(id==version->getId()) transition->setTo(version);
				}
			} catch (...) {std::cerr<<"ERROR: XLink expression not supported! : \""<<transition->to()->getAttribute("xlink")<<"\""<<std::endl;}
		}
		//std::cout<<"Transition \""<<transition->getId()<<"\" :"<<std::endl;
		//std::cout<<"    - from: "<<transition->from()->getId()<<std::endl;
		//std::cout<<"    - to: "<<transition->to()->getId()<<std::endl;
	}
	//std::cout<<std::endl;
	for ( temporal::Version* version : _versions)
	{
		//std::cout<<"Version \""<<version->getId()<<"\" :"<<std::endl;
		std::vector<citygml::CityObject*>* members = version->getVersionMembers();
		for (std::vector<citygml::CityObject*>::iterator it = members->begin(); it != members->end(); it++)
		{
			if ((*it)->_isXlink==citygml::xLinkState::UNLINKED)
			{
				try {
				std::string id = getIDfromQuery((*it)->getAttribute("xlink"));
				citygml::CityObject* target = (*getModel())->getNodeById(id);
				(*it)=target;
				} catch (...) {std::cerr<<"ERROR: XLink expression not supported! : \""<<(*it)->getAttribute("xlink")<<"\""<<std::endl;}
			} 
		}
		//for (std::vector<citygml::CityObject*>::iterator it = members->begin(); it != members->end(); it++)
		//{
		//	std::cout<<"    - member: "<<(*it)->getId()<<std::endl;
		//}
	}

	citygml::CityModel** model = getModel();
	(*model)->setVersions(_versions,_transitions);
	(*model)->setWorkspaces(_workspaces);
}
