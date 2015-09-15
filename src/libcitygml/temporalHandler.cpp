#include "temporalHandler.hpp"
#include "utils.hpp"
#include "citygml.hpp"

#include <libxml/parser.h>
#include <libxml/SAX.h>

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
//std::string TempHandler::getGmlIdAttribute( void* attributes ) { return getAttribute( attributes, "gml:id", "" ); }

std::string TempHandler::removeNamespace(std::string name)
{
	size_t pos = name.find_first_of( ":" );
	return name.substr( pos + 1 );
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
		_versions.push_back(_currentVersion);
		if(_inFromTags) _currentTransition->setFrom(_currentVersion);
		if(_inToTags) _currentTransition->setTo(_currentVersion);
	}
	if (name == "VersionMember")
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
#if 0
#define MANAGE_OBJECT( _t_ )\
	if(name.find(#_t_)!=std::string::npos){\
		citygml::CityObjectsTypeMask objectsMask = *getObjectsMask();\
		if ( objectsMask & citygml::COT_##_t_ )\
        {\
		pushCityObject( new citygml::##_t_##( getGmlIdAttribute( attributes ) ) );\
			citygml::CityObject** currentCityObject = getCurrentCityObject();\
            pushObject( *currentCityObject ); std::cout << "new "<< #_t_ " - " << (*currentCityObject)->getId() << std::endl;\
			citygml::ParserParams* params = getParams();\
            if(params->temporalImport)\
            {\
                std::string id = getGmlIdAttribute( attributes );\
                auto it = id.find("_TAG");\
                auto it1 = id.find("_STATE");\
                auto it2 = id.find("_DYNSTATE");\
                if(it!=std::string::npos)\
                {\
					citygml::CityObjectTag** currentTag = getCurrentTag();\
                    *currentTag = new citygml::CityObjectTag(0, *currentCityObject);\
					citygml::CityModel** model = getModel();\
                    citygml::CityObject* o = (*model)->getNodeById(id.substr(0, it));\
                    o->addTag(*currentTag);\
                    (*currentTag)->m_parent = o;\
                    o->checkTags();\
                    (*currentCityObject)->m_temporalUse = true;\
                }\
                else if(it1!=std::string::npos)\
                {\
					citygml::CityObjectState** currentState = getCurrentState();\
                    *currentState = new citygml::CityObjectState(*currentCityObject);\
					citygml::CityModel** model = getModel();\
                    citygml::CityObject* o = (*model)->getNodeById(id.substr(0, it1));\
                    o->addState(*currentState);\
                    (*currentState)->m_parent = o;\
                    (*currentCityObject)->m_temporalUse = true;\
                }\
                else if(it2!=std::string::npos)\
                {\
					citygml::CityObjectDynState** currentDynState = getCurrentDynState();\
                    *currentDynState = new citygml::CityObjectDynState(*currentCityObject);\
					citygml::CityModel** model = getModel();\
                    citygml::CityObject* o = (*model)->getNodeById(id.substr(0, it2));\
                    o->addState(*currentDynState);\
                    (*currentDynState)->m_parent = o;\
                    (*currentCityObject)->m_temporalUse = true;\
                }\
            }\
        }\
        else\
        {\
            pushCityObject( nullptr );\
			bool* filterNodeType = getFilterNodeType();\
            *filterNodeType = true;\
			unsigned int * filterDepth = getFilterDepth();\
            *filterDepth = getPathDepth();\
        }\
	}\
		
	MANAGE_OBJECT( GenericCityObject );
	MANAGE_OBJECT( Building );
	MANAGE_OBJECT( BuildingPart );
	MANAGE_OBJECT( Room );
	MANAGE_OBJECT( BuildingInstallation );
	MANAGE_OBJECT( BuildingFurniture );
	MANAGE_OBJECT( Door );
	MANAGE_OBJECT( Window );
	MANAGE_OBJECT( CityFurniture );
	MANAGE_OBJECT( Track );
	MANAGE_OBJECT( Road );
	MANAGE_OBJECT( Railway );
	MANAGE_OBJECT( Square );
	MANAGE_OBJECT( PlantCover );
	MANAGE_OBJECT( SolitaryVegetationObject );
	MANAGE_OBJECT( WaterBody );
	MANAGE_OBJECT( TINRelief );
	MANAGE_OBJECT( LandUse );		
	MANAGE_OBJECT( Tunnel );
	MANAGE_OBJECT( Bridge );
	MANAGE_OBJECT( BridgeConstructionElement );
	MANAGE_OBJECT( BridgeInstallation );
	MANAGE_OBJECT( BridgePart );
#undef MANAGE_OBJECT


	std::string nameSurface;
#define MANAGE_SURFACETYPE( _t_ )\
	nameSurface= #_t_+std::string("Surface");\
	if(name.find(nameSurface)!=std::string::npos){\
		citygml::CityObjectsTypeMask objectsMask = *getObjectsMask();\
		citygml::GeometryType* currentGeometryType = getCurrentGeometryType();\
		*currentGeometryType = citygml::GT_##_t_;\
		if ( objectsMask & citygml::COT_##_t_##Surface )\
        {\
		pushCityObject( new citygml::##_t_##Surface( getGmlIdAttribute( attributes ) ) );\
			citygml::CityObject** currentCityObject = getCurrentCityObject();\
            pushObject( *currentCityObject ); /*std::cout << "new "<< #_t_ " - " << _currentCityObject->getId() << std::endl;*/\
			citygml::ParserParams* params = getParams();\
            if(params->temporalImport)\
            {\
                std::string id = getGmlIdAttribute( attributes );\
                auto it = id.find("_TAG");\
                auto it1 = id.find("_STATE");\
                auto it2 = id.find("_DYNSTATE");\
                if(it!=std::string::npos)\
                {\
					citygml::CityObjectTag** currentTag = getCurrentTag();\
                    *currentTag = new citygml::CityObjectTag(0, *currentCityObject);\
                    (*currentCityObject)->m_path = params->m_basePath;\
					citygml::CityModel** model = getModel();\
                    citygml::CityObject* o = (*model)->getNodeById(id.substr(0, it));\
                    o->addTag(*currentTag);\
                    (*currentTag)->m_parent = o;\
                    o->checkTags();\
                    (*currentCityObject)->m_temporalUse = true;\
                }\
                else if(it1!=std::string::npos)\
                {\
					citygml::CityObjectState** currentState = getCurrentState();\
                    *currentState = new citygml::CityObjectState(*currentCityObject);\
					citygml::CityModel** model = getModel();\
                    citygml::CityObject* o = (*model)->getNodeById(id.substr(0, it1));\
                    o->addState(*currentState);\
                    (*currentState)->m_parent = o;\
                    (*currentCityObject)->m_temporalUse = true;\
                }\
                else if(it2!=std::string::npos)\
                {\
					citygml::CityObjectDynState** currentDynState = getCurrentDynState();\
                    *currentDynState = new citygml::CityObjectDynState(*currentCityObject);\
					citygml::CityModel** model = getModel();\
                    citygml::CityObject* o = (*model)->getNodeById(id.substr(0, it2));\
                    o->addState(*currentDynState);\
                    (*currentDynState)->m_parent = o;\
                    (*currentCityObject)->m_temporalUse = true;\
                }\
            }\
        }\
        else\
        {\
            pushCityObject( nullptr );\
			bool* filterNodeType = getFilterNodeType();\
            *filterNodeType = true;\
			unsigned int * filterDepth = getFilterDepth();\
            *filterDepth = getPathDepth();\
        }\
	}
	MANAGE_SURFACETYPE( Wall );
	MANAGE_SURFACETYPE( Roof );
	MANAGE_SURFACETYPE( Ground );
	MANAGE_SURFACETYPE( Closure );
	MANAGE_SURFACETYPE( Floor );
	MANAGE_SURFACETYPE( InteriorWall );
	MANAGE_SURFACETYPE( Ceiling );
#undef MANAGE_SURFACETYPE
#endif
}
/******************************************************/
void TempHandler::endElement(std::string name)
{

	name = removeNamespace(name);

	if (name=="validFrom")
	{
		citygml::Object** currentObject = getCurrentObject();
		if ( *currentObject ) (*currentObject)->setAttribute( "validFrom", getBuff()->str(), false );
	}
	if (name=="validTo")
	{
		citygml::Object** currentObject = getCurrentObject();
		if ( *currentObject ) (*currentObject)->setAttribute( "validTo", getBuff()->str(), false );
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
		_currentVersion->addTag(getBuff()->str());
	}
	if (name=="reason")
	{
		_currentTransition->setReason(getBuff()->str());
	}
	if (name=="clonePredecessor")
	{
		std::string buff = getBuff()->str();
		_currentTransition->setClone(buff=="true");
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
		std::string buff = getBuff()->str();
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
#if 0
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
	if ((name.find("GenericCityObject")!=std::string::npos)
	||(name.find("Building")!=std::string::npos)
	||(name.find("BuildingPart")!=std::string::npos)
	||(name.find("Room")!=std::string::npos)
	||(name.find("BuildingInstallation")!=std::string::npos)
	||(name.find("BuildingFurniture")!=std::string::npos)
	||(name.find("Door")!=std::string::npos)
	||(name.find("Window")!=std::string::npos)
	||(name.find("CityFurniture")!=std::string::npos)
	||(name.find("Track")!=std::string::npos)
	||(name.find("Road")!=std::string::npos)
	||(name.find("Railway")!=std::string::npos)
	||(name.find("Square")!=std::string::npos)
	||(name.find("PlantCover")!=std::string::npos)
	||(name.find("SolitaryVegetationObject")!=std::string::npos)
	||(name.find("WaterBody")!=std::string::npos)
	||(name.find("TINRelief")!=std::string::npos)
	||(name.find("LandUse")!=std::string::npos)
	||(name.find("Tunnel")!=std::string::npos)
	||(name.find("Bridge")!=std::string::npos)
	||(name.find("BridgeConstructionElement")!=std::string::npos)
	||(name.find("BridgeInstallation")!=std::string::npos)
	||(name.find("BridgePart")!=std::string::npos)
	||(name.find("WallSurface")!=std::string::npos)
	||(name.find("RoofSurface")!=std::string::npos)
	||(name.find("GroundSurface")!=std::string::npos)
	||(name.find("ClosureSurface")!=std::string::npos)
	||(name.find("FloorSurface")!=std::string::npos)
	||(name.find("InteriorWallSurface")!=std::string::npos)
	||(name.find("CeilingSurface")!=std::string::npos))
	{
		citygml::ParserParams* params = getParams();
		citygml::CityObject** currentCityObject = getCurrentCityObject();
		if ( *currentCityObject && ( (*currentCityObject)->size() > 0 || (*currentCityObject)->getChildCount() > 0 || !(params->pruneEmptyObjects) ) ) 
        {	// Prune empty objects
            if(params->temporalImport && (*currentCityObject)->m_temporalUse)
            {
                // this is a cityobject for a tag or state, do not add to citymodel
				citygml::CityObjectTag** currentTag = getCurrentTag();
				citygml::CityObjectState** currentState = getCurrentState();
				citygml::CityObjectDynState** currentDynState = getCurrentDynState();
                *currentTag = nullptr;
                *currentState = nullptr;
                *currentDynState = nullptr;
            }
            else
            {
				citygml::CityModel** model = getModel();
                (*model)->addCityObject( *currentCityObject );
				std::stack<citygml::CityObject*>* cityObjectStack = getCityObjectStack();
                if ( cityObjectStack->size() == 1 ) (*model)->addCityObjectAsRoot( *currentCityObject );
            }
		}
		else delete *currentCityObject; 
		popCityObject();
		popObject();
		bool* filterNodeType = getFilterNodeType();
		*filterNodeType = false;
		citygml::GeometryType* currentGeometryType = getCurrentGeometryType();
		*currentGeometryType = citygml::GT_Unknown;
	}
#endif
}
