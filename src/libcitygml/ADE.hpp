#ifndef _ADE_HPP_
#define _ADE_HPP_

#include "parser.hpp"


///////////////////////////////////////////////////////////////////////////////////////////////////
// Base ADE Handler class, containing methods to access the CityGMLHandler members
class ADEHandler
{
public:
	ADEHandler(void){gmlHandler=NULL;}
	ADEHandler(citygml::CityGMLHandler* gHandler){gmlHandler=gHandler;}
	~ADEHandler(void){};
	void setGMLHandler(citygml::CityGMLHandler* gHandler){gmlHandler=gHandler;}
	virtual void startElement(std::string,void*){};
	virtual void endElement(std::string){};
	virtual void endDocument(){};
protected:
	citygml::CityGMLHandler* gmlHandler;
	//access to gmlHandler members
	std::map< std::string, citygml::CityGMLNodeType >* getCityGMLNodeTypeMap(){return &(gmlHandler->s_cityGMLNodeTypeMap);}
	std::vector< std::string >* getKnownNamespace(){return &(gmlHandler->s_knownNamespace);}
	std::vector< std::string >* getNodePath(){return &(gmlHandler->_nodePath);}
	std::stringstream* getBuff(){return &(gmlHandler->_buff);}
	citygml::ParserParams* getParams(){return &(gmlHandler->_params);}
	citygml::CityModel** getModel(){return &(gmlHandler->_model);}
	TVec3d* getTranslate(){return &(gmlHandler->_translate);}
	citygml::CityObject** getCurrentCityObject(){return &(gmlHandler->_currentCityObject);}
	std::stack<citygml::CityObject*>* getCityObjectStack(){return &(gmlHandler->_cityObjectStack);}
	citygml::Object** getCurrentObject(){return &(gmlHandler->_currentObject);}
	std::stack<citygml::Object*>* getObjectStack(){return &(gmlHandler->_objectStack);}
	citygml::Geometry** getCurrentGeometry(){return &(gmlHandler->_currentGeometry);}
	std::set<citygml::Geometry*>* getGeometries(){return &(gmlHandler->_geometries);}
	citygml::Polygon** getCurrentPolygon(){return &(gmlHandler->_currentPolygon);}
	citygml::LinearRing** getCurrentRing(){return &(gmlHandler->_currentRing);}
	citygml::Appearance** getCurrentAppearance(){return &(gmlHandler->_currentAppearance);}
	citygml::CityObjectsTypeMask* getObjectsMask(){return &(gmlHandler->_objectsMask);}
	std::string* getAttributeName(){return &(gmlHandler->_attributeName);}
	int* getCurrentLOD(){return &(gmlHandler->_currentLOD);}
	bool* getFilterNodeType(){return &(gmlHandler->_filterNodeType);}
	(unsigned int)* getFilterDepth(){return &(gmlHandler->_filterDepth);}
	std::vector<TVec3d>* getPoints(){return &(gmlHandler->_points);}
	int* getSrsDimension(){return &(gmlHandler->_srsDimension);}
	char* getOrientation(){return &(gmlHandler->_orientation);}
	bool* getExterior(){return &(gmlHandler->_exterior);}
	bool* getAppearanceAssigned(){return &(gmlHandler->_appearanceAssigned);}
	citygml::GeometryType* getCurrentGeometryType(){return &(gmlHandler->_currentGeometryType);}
	void** getGeoTransform(){return &(gmlHandler->_geoTransform);}
	citygml::CityObjectState** getCurrentState(){return &(gmlHandler->m_currentState);}
	citygml::CityObjectDynState** getCurrentDynState(){return &(gmlHandler->m_currentDynState);}
	citygml::CityObjectTag** getCurrentTag(){return &(gmlHandler->m_currentTag);}

	//some gmlHandler methods
	virtual std::string getAttribute( void*, const std::string&, const std::string&){return "";}
	inline std::string getGmlIdAttribute( void* attributes ) { return getAttribute( attributes, "gml:id", "" ); }
	inline unsigned int getPathDepth( void ) { return (getNodePath())->size(); }
	inline void pushCityObject( citygml::CityObject* object )
	{
		citygml::CityObject** currentCityObject = getCurrentCityObject();
        // add parent relation
        if(*currentCityObject)
        {
            object->_parent = *currentCityObject;
        }

		if ( *currentCityObject && object ) (*currentCityObject)->getChildren().push_back( object );
		std::stack<citygml::CityObject*>* cityObjectStack = getCityObjectStack();
		cityObjectStack->push( *currentCityObject );
		*currentCityObject = object;
	}
	inline void pushObject( citygml::Object* object )
	{
		std::stack<citygml::Object*>* objectStack = getObjectStack();
		objectStack->push( object );
		citygml::Object** currentObject = getCurrentObject();
		*currentObject = object;
	}
	inline void popCityObject( void )
	{
		citygml::CityObject** currentCityObject = getCurrentCityObject();
		std::stack<citygml::CityObject*>* cityObjectStack = getCityObjectStack();
		*currentCityObject = 0; 
		if ( cityObjectStack->empty() ) return; 
		*currentCityObject = cityObjectStack->top(); 
		cityObjectStack->pop();
	}
	inline void popObject( void )
	{
		citygml::Object** currentObject = getCurrentObject();
		std::stack<citygml::Object*>* objectStack = getObjectStack();
		*currentObject = 0; 
		if ( objectStack->empty() ) return; 
		objectStack->pop();
		*currentObject = objectStack->empty() ? 0 : objectStack->top();			
	}
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Derived ADE handlers management :
// inspired from http://stackoverflow.com/questions/582331/is-there-a-way-to-instantiate-objects-from-a-string-holding-their-class-name
//

//template for creating a handler of the given type
template<typename T> ADEHandler* createT() {return new T();};

//Contains the map for the derived ADE handlers
struct ADEHandlerFactory
{
	typedef std::map<std::string,ADEHandler*(*)()> mapType;

	static ADEHandler* createInstance(std::string const& s) 
	{
		mapType::iterator it = getMap()->find(s);
		if(it == getMap()->end())
			return 0;
		return it->second();
    }

	static void getInstances(std::map<std::string,ADEHandler*>& map)
	{
		for(mapType::iterator it = getMap()->begin(); it != getMap()->end(); it++) map[it->first] = it->second();
	}

protected:
	static mapType* getMap() {if (!ADEmap) {ADEmap = new mapType();} return ADEmap;}

private:
	static mapType* ADEmap;
};
//added to avoid linking errors?
ADEHandlerFactory::mapType * ADEHandlerFactory::ADEmap = new mapType();

//template for registring each ADE handler in the ADEHandlerFactory map
template<typename T> struct ADERegister:ADEHandlerFactory
{
	ADERegister(const std::string s)
	{
		std::pair<std::string,ADEHandler*(*)()> entry (s,&createT<T>);
		getMap()->insert(entry);
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////
#endif