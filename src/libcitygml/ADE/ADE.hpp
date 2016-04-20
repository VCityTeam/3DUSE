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
    virtual ~ADEHandler(void){};
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
	unsigned int* getFilterDepth(){return &(gmlHandler->_filterDepth);} // MT (MAC OS X problem...)
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
	std::string getGmlIdAttribute( void* attributes );
	unsigned int getPathDepth( void );
	void pushCityObject( citygml::CityObject* );
	void pushObject( citygml::Object* );
	void popCityObject( void );
	void popObject( void );
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Derived ADE handlers management :
//
//  JE 17/02/2016: DISCARDED BECAUSE PROBLEMS WITH RECENT VERSIONS OF UBUNTU: 
//	inspired from http://stackoverflow.com/questions/582331/is-there-a-way-to-instantiate-objects-from-a-string-holding-their-class-name
//
//
////template for creating a handler of the given type
//template<typename T> ADEHandler* createT() {return new T();};
//
////Contains the map for the derived ADE handlers
//struct ADEHandlerFactory
//{
//	typedef std::map<std::string,ADEHandler*(*)()> mapType;
//
//	static ADEHandler* createInstance(std::string const& s) 
//	{
//		mapType::iterator it = getMap()->find(s);
//		if(it == getMap()->end())
//			return 0;
//		return it->second();
//    }
//
//	static void getInstances(std::map<std::string,ADEHandler*>& map)
//	{
//		for(mapType::iterator it = getMap()->begin(); it != getMap()->end(); it++) map[it->first] = it->second();
//	}
//
//protected:
//	static mapType* getMap() {if (!ADEmap) {ADEmap = new mapType();} return ADEmap;}
//
//private:
//	static mapType* ADEmap;
//};
//
////template for registring each ADE handler in the ADEHandlerFactory map
//template<typename T> struct ADERegister:ADEHandlerFactory
//{
//	ADERegister(const std::string s)
//	{
//		std::pair<std::string,ADEHandler*(*)()> entry (s,&createT<T>);
//		getMap()->insert(entry);
//	}
//};

class ADEHandlerFactory
{
public:
	void getInstances(std::map<std::string,ADEHandler*>*); 
};

///////////////////////////////////////////////////////////////////////////////////////////////////
#endif
