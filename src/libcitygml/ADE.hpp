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
	~ADEHandler(void);
	void setGMLHandler(citygml::CityGMLHandler* gHandler){gmlHandler=gHandler;}
	virtual void endElement(std::string){};
protected:
	citygml::CityGMLHandler* gmlHandler;
	//access to gmlHandler members
	std::map< std::string, citygml::CityGMLNodeType >* getCityGMLNodeTypeMap(){return &(gmlHandler->s_cityGMLNodeTypeMap);}
	std::vector< std::string >* getKnownNamespace(){return &(gmlHandler->s_knownNamespace);}
	std::vector< std::string >* getNodePath(){return &(gmlHandler->_nodePath);}
	std::stringstream* getBuff(){return &(gmlHandler->_buff);}
	citygml::ParserParams* getParams(){return &(gmlHandler->_params);}
		//
		//citygml::CityModel* _model;

		//TVec3d _translate;

		//citygml::CityObject* _currentCityObject;
		//std::stack<citygml::CityObject*> _cityObjectStack;

		//citygml::Object* _currentObject;
		//std::stack<citygml::Object*> _objectStack;

		//citygml::Geometry* _currentGeometry;
  //      std::set<citygml::Geometry*> _geometries;

		//citygml::Polygon* _currentPolygon;

		//citygml::LinearRing* _currentRing;

		//citygml::Appearance* _currentAppearance;

		//citygml::CityObjectsTypeMask _objectsMask;

		//std::string _attributeName;

		//int _currentLOD;

		//bool _filterNodeType;
		//unsigned int _filterDepth;

		//std::vector<TVec3d> _points;

		//int _srsDimension;

		//char _orientation;

		//bool _exterior;

		//bool _appearanceAssigned;

		//citygml::GeometryType _currentGeometryType;

		//void* _geoTransform;

  //      // temporal ext
  //      citygml::CityObjectState* m_currentState;
  //      citygml::CityObjectDynState* m_currentDynState;
	citygml::CityObjectTag** getCurrentTag(){return &(gmlHandler->m_currentTag);}
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Derived ADE handlers management :

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