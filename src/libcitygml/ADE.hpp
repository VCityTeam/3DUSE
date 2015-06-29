#ifndef _ADE_HPP_
#define _ADE_HPP_

#include "parser.hpp"

#include <libxml/parser.h>
#include <libxml/SAX.h>

class ADEHandler
{
public:
	ADEHandler(void);
	ADEHandler(citygml::CityGMLHandler*);
	~ADEHandler(void);
	void endElement(std::string);
protected:
	citygml::CityGMLHandler* gmlHandler;
	//access to gmlHandler members
	//std::map< std::string, citygml::CityGMLNodeType >* getCityGMLNodeTypeMap(){return &(gmlHandler->s_cityGMLNodeTypeMap);}
	//std::vector< std::string >* getKnownNamespace(){return &(gmlHandler->s_knownNamespace);}
	//std::vector< std::string >* getNodePath(){return &(gmlHandler->_nodePath);}
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

#endif