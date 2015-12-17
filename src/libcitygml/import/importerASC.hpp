#ifndef _IMPORTERASC_HPP_
#define _IMPORTERASC_HPP_

#include "importer.hpp"
#include "../citygml.hpp"
#include "../gui/osg/osgMnt.hpp"

namespace citygml
{

class ImporterASC : public Importer
{
public:
	ImporterASC(void);
	~ImporterASC(void);
	
	CityModel* reliefToCityGML(MNT*);
	CityModel* waterToCityGML(MNT*);
protected:
	Geometry* generateTriangles( MNT* );
};

} //namespace citygml

#endif //_IMPORTERASC_HPP_