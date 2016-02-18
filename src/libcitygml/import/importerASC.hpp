#ifndef _IMPORTERASC_HPP_
#define _IMPORTERASC_HPP_

#include "importer.hpp"
#include "../citygml.hpp"
//#include "src/core/ASCRaster.hpp"
#include "src/gui/osg/osgMnt.hpp"
#include "src/gui/osg/osgGDAL.hpp"

namespace citygml
{

class ImporterASC : public Importer
{
public:
	ImporterASC(void);
	~ImporterASC(void);
	
	CityModel* reliefToCityGML(MNT*);
	CityModel* waterToCityGML(MNT*);
	CityModel* waterToCityGMLPolygons(MNT*);
	CityModel* fusionResolutions(MNT* asc1, MNT* asc2);
	void cutASC(MNT*, std::string, std::string, int );
	
protected:
	Geometry* generateTriangles(MNT*);
	void propagateCategory(MNT*,std::deque<std::pair<int, int>>*, float alt, float zPrec);
	OGRPolygon* createPoly(MNT*, int x,int y,float prec);

	std::vector<OGRPolygon*> geom_list;
	bool* treated;
};

} //namespace citygml

#endif //_IMPORTERASC_HPP_