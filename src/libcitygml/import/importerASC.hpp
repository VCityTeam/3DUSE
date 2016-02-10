#ifndef _IMPORTERASC_HPP_
#define _IMPORTERASC_HPP_

#include "importer.hpp"
#include "../citygml.hpp"
#include "src/gui/osg/osgGDAL.hpp"

namespace citygml
{

class ImporterASC : public Importer
{
public:
	ImporterASC(void);
	~ImporterASC(void);
	
	CityModel* reliefToCityGML();
	CityModel* waterToCityGML();
	bool charge(const char*, const char*);
	void cutASC(std::string, std::string, int );

	CityModel* waterToCityGMLPolygons();
	void propagateCategory(std::deque<std::pair<int, int>>*);

protected:
	Geometry* generateTriangles();

	float  get_altitude(const int x, const int y)  { return altitudes[x+y*dim_x]; }

	char	nom_chantier[500];
	char	unites_xy[20];
	float	precision_xy;
	int		dim_x, dim_y;
	float	x_noeud_SO, y_noeud_SO;
	float	pas_x, pas_y;
	char	unites_z[20];
	float	precision_z;

	float	NODATA_value;

	float*	altitudes;
	std::vector<OGRPolygon*> geom_list;
	bool* treated;
	OGRPolygon* createPoly(int,int);
};

} //namespace citygml

#endif //_IMPORTERASC_HPP_