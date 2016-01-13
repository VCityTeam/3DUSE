#ifndef _IMPORTERASC_HPP_
#define _IMPORTERASC_HPP_

#include "importer.hpp"
#include "../citygml.hpp"

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

protected:
	Geometry* generateTriangles();

	int  get_altitude(const int x, const int y)  { return altitudes[x+y*dim_x]; }

	char	nom_chantier[500];
	char	unites_xy[20];
	float	precision_xy;
	int		dim_x, dim_y;
	float	x_noeud_NO, y_noeud_NO;
	float	pas_x, pas_y;
	char	unites_z[20];
	float	precision_z;

	float		NODATA_value;

	int		*altitudes;
};

} //namespace citygml

#endif //_IMPORTERASC_HPP_