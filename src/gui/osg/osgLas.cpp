// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgLas.hpp"

#include <time.h>
//#include <osg/Geometry>

static double taketime()
{
  return (double)(clock())/CLOCKS_PER_SEC;
}

////////////////////////////////////////////////////////////////////////////////
LAS::LAS()
{
}
////////////////////////////////////////////////////////////////////////////////
LAS::~LAS()
{
}
////////////////////////////////////////////////////////////////////////////////
bool LAS::charge(const char* nom_fichier)
{
	double start_time = 0.0;

	start_time = taketime();
	lasreadopener.set_file_name(nom_fichier);

	// open lasreader
	LASreader* lasreader = lasreadopener.open();
	if (lasreader == 0)
	{
		fprintf(stderr, "LAS: ERROR: could not open lasreader\n");
		delete lasreader;

		return false;
	}

	fprintf(stdout, "LAS: reading %I64d points from '%s'.\n", lasreader->npoints, lasreadopener.get_file_name());

	// loop over points
	while (lasreader->read_point())
	{
	}

	fprintf(stdout, "LAS: total time: %g sec for %I64d points\n", taketime()-start_time, lasreader->p_count);

	lasreader->close();
	delete lasreader;

	return true;
}
////////////////////////////////////////////////////////////////////////////////