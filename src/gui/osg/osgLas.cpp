// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgLas.hpp"

#include <time.h>
#include <osg/Geometry>

#include <core/application.hpp>

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
bool LAS::open(const char* nom_fichier)
{
	lasreadopener.set_file_name(nom_fichier);

	// open lasreader
	lasreader = lasreadopener.open();
	if (lasreader == 0)
	{
		fprintf(stderr, "LAS: ERROR: could not open lasreader\n");
		delete lasreader;

		return false;
	}

	return true;
}
void LAS::close()
{
	lasreader->close();

	delete lasreader;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> LAS::buildLasPoints(const vcity::URI& uriLayer, float offset_x, float offset_y, float offset_z, int zfactor)
{
	// function ?
	vcity::abstractLayer* layer = vcity::app().getScene().getLayer(uriLayer);
	vcity::LayerLas* layerLas = NULL;
    if(layer)
		layerLas = dynamic_cast<vcity::LayerLas*>(layer);

    osg::ref_ptr<osg::Geode> geode;
	geode = new osg::Geode;
	geode->setName("lasPoints");

		// Create geometry basic properties
        osg::Geometry* geom = new osg::Geometry;
        geode->addDrawable( geom );
    
		// vertices
        osg::Vec3Array* va = new osg::Vec3Array(lasreader->npoints);
		osg::Vec4Array* colors = new osg::Vec4Array;
		unsigned int i=0;                

		// loop over points
		double start_time = taketime();
		fprintf(stdout, "LAS: reading %lld points from '%s'.\n", lasreader->npoints, lasreadopener.get_file_name());
			while (lasreader->read_point())
			{
				(*va)[i++].set( offset_x+(double)(lasreader->point.get_X())/100., offset_y+(double)(lasreader->point.get_Y())/100., (((double)(lasreader->point.get_Z())/100.)-offset_z) * zfactor );				
				if (layerLas)
					layerLas->addLASpoint(lasreader->point);

				switch(lasreader->point.get_classification())
				{
					case  1 : colors->push_back(osg::Vec4(0.0f,0.3f,0.0f,1.0f)); break; // Non classé			(vert foncé)
					case  4 : colors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f)); break; // Végétation moyenne	(vert)
					case  8 : colors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f)); break; // Model Key Point		(rouge)
					case  9 : colors->push_back(osg::Vec4(0.0f,1.0f,1.0f,1.0f)); break; // Eau					(bleu clair)
					case 10 : colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f)); break; // Ponts				(jaune)

					default : colors->push_back(osg::Vec4(0.0f,0.0f,0.0f,1.0f)); break; // autres, si besoin... (noir)
				}
			}
			//fprintf(stdout, "x: %lf - y: %lf - z: %lf \n", (double)(lasreader->point.get_X())/100., (double)(lasreader->point.get_Y())/100., ((double)(lasreader->point.get_Z())/100.));
		fprintf(stdout, "LAS: total time: %g sec for %lld points\n", taketime()-start_time, lasreader->p_count);

		geom->setVertexArray( va );

		geom->setColorArray(colors);
		geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		// Create geometry primitives
        osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS);

		for( unsigned int j=0; j<i; j++ )
			indices->push_back( j );
            
        geom->getOrCreateStateSet(); // MT : rajouté par MM -> utilité ?
            
        geom->addPrimitiveSet( indices );
		// Create geometry primitives

    return geode;
}
////////////////////////////////////////////////////////////////////////////////