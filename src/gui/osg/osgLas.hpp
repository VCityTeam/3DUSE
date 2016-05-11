// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGLAS_HPP__
#define __OSGLAS_HPP__

#ifdef _MSC_VER
#pragma warning(disable : 4996) // TEMP MT
#pragma warning(disable : 4267) // TEMP MT
#endif

// MT : the first time, you must do from /externals/laslib folder :
// mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release; cmake .. -DCMAKE_BUILD_TYPE=Release; make; sudo make install
#include <lasreader.hpp>

#include <libcitygml/URI.hpp>

#include <osg/Geode>
////////////////////////////////////////////////////////////////////////////////
class LAS
{
public:
	LAS();
	~LAS();

	bool open(const char* nom_fichier);
	void close();
	osg::ref_ptr<osg::Geode> buildLasPoints(const vcity::URI& uriLayer, float offset_x, float offset_y, float offset_z=0.0f, int zfactor=1);

private:
	LASreadOpener lasreadopener;
	LASreader* lasreader;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGLAS_HPP__
