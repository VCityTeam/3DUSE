// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGLAS_HPP__
#define __OSGLAS_HPP__

#ifdef _MSC_VER
#pragma warning(disable : 4996) // TEMP MT
#pragma warning(disable : 4267) // TEMP MT
#endif

#include <lasreader.hpp>

//#include <osg/Geode>
////////////////////////////////////////////////////////////////////////////////
class LAS
{
public:
	LAS();
	~LAS();

	bool charge(const char* nom_fichier);

private:
	LASreadOpener lasreadopener;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGLAS_HPP__
