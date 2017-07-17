// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGLAS_HPP__
#define __OSGLAS_HPP__

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#pragma warning(disable : 4267)
#endif

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
    osg::ref_ptr<osg::Geode> buildLasPoints(const vcity::URI& uriLayer, float offset_x, float offset_y, float offset_z = 0.0f, int zfactor = 1);

private:
    LASreadOpener lasreadopener;
    LASreader* lasreader;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGLAS_HPP__
