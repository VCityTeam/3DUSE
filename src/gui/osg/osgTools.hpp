// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGTOOLS_HPP__
#define __OSGTOOLS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "libcitygml/URI.hpp"
#include <osg/Geode>
////////////////////////////////////////////////////////////////////////////////
namespace osgTools
{
    /// Build a bbox
    osg::ref_ptr<osg::Geode> buildBBox(osg::Vec3 lowerBound, osg::Vec3 upperBound);

    /// \brief getURI Compute the uri of an osg node
    /// \param node The osg node
    /// \return URI
    vcity::URI getURI(osg::Node* node);
}
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGTOOLS_HPP__
