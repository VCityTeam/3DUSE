// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef OSGUPDATEINFO_HPP
#define OSGUPDATEINFO_HPP

////////////////////////////////////////////////////////////////////////////////
/// \brief
////////////////////////////////////////////////////////////////////////////////

#include "osgInfo.hpp"
#include "osgInfoDataType.hpp"
#include "gui/moc/mainWindow.hpp"
#include <osg/Switch>
#include <osg/NodeCallback>
#include <map>


class UpdateInfo: public osg::NodeCallback
{
public:
    UpdateInfo();
    void operator()( osg::Node* node, osg::NodeVisitor* nv );
};

#endif // OSGUPDATEINFO_HPP
