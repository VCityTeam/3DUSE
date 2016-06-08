// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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
