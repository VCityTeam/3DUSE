// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef OSGINFODATATYPE_HPP
#define OSGINFODATATYPE_HPP

////////////////////////////////////////////////////////////////////////////////
/// \brief
////////////////////////////////////////////////////////////////////////////////
#include "osgInfo.hpp"
#include "vecs.hpp"
#include <osg/Switch>
#include <osg/Point>
#include <osg/MatrixTransform>


class InfoDataType : public osg::Referenced
{
public:
    InfoDataType();
    InfoDataType(osg::Node* n);
    osg::ref_ptr<osg::Switch> getSwitchRoot();
    void computeDCAM(osg::Camera *cam, osgInfo *info);
    void computeDSC(osg::Camera *cam, int screenX, int screenY, osgInfo *info);
    void computeOVa(int screenX, int screenY, std::map<float, osgInfo *> m_info);
    void updateScale(osgInfo *info, int screenX, int screenY);
    void setDisplayability(osgInfo *info);
    void stairedDisplay(std::map<float, osgInfo*> m_info);
    void display();

protected:
    osg::ref_ptr<osg::Switch> switchRoot;

};

#endif // OSGINFODATATYPE_HPP
