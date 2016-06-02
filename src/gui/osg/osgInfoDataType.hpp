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
    /// \brief InfoDataType constructor
    InfoDataType();

    /// \brief InfoDataType constructor
    /// \param osg::Node
    InfoDataType(osg::Node* n);

    /// \brief Get switchroot node member
    /// \return osg::ref_ptr<osg::Switch> pointer to switchroot
    osg::ref_ptr<osg::Switch> getSwitchRoot();

    /// \brief Compute and update DCAM value for current document
    /// \param osg::Camera* pointer to camera to get camera current position
    /// \param osgInfo* pointer to info object to make update
    void computeDCAM(osg::Camera *cam, osgInfo *info);

    /// \brief Compute and update DSC value for current document
    /// \param osg::Camera* pointer to camera to get camera current position
    /// \param int screenX width of screen (in pixels)
    /// \param int screenY height of screen (in pixels)
    /// \param osgInfo* pointer to info object to make update
    void computeDSC(osg::Camera *cam, int screenX, int screenY, osgInfo *info);

    /// \brief Compute and update DSC value for every document in the map
    /// \param int screenX width of screen (in pixels)
    /// \param int screenY height of screen (in pixels)
    /// \param std::map sorting infos according to their DCAM
    void computeOVa(int screenX, int screenY, std::map<float, osgInfo *> m_info);

    /// \brief Update scale of current document
    /// \param osgInfo* pointer to current info to update scale
    /// \param int screenX width of screen (in pixels)
    /// \param int screenY height of screen (in pixels)
    void updateScale(osgInfo *info, int screenX, int screenY);

    /// \brief Compute displayability of current info
    /// \param osgInfo* pointer to current info
    void setDisplayability(osgInfo *info);

    /// \brief Display document in map as stairs
    /// Each document is higher than the one in front
    /// \param osgInfo* pointer to current info
    void stairedDisplay(std::map<float, osgInfo*> m_info);

    /// \brief Turn on switches with displayable documents
    void display();

protected:
    osg::ref_ptr<osg::Switch> switchRoot;

};

#endif // OSGINFODATATYPE_HPP
