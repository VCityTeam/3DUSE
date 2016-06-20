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

    /// \brief Compute and update DSC value for every document in the map
    /// \param int screenX width of screen (in pixels)
    /// \param int screenY height of screen (in pixels)
    /// \param std::map sorting infos according to their DCAM
    void computeOVa(int screenX, int screenY, std::map<float, osgInfo *> m_info);

    /// \brief Display document in map as stairs
    /// Each document is higher than the one in front
    /// \param sorted map with osgInfo and its DCAM
    void stairedDisplay(std::map<float, osgInfo*> m_info);

    /// \brief Display document by OVa
    /// \param sorted map with osgInfo and its DCAM
    void OVaDisplay(std::map<float, osgInfo *> m_info);

    /// \brief Turn on switches with displayable documents
    void display();

protected:
    osg::ref_ptr<osg::Switch> switchRoot;

};

#endif // OSGINFODATATYPE_HPP
