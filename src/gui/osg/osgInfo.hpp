// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGINFO_HPP__
#define __OSGINFO_HPP__

////////////////////////////////////////////////////////////////////////////////
/// \brief Info class to manage doc and img
////////////////////////////////////////////////////////////////////////////////

#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
using boost::uuids::uuid;
using boost::uuids::random_generator;


class osgInfo
{
public:
    osgInfo();
    osgInfo(float height, float width, osg::Vec3 position, double angle, osg::Vec3 axis, std::string filepath, std::string name);

    void updateGeometry();

    // Setters
    void setPosition(osg::Vec3 newPos);
    void setAxis(osg::Vec3 newAxis);

    void setHeight(float newHeight);
    void setWidth(float newWidth);
    void setAngle(float newAngle);

    void setTexture(std::string filepath);

    // Getters
    osg::Vec3 getPosition();
    double getAngle();
    osg::Vec3 getAxis();

    osg::Texture2D* getTexture();
    osg::StateSet* getState();
    osg::Geometry* getGeom() ;
    osg::Geode* getGeode() ;
    osg::PositionAttitudeTransform* getPAT() ;

    boost::uuids::uuid getID();

private:

    boost::uuids::uuid id; ///unique ID of the info doc
    std::string m_name ; ///name of the info doc
    std::string m_filetype;
    std::string m_purposetype;
    std::string m_sourcetype;


    osg::Vec3 m_position ; ///position of the node
    osg::Vec3 m_axe ; ///rotation axis of the node

    float m_height; ///height of the doc
    float m_width; ///width of the doc
    double m_angle ; ///rotation angle of the node

    unsigned int m_date_deb ;
    unsigned int m_date_fin ;


    osg::Texture2D *m_texture ; ///texture of the doc
    osg::StateSet *m_state; ///state of the doc
    osg::Geometry  *m_geom; ///geometry instance for the node
    osg::Geode *m_geode; ///geometry node
    osg::PositionAttitudeTransform *m_pat; ///position attitude transforme of the node

};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGINFO_HPP__

