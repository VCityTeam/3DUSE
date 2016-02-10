// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGINFO_HPP__
#define __OSGINFO_HPP__

////////////////////////////////////////////////////////////////////////////////
/// \brief Info class to manage doc and img
////////////////////////////////////////////////////////////////////////////////

#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/Texture2D>
#include <osgDB/ReadFile>


class osgInfo
{
public:
    osgInfo();
    osgInfo(osg::Vec3 position, float height, float width, float angle, osg::Vec3 axis, std::string filepath);

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
    float getAngle();
    osg::Vec3 getAxe();
    osg::Texture2D *getTexture();

    osg::Geometry* getGeom() ;

private:

    osg::Vec3 m_position ;
    osg::Vec3 m_axe ;

    float m_height;
    float m_width;
    float m_angle ;

    osg::Texture2D *m_texture ;

    osg::Geometry  *m_geom;

};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGINFO_HPP__

