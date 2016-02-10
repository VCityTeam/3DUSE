// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgInfo.hpp"
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
osgInfo::osgInfo()
{
    m_texture = new osg::Texture2D;
    m_geom = new osg::Geometry;
}

osgInfo::osgInfo(osg::Vec3 pos, float height, float width, float ang, osg::Vec3 axis, std::string filepath)
{
    m_texture = new osg::Texture2D;
    m_geom = new osg::Geometry;

    m_position = pos;
    m_angle = ang;
    m_axe = axis;

    m_texture->setImage(osgDB::readImageFile(filepath));
    m_height = height;
    m_width = width;

    osg::Vec3Array* qVertices = new osg::Vec3Array;
    qVertices->push_back( osg::Vec3( -m_width/2, 0, 0) ); // bottom left
    qVertices->push_back( osg::Vec3(m_width/2, 0, 0) ); // bottom right
    qVertices->push_back( osg::Vec3(m_width/2,0, m_height) ); // top right
    qVertices->push_back( osg::Vec3(-m_width/2,0, m_height) ); // top left

    m_geom->setVertexArray( qVertices );

    osg::DrawElementsUInt* Quad = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    Quad->push_back(3);
    Quad->push_back(2);
    Quad->push_back(1);
    Quad->push_back(0);
    m_geom->addPrimitiveSet(Quad);

    osg::Vec2Array* qTexCoords = new osg::Vec2Array;
    qTexCoords->push_back(osg::Vec2(0.0f,0.0f) );
    qTexCoords->push_back(osg::Vec2(1.0f,0.0f) );
    qTexCoords->push_back(osg::Vec2(1.0f,1.0f) );
    qTexCoords->push_back(osg::Vec2(0.0f,1.0f) );

    m_geom->setTexCoordArray(0,qTexCoords);

}

float osgInfo::getAngle()
{
    return m_angle;
}

osg::Vec3 osgInfo::getPosition()
{
    return m_position;
}

osg::Vec3 osgInfo::getAxe()
{
    return m_axe;
}

osg::Geometry* osgInfo::getGeom()
{
    return m_geom;
}

osg::Texture2D* osgInfo::getTexture()
{
    return m_texture;
}

void osgInfo::setAxis(osg::Vec3 newAxis)
{
    m_axe=newAxis;
}

void osgInfo::setPosition(osg::Vec3 newPos)
{
    m_position=newPos;
}

void osgInfo::setAngle(float newAngle)
{
    m_angle=newAngle;
}

void osgInfo::setHeight(float newHeight)
{
    m_height=newHeight;
    osg::Vec3Array* qVertices = new osg::Vec3Array;
    qVertices->push_back( osg::Vec3( -m_width/2, 0, 0) ); // bottom left
    qVertices->push_back( osg::Vec3(m_width/2, 0, 0) ); // bottom right
    qVertices->push_back( osg::Vec3(m_width/2,0, m_height) ); // top right
    qVertices->push_back( osg::Vec3(-m_width/2,0, m_height) ); // top left

    m_geom->setVertexArray( qVertices );

    osg::DrawElementsUInt* Quad = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    Quad->push_back(3);
    Quad->push_back(2);
    Quad->push_back(1);
    Quad->push_back(0);
    m_geom->addPrimitiveSet(Quad);
}

void osgInfo::setWidth(float newWidth)
{
    m_width=newWidth;
    osg::Vec3Array* qVertices = new osg::Vec3Array;
    qVertices->push_back( osg::Vec3( -m_width/2, 0, 0) ); // bottom left
    qVertices->push_back( osg::Vec3(m_width/2, 0, 0) ); // bottom right
    qVertices->push_back( osg::Vec3(m_width/2,0, m_height) ); // top right
    qVertices->push_back( osg::Vec3(-m_width/2,0, m_height) ); // top left

    m_geom->setVertexArray( qVertices );

    osg::DrawElementsUInt* Quad = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    Quad->push_back(3);
    Quad->push_back(2);
    Quad->push_back(1);
    Quad->push_back(0);
    m_geom->addPrimitiveSet(Quad);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
