// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgInfo.hpp"
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
osgInfo::osgInfo()
{
    m_texture = new osg::Texture2D;

    m_geom = new osg::Geometry;
    osg::Vec2Array* qTexCoords = new osg::Vec2Array;
    qTexCoords->push_back(osg::Vec2(0.0f,0.0f) );
    qTexCoords->push_back(osg::Vec2(1.0f,0.0f) );
    qTexCoords->push_back(osg::Vec2(1.0f,1.0f) );
    qTexCoords->push_back(osg::Vec2(0.0f,1.0f) );
    m_geom->setTexCoordArray(0,qTexCoords);

    m_state = new osg::StateSet;
    m_state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    m_geode->addDrawable(m_geom);
    m_pat->addChild(m_geode);

    m_name = "NULL";
    m_filetype = "NULL";
    m_sourcetype = "NULL";
    m_LOD ="NULL";

    m_displayable = true;
    m_requested = true;

}

osgInfo::osgInfo(float height, float width, osg::Vec3 pos, double ang, osg::Vec3 axis, std::string filepath, std::string name, std::string type, std::string source, std::string lod)
{
    m_texture = new osg::Texture2D;
    m_geom = new osg::Geometry;
    m_state = new osg::StateSet;
    m_geode = new osg::Geode;
    m_pat = new osg::PositionAttitudeTransform;

    m_position = pos;
    m_angle = ang;
    m_axe = axis;

    m_name = name;
    m_filetype = type;
    m_sourcetype = source;
    m_LOD = lod;

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

    m_state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    m_state->setTextureAttributeAndModes(0, m_texture, osg::StateAttribute::ON );

    //m_geom->setStateSet(m_state);

    m_geode->addDrawable(m_geom);

    m_pat->setStateSet(m_state);
    m_pat->setPosition(m_position);
    m_pat->setAttitude(osg::Quat(osg::DegreesToRadians(m_angle), m_axe));

    m_pat->addChild(m_geode);

    m_displayable = true;
    m_requested = true;


    //id = boost::uuids::random_generator()();


}
////////////////////////////////////////////////////////////////////////
/// \brief Angle getter
/// \return
///
double osgInfo::getAngle()
{
    return m_angle;
}
////////////////////////////////////////////////////////////////////////
/// \brief Position getter
/// \return
///
osg::Vec3 osgInfo::getPosition()
{
    return m_position;
}
////////////////////////////////////////////////////////////////////////
/// \brief Axe getter
/// \return
///
osg::Vec3 osgInfo::getAxis()
{
    return m_axe;
}
////////////////////////////////////////////////////////////////////////
/// \brief Geometry getter
/// \return
///
osg::Geometry* osgInfo::getGeom()
{
    return m_geom;
}
////////////////////////////////////////////////////////////////////////
/// \brief Texture getter
/// \return
///
osg::Texture2D* osgInfo::getTexture()
{
    return m_texture;
}
////////////////////////////////////////////////////////////////////////
/// \brief State getter
/// \return
///
osg::StateSet* osgInfo::getState()
{
    return m_state;
}

////////////////////////////////////////////////////////////////////////
/// \brief geode getter
/// \return
///
osg::Geode* osgInfo::getGeode()
{
    return m_geode;
}

////////////////////////////////////////////////////////////////////////
/// \brief pat getter
/// \return
///
osg::PositionAttitudeTransform* osgInfo::getPAT()
{
    return m_pat;
}

////////////////////////////////////////////////////////////////////////
/// \brief name getter
/// \return
///
std::string osgInfo::getInfoName()
{
    return m_name;
}

////////////////////////////////////////////////////////////////////////
/// \brief type getter
/// \return
///
std::string osgInfo::getType()
{
    return m_filetype;
}

////////////////////////////////////////////////////////////////////////
/// \brief source getter
/// \return
///
std::string osgInfo::getSourceType()
{
    return m_sourcetype;
}

////////////////////////////////////////////////////////////////////////
/// \brief source getter
/// \return
///
std::string osgInfo::getInfoLOD()
{
    return m_LOD;
}

////////////////////////////////////////////////////////////////////////
/// \brief displayable getter
/// \return
///
bool osgInfo::isDisplayable()
{
    return m_displayable;
}

////////////////////////////////////////////////////////////////////////
/// \brief requested getter
/// \return
///
bool osgInfo::isRequested()
{
    return m_requested;
}

////////////////////////////////////////////////////////////////////////
/// \brief Axis setter
/// \param newAxis
///
void osgInfo::setAxis(osg::Vec3 newAxis)
{
    m_axe=newAxis;
    //TODO : void update pat

}
////////////////////////////////////////////////////////////////////////
/// \brief Position setter
/// \param newPos
///
void osgInfo::setPosition(osg::Vec3 newPos)
{
    m_position=newPos;
    //TODO : void update pat

}
////////////////////////////////////////////////////////////////////////
/// \brief Angle setter
/// \param newAngle
///
void osgInfo::setAngle(float newAngle)
{
    m_angle=newAngle;
    //TODO : void update pat

}
////////////////////////////////////////////////////////////////////////
/// \brief Height setter
/// \param newHeight
///
void osgInfo::setHeight(float newHeight)
{
    m_height=newHeight;
    //TODO : void update geom

}

////////////////////////////////////////////////////////////////////////
/// \brief Width setter
/// \param newWidth
///
void osgInfo::setWidth(float newWidth)
{
    m_width=newWidth;
    //TODO : void update geom

}

void osgInfo::setTexture(std::string filepath)
{
    m_texture->setImage(osgDB::readImageFile(filepath));

    m_state->setTextureAttributeAndModes(0, m_texture, osg::StateAttribute::ON );

}

void osgInfo::setDisplayable(bool statut)
{
    m_displayable=statut;
}

void osgInfo::setRequested(bool statut)
{
    m_requested=statut;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
