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

    m_distancetocam = 0 ;

    osg::Vec3Array* qVertices = new osg::Vec3Array;
    qVertices->push_back( osg::Vec3( -m_width/2, 0, -m_height/2) ); // bottom left
    qVertices->push_back( osg::Vec3(m_width/2, 0, -m_height/2) ); // bottom right
    qVertices->push_back( osg::Vec3(m_width/2,0, m_height/2) ); // top right
    qVertices->push_back( osg::Vec3(-m_width/2,0, m_height/2) ); // top left

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

    m_state->setTextureAttributeAndModes(0, m_texture, osg::StateAttribute::ON );
    //m_state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    //Create a material for the geometry.
    m_material = new osg::Material;
    m_material->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    m_material->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    m_material->setAlpha(osg::Material::FRONT_AND_BACK, 1);


    m_state->setAttribute(m_material,osg::StateAttribute::ON);
    m_state->setMode(GL_BLEND, osg::StateAttribute::ON);

    osg::BlendFunc* blend = new osg::BlendFunc;
    blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_DST_ALPHA);

    //m_geom->setStateSet(m_state);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    vertices->push_back( osg::Vec3( m_position.x(), m_position.y(), m_position.z()-m_height/2) );
    vertices->push_back( osg::Vec3( m_position.x(), m_position.y(), 180 ));

    indices->push_back(0);
    indices->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geom->setColorArray(color, osg::Array::BIND_OVERALL);

    geode->addDrawable(geom);




    m_geode->addDrawable(m_geom);

    m_pat->setStateSet(m_state);
    m_pat->setPosition(m_position);
    m_pat->setAttitude(osg::Quat(osg::DegreesToRadians(m_angle), m_axe));

    m_billboard = new osg::Billboard();
    //osg::Drawable* billboardDrawable = m_geom;
    m_billboard->addDrawable(m_geom, osg::Vec3(0,0,0));
    //m_pat->addChild(m_billboard);

    m_group = new osg::Group;

    m_group->addChild(m_pat);
    //m_group->addChild(geode);

    m_pat->addChild(m_geode);


    m_displayable = true;
    m_requested = true;


    //id = boost::uuids::random_generator()();


}

void osgInfo::BillboardOFF()
{
    m_pat->removeChild(m_billboard);
    m_pat->addChild(m_geode);
}

void osgInfo::BillboardON()
{
    m_pat->removeChild(m_geode);
    m_pat->addChild(m_billboard);
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
/// \brief Distance to cam getter
/// \return
///
float osgInfo::getDistancetoCam()
{
    return m_distancetocam;
}

////////////////////////////////////////////////////////////////////////
/// \brief pat getter
/// \return
///
osg::Group* osgInfo::getPAT()
{
    return m_group;
    //return m_pat;
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


////////////////////////////////////////////////////////////////////////
/// \brief Distance to cam setter
/// \param newDist
///
void osgInfo::setDistancetoCam(float newDist)
{
    m_distancetocam=newDist;
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

void osgInfo::setTransparency(float alpha)
{
   m_material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
}


void osgInfo::Scaling(float scale)
{
   //m_width*=scale;
   //m_height*=scale;


   scale = 2*(1-scale);
   m_pat->setScale(osg::Vec3(scale, 1, scale));


}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
