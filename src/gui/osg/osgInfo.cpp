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

osgInfo::osgInfo(float height, float width, osg::Vec3 pos, double ang, osg::Vec3 axis, std::string filepath, std::string name, std::string type,
                 std::string source, std::string lod, float anchor, int priority,   time_t publicationDate)
{
    m_texture = new osg::Texture2D;
    m_geom = new osg::Geometry;
    m_state = new osg::StateSet;
    m_geode = new osg::Geode;
    m_pat = new osg::PositionAttitudeTransform;

    m_initposition = pos;
    m_currentposition = pos;
    m_angle = ang;
    m_axe = axis;

    m_name = name;
    m_filetype = type;
    m_sourcetype = source;
    m_LOD = lod;
    m_filepath=filepath;

    m_texture->setImage(osgDB::readImageFile(filepath));
    m_height = height;
    m_width = width;

    m_anchoring = anchor ;
    m_publcationDate = publicationDate;




    m_priority = priority;

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

    //Create a material for the geometry.
    m_material = new osg::Material;
    m_material->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    m_material->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    m_material->setAlpha(osg::Material::FRONT_AND_BACK, 1);


    m_state->setAttribute(m_material,osg::StateAttribute::ON);
    m_state->setMode(GL_BLEND, osg::StateAttribute::ON);

    osg::BlendFunc* blend = new osg::BlendFunc;
    blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_DST_ALPHA);


    // ANCHORING LINE
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    vertices->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()-m_height/2) );
    vertices->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_anchoring ));

    indices->push_back(0);
    indices->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geom->setColorArray(color, osg::Array::BIND_OVERALL);

    geode->addDrawable(geom);

    /******************* TETRAEDRE **************************/

    m_tetra = new osg::Geode;

            // TETRA LINE 1

    osg::Geometry* line1 = new osg::Geometry;
    osg::Vec3Array* verticesL1 = new osg::Vec3Array;
    osg::DrawElementsUInt* indicesL1 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    verticesL1->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2));
    verticesL1->push_back( osg::Vec3( m_initposition.x()+20, m_initposition.y(), m_initposition.z()+m_height/2));

    indicesL1->push_back(0);
    indicesL1->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color1 = new osg::Vec4Array;
    color1->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    line1->setVertexArray(verticesL1);
    line1->addPrimitiveSet(indicesL1);
    line1->setColorArray(color1, osg::Array::BIND_OVERALL);

            // TETRA LINE 2

    osg::Geometry* line2 = new osg::Geometry;
    osg::Vec3Array* verticesL2 = new osg::Vec3Array;
    osg::DrawElementsUInt* indicesL2 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    verticesL2->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2));
    verticesL2->push_back( osg::Vec3( m_initposition.x(), m_initposition.y()+20, m_initposition.z()+m_height/2));

    indicesL2->push_back(0);
    indicesL2->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color2 = new osg::Vec4Array;
    color2->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    line2->setVertexArray(verticesL2);
    line2->addPrimitiveSet(indicesL2);
    line2->setColorArray(color2, osg::Array::BIND_OVERALL);

            // TETRA LINE 3

    osg::Geometry* line3 = new osg::Geometry;
    osg::Vec3Array* verticesL3 = new osg::Vec3Array;
    osg::DrawElementsUInt* indicesL3 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    verticesL3->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2 ));
    verticesL3->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2+20 ));

    indicesL3->push_back(0);
    indicesL3->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color3 = new osg::Vec4Array;
    color3->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    line3->setVertexArray(verticesL3);
    line3->addPrimitiveSet(indicesL3);
    line3->setColorArray(color3, osg::Array::BIND_OVERALL);


    m_tetra->addDrawable(line1);
    m_tetra->addDrawable(line2);
    m_tetra->addDrawable(line3);




    m_geode->addDrawable(m_geom);

    m_pat->setStateSet(m_state);
    m_pat->setPosition(m_initposition);
    m_pat->setAttitude(osg::Quat(osg::DegreesToRadians(m_angle), m_axe));

    m_billboard = new osg::Billboard();

    m_billboard->addDrawable(m_geom, osg::Vec3(0,0,0));
    m_billboard->setAxis(osg::Vec3(0,0,1));
    //m_billboard->setMode(osg::Billboard::POINT_ROT_WORLD);


    m_group = new osg::Group;

    m_group->addChild(m_pat);
    m_group->addChild(geode);
    //m_group->addChild(m_tetra);

    m_pat->addChild(m_geode);


    m_displayable = true;
    m_requested = true;
    m_onscreen = false;

    m_DCAM = 0.0f;
    m_DSC = 0.0f;
    m_Da= 0.0f;
    m_OVa = 0.0f;


}

void osgInfo::setBillboarding(bool option)
{
    if(option)
    {
        m_pat->removeChild(m_geode);
        m_pat->addChild(m_billboard);
    }
    else
    {
        m_pat->removeChild(m_billboard);
        m_pat->addChild(m_geode);
    }
}

void osgInfo::UpdateTetra(osg::Vec3f normale, osg::Vec3f axis, osg::Vec3f ortho)
{

    //// TETRA LINE 1

    osg::Geometry* newline1 = new osg::Geometry;
    osg::Vec3Array* verticesL1 = new osg::Vec3Array;
    osg::DrawElementsUInt* indicesL1 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    verticesL1->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2));
    verticesL1->push_back(osg::Vec3(m_initposition.x()+ortho.x()*20, m_initposition.y()+ortho.y()*20, m_initposition.z()+m_height/2+ortho.z()*20));

    indicesL1->push_back(0);
    indicesL1->push_back(1);

    osg::ref_ptr<osg::Vec4Array> colorortho = new osg::Vec4Array;
    colorortho->push_back(osg::Vec4(1.0,0.0,0.0,1.0));

    newline1->setVertexArray(verticesL1);
    newline1->addPrimitiveSet(indicesL1);
    newline1->setColorArray(colorortho, osg::Array::BIND_OVERALL);

    //// TETRA LINE 2
    osg::Geometry* newline2 = new osg::Geometry;
    osg::Vec3Array* verticesL2 = new osg::Vec3Array;
    osg::DrawElementsUInt* indicesL2 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES);

    verticesL2->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2));
    verticesL2->push_back(osg::Vec3(m_initposition.x()+normale.x()*20, m_initposition.y()+normale.y()*20, m_initposition.z()+m_height/2+normale.z()*20));

    indicesL2->push_back(0);
    indicesL2->push_back(1);

    osg::ref_ptr<osg::Vec4Array> colornormale = new osg::Vec4Array;
    colornormale->push_back(osg::Vec4(0.0,1.0,0.0,1.0));

    newline2->setVertexArray(verticesL2);
    newline2->addPrimitiveSet(indicesL2);
    newline2->setColorArray(colornormale, osg::Array::BIND_OVERALL);

    //// TETRA LINE 3
    osg::Geometry* newline3 = new osg::Geometry;
    osg::Vec3Array* verticesL3 = new osg::Vec3Array;
    osg::DrawElementsUInt* indicesL3 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    verticesL3->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()+m_height/2));
    verticesL3->push_back( osg::Vec3(m_initposition.x()+axis.x()*20, m_initposition.y()+axis.y()*20, m_initposition.z()+m_height/2+axis.z()*20));

    indicesL3->push_back(0);
    indicesL3->push_back(1);

    osg::ref_ptr<osg::Vec4Array> coloraxis = new osg::Vec4Array;
    coloraxis->push_back(osg::Vec4(0.0,0.0,1.0,1.0));

    newline3->setVertexArray(verticesL3);
    newline3->addPrimitiveSet(indicesL3);
    newline3->setColorArray(coloraxis, osg::Array::BIND_OVERALL);

    m_tetra->removeDrawables(0,3);
    m_tetra->addDrawable(newline1);
    m_tetra->addDrawable(newline2);
    m_tetra->addDrawable(newline3);


}

void osgInfo::UpdateAnchoringLine(float newZ)
{
    m_group->removeChild(1);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    vertices->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), newZ-m_height/2) );
    vertices->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_anchoring ));

    indices->push_back(0);
    indices->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geom->setColorArray(color, osg::Array::BIND_OVERALL);

    geode->addDrawable(geom);
    m_group->addChild(geode);
}

void osgInfo::UpdatePosition(osg::Vec3 newPos)
{
    m_currentposition=newPos;
    UpdateAnchoringLine(newPos.z());
}

osg::Vec3 osgInfo::getPosition()
{
    return m_currentposition;
}

float osgInfo::getDCAM()
{
    return m_DCAM;
}

float osgInfo::getDSC()
{
    return m_DSC;
}

osg::Group* osgInfo::getGroup()
{
    return m_group;
    //return m_pat;
}

std::string osgInfo::getInfoName()
{
    return m_name;
}

std::string osgInfo::getType()
{
    return m_filetype;
}

std::string osgInfo::getSourceType()
{
    return m_sourcetype;
}

std::string osgInfo::getInfoLOD()
{
    return m_LOD;
}

bool osgInfo::isDisplayable()
{
    return m_displayable;
}

bool osgInfo::isRequested()
{
    return m_requested;
}

bool osgInfo::isonScreen()
{
    return m_onscreen;
}

void osgInfo::setAxis(osg::Vec3 newAxis)
{
    m_axe=newAxis;
    //TODO : void update pat

}

void osgInfo::setAngle(float newAngle)
{
    m_angle=newAngle;
    //TODO : void update pat

}

void osgInfo::setHeight(float newHeight)
{
    m_height=newHeight;
    //TODO : void update geom

}

void osgInfo::setWidth(float newWidth)
{
    m_width=newWidth;
    //TODO : void update geom

}

void osgInfo::setDCAM(float newDist)
{
    m_DCAM=newDist;

}

void osgInfo::setDSC(float newScreenDist)
{
    m_DSC=newScreenDist;

}

void osgInfo::setAnchoringPoint(float altitude)
{
    m_anchoring=altitude;
    m_group->removeChild(1);

    if(m_LOD=="street")
        m_initposition.z()=m_anchoring+streetZ;
    if(m_LOD=="building")
        m_initposition.z()=m_anchoring+buildingZ;
    if(m_LOD=="district")
        m_initposition.z()=m_anchoring+districtZ;
    if(m_LOD=="city")
        m_initposition.z()=m_anchoring+cityZ;

    m_pat->setPosition(m_initposition);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    vertices->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_initposition.z()-m_height/2) );
    vertices->push_back( osg::Vec3( m_initposition.x(), m_initposition.y(), m_anchoring ));

    indices->push_back(0);
    indices->push_back(1);

    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0,1.0,1.0,1.0));

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geom->setColorArray(color, osg::Array::BIND_OVERALL);

    geode->addDrawable(geom);
    m_group->addChild(geode);

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

void osgInfo::setonScreen(bool statut)
{
    m_onscreen=statut;
}

void osgInfo::setDa(float area)
{
    m_Da=area;
}

void osgInfo::setTransparency(float alpha)
{
   m_material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
}

void osgInfo::Scaling(float scale)
{
   scale = 2*(1-scale);
   m_pat->setScale(osg::Vec3(scale, 1, scale));
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
