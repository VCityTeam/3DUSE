// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGINFO_HPP__
#define __OSGINFO_HPP__

////////////////////////////////////////////////////////////////////////////////
/// \brief Info class to manage doc and img
////////////////////////////////////////////////////////////////////////////////

#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/Billboard>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osg/BlendFunc>
#include <osg/Material>


class osgInfo : public osg::Group
{
public:
    osgInfo();
    osgInfo(float height, float width, osg::Vec3 position, double angle, osg::Vec3 axis, std::string filepath, std::string name, std::string type, std::string source, std::string lod, float anchor);

    void BillboardOFF();///use geode
    void BillboardON();///use billboard
    void Scaling(float scale);

    // Setters
    void setPosition(osg::Vec3 newPos);
    void setAxis(osg::Vec3 newAxis);

    void setAngle(float newAngle);
    void setHeight(float newHeight);
    void setWidth(float newWidth);
    void setDistancetoCam(float newDist);
    void setDistancetoSC(float newScreenDist);
    void setAnchoringPoint(float altitude);

    void setTexture(std::string filepath);
    void setTransparency(float alpha);
    void setDisplayable(bool statut);
    void setRequested(bool statut);
    void setonScreen(bool statut);

    // Getters
    osg::Vec3 getPosition();
    float getDistancetoCam();
    float getDistancetoSC();

    osg::Group *getPAT();

    std::string getInfoName();
    std::string getType();
    std::string getSourceType();
    std::string getInfoLOD();

    bool isDisplayable();
    bool isRequested();
    bool isonScreen();

private:

    float m_distancetocam ; ///distance between doc and cam
    float m_distancetoSC ; /// distance between doc and screen center


    unsigned int m_date_deb ;
    unsigned int m_date_fin ;

    bool m_displayable;
    bool m_requested;
    bool m_onscreen;


    osg::Texture2D *m_texture ; ///texture of the doc
    osg::Material *m_material ; ///material
    osg::StateSet *m_state; ///state of the doc
    osg::Geometry  *m_geom; ///geometry instance for the node
    osg::Geode *m_geode; ///geometry node
    osg::Group *m_group; ///group node
    osg::Billboard *m_billboard; ///billboard
    osg::PositionAttitudeTransform *m_pat; ///position attitude transforme of the node

public :
    float m_height; ///height of the doc
    float m_width; ///width of the doc

    osg::Vec3 m_position ; ///position of the node
    double m_angle ; ///rotation angle of the node
    osg::Vec3 m_axe ; ///rotation axis of the node

    std::string m_filepath ; ///level of detail to display (street, building, district, city)
    std::string m_name ; ///name of the info doc

    std::string m_filetype; ///type of file (image, doc...)
    std::string m_sourcetype; ///source of file (official, user..)
    std::string m_LOD ; ///level of detail to display (street, building, district, city)

    float m_anchoring ;


};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGINFO_HPP__

