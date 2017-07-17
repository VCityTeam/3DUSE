// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
#include <osg/Point>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osg/BlendFunc>
#include <osg/Material>
#include "vecs.hpp"
#include <ctime>


class osgInfo : public osg::Group
{
public:
    osgInfo();
    /// \brief Details of a document
    /// \param height of the document
    /// \param width of the document
    /// \param position of the document in EPSG: 3946
    /// \param angle or orientation of document
    /// \param axis
    /// \param filepath of the document
    /// \param name of the document
    /// \param type of the document (official etc.)
    /// \param source of the document
    /// \param lod or level of detail
    /// \param anchor (at what height)
    /// \param priority of the document
    /// \param publicationDate of the document
    osgInfo(float height, float width, osg::Vec3 position, double angle, osg::Vec3 axis, std::string filepath, std::string name, std::string type,
                std::string source, std::string lod, float anchor, int priority, std::string publicationDate);

    /// \brief Turn fixed document into billboard or reverse operation
    /// \param boolean value to set billboard mode or not
    void setBillboarding(bool option);

    /// \brief Update roation axis of billboard object
    /// \param osg Vec3 new rotation for billboard
    void setBBAxis(osg::Vec3 newAxis);

    /// \brief Update scale of current document
    /// \param int screenX width of screen (in pixels)
    /// \param int screenY height of screen (in pixels)
    void updateScale( int screenX, int screenY);

    /// \brief Compute displayability of current info
    void updateDisplayability();

    /// \brief Update the 3 local vectors
    /// \param osg::Vec3f normale of the document
    /// \param osg::Vec3f rotation axis of the document
    /// \param osg::Vec3f cross product of normale and axis
    void UpdateTetra(osg::Vec3f normale, osg::Vec3f axis, osg::Vec3f ortho);

    /// \brief Update anchoring line upper coordinate according to new document position
    /// \param float new z value
    void UpdateAnchoringLine(float newZ);

    /// \brief Update current document position
    /// \param osg::Vec3 new coordinates
    void UpdatePosition(osg::Vec3 newPos);

    /// \brief Display red point objects at upper and lower cornes of info
    /// \param osg::Vec3 upperCorner
    /// \param osg::Vec3 lowerCorner
    /// \param osgInfo* concerned info
    void displayRedCorners(osg::Vec3 upperCorner, osg::Vec3 lowerCorner);

    /// \brief Compute and update DSC value for current document
    /// \param osg::Camera* pointer to camera to get camera current position
    /// \param int screenX width of screen (in pixels)
    /// \param int screenY height of screen (in pixels)
    void computeDSC(osg::Camera *cam, int screenX, int screenY);

    /// \brief Compute and update DCAM value for current document
    /// \param osg::Camera* pointer to camera to get camera current position
    void computeDCAM(osg::Camera *cam);

    void computeOVaMatrix(std::vector< std::vector<float> > screen);

    void computeOVas(std::vector< std::vector<float> > screen);

    // Setters

    /// \brief Update document rotation axis
    /// \param osg::Vec3 new axis
    void setAxis(osg::Vec3 newAxis);

    /// \brief Update document angle
    /// \param float new angle
    void setAngle(float newAngle);

    /// \brief Update document height
    /// \param float new height
    void setHeight(float newHeight);

    /// \brief Update document width
    /// \param float width
    void setWidth(float newWidth);

    /// \brief Update distance between document center and camera position
    /// \param float new distance
    void setDCAM(float newDist);

    /// \brief Update distance between document center and screen center
    /// (2D screen coordinates)
    /// \param float new distance
    void setDSC(float newScreenDist);

    /// \brief Update anchoring line lower coordinate
    /// i.e the highest point on the map underneath the document
    /// \param float new z value
    void setAnchoringPoint(float altitude);

    /// \brief Update texture mapped on document geometry
    /// \param string filepath of the image
    void setTexture(std::string filepath);

    /// \brief Update document transparency
    /// \param float alpha value [0,1]
    void setTransparency(float alpha);

    /// \brief Update m_displayability if document needs to be displayed
    /// \param boolean value
    void setDisplayable(bool statut);

    /// \brief Update m_requested if filter bar is used and request a document
    /// \param boolean value
    void setRequested(bool statut);

    /// \brief Update m_onscreen to tell if a document is actually displayed
    /// \param boolean value
    void setonScreen(bool statut);

    /// \brief Update set current document area on screen
    /// \param float new area
    void setDa(float area);

    // Getters

    /// \brief Get current document position
    /// \return osg::Vec3 m_currentposition
    osg::Vec3 getPosition();

    /// \brief Get distance between document center and camera position
    /// \return float m_DCAM
    float getDCAM();

    /// \brief Get distance between document center and screen center
    /// \return float m_DSC
    float getDSC();

    /// \brief Get group node storing sub-nodes
    /// One group for one document and its line
    /// \return osg::Group m_group
    osg::Group *getGroup();

    /// \brief Get document name
    /// \return std::string m_name
    std::string getInfoName();

    /// \brief Get file type (image, pdf, ...)
    /// \return std::string m_filetype
    std::string getType();

    /// \brief Get document source type (official, user, ..)
    /// \return std::string m_sourcetype
    std::string getSourceType();

    /// \brief Get document level of detail
    /// \return std::string m_LOD
    std::string getInfoLOD();

    /// \brief Get displayability
    /// \return boolean is_displayable
    bool isDisplayable();

    /// \brief Get is_requested value
    /// \return boolean is_requested
    bool isRequested();

    /// \brief Get m_onscreen value
    /// \return boolean m_onscreen
    bool isonScreen();

private:

    ///Document altitude according to their LOD
    unsigned int streetZ = 30;
    unsigned int buildingZ = 100;
    unsigned int districtZ = 200;
    unsigned int cityZ = 500;

    bool m_displayable; /// is this document displayable
    bool m_requested; /// is this document requested by the filter bar
    bool m_onscreen; /// is this document on screen


    osg::Texture2D *m_texture ; ///texture of the doc
    osg::Material *m_material ; ///material of the doc
    osg::StateSet *m_state; ///state of the doc
    osg::Geometry  *m_quad; ///geometry to store quad drawable
    osg::Geode *m_geode; ///geometry node
    osg::Group *m_group; ///group node to store quad and line
    osg::PositionAttitudeTransform *m_pat; ///position attitude transforme of the node

public :
    float m_height; ///height of the doc
    float m_width; ///width of the doc

    osg::Vec3 m_initposition ; ///initial position of the node
    osg::Vec3 m_currentposition ; ///current position of the node
    double m_angle ; ///rotation angle of the node
    osg::Vec3 m_axe ; ///rotation axis of the node

    std::string m_filepath ; ///level of detail to display (street, building, district, city)
    std::string m_name ; ///name of the info doc

    std::string m_filetype; ///type of file (image, doc...)
    std::string m_sourcetype; ///source of file (official, user..)
    std::string m_LOD ; ///level of detail to display (street, building, district, city)

    float m_anchoring ; ///highest point on the map underneath the document
    int m_priority; /// level of importance of the document

    float m_DCAM ; ///distance between doc and cam
    float m_DSC ; /// distance between doc and screen center
    int m_Da; ///document area on screen
    float m_initOVa; ///total area of document overlapped by others in front of it in its init position
    float m_currentOVa; ///total area of document overlapped by others in front of it in its current position

    std::string m_publicationDate;

    osg::Billboard *m_billboard; ///billboard object is needed

    osg::Vec3 m_currentsCornerMax; ///screen coordinates of max corner
    osg::Vec3 m_currentsCornerMin; /// screen coordinates of min corner

    osg::Vec3 m_initsCornerMax; ///screen coordinates of max corner
    osg::Vec3 m_initsCornerMin; /// screen coordinates of min corner

    std::vector< std::vector<float> > m_OVaMatrix ;

    osg::Geode *m_tetra; ///geometry node to display local vector

};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGINFO_HPP__

