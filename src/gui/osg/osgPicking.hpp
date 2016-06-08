 // -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGPICKING_HPP__
#define __OSGPICKING_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>
#include <QTextBrowser>
#include <QTreeWidget>
#include "core/scene.hpp"
#include "gui/applicationGui.hpp"
////////////////////////////////////////////////////////////////////////////////
// class to handle events with a pick
/*class PickHandler : public osgGA::GUIEventHandler
{
public:
    PickHandler() {}
    ~PickHandler() {}

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

    void setLabel(const std::string& name)
    {
        if(m_updateText)
        {
            m_updateText->setText(name.c_str());
        }
    }

    void setPickHandlerTextBox(QTextBrowser* updateText)
    {
        m_updateText = updateText;
    }

protected:
    QTextBrowser* m_updateText;
};*/
////////////////////////////////////////////////////////////////////////////////
class PickHandler : public osgGA::GUIEventHandler
{
public:
    PickHandler();

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    void setPickingMode(int mode);

    void resetPicking();

public:
    /// \brief pickPoint Picking with a point click
    /// \param ea
    /// \param viewer
    void pickPoint(const osgGA::GUIEventAdapter& ea, osgViewer::View* viewer);
    void pickRectangle(const osgGA::GUIEventAdapter& ea, osgViewer::View* viewer);

    /// \brief toggleSelected Toggle selection of a node
    /// \param node
    /// \param parent
    /// \param forceUnselect
    void toggleSelected(osg::Node* node, osg::Group* parent = NULL, bool forceUnselect = false);

    /// \brief toggleSelected Toggle selection of a node
    /// \param uri URI pointing to the node
    void toggleSelected(const vcity::URI& uri);

    void selectNode(const vcity::URI& uri);

    void deselectNode(const vcity::URI& uri);


    void updateLabel(const vcity::URI& uri); // remove

    /// \brief resetSelection Reset selection. Deselect all nodes
    void resetSelection();

    /// \brief addSelection Add a node to selection
    /// \param uri URI pointing to the node
    void addSelection(const vcity::URI& uri);

protected:
    float m_mx, m_my;

    int m_pickingMode;              ///< 0: face, 1: building
    bool m_addToSelection;          ///< are we pressing ctrl key ?
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGPICKING_HPP__
