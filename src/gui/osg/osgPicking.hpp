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

    /*void setLabel(const std::string& name)
    {
        appGui().getTextBrowser()->setText(name.c_str());
    }

    void setLabel(citygml::CityObject* node)
    {
        if(node)
        {
            std::stringstream ss;
            ss << node->getId().c_str() << std::endl;
            citygml::AttributesMap attribs = node->getAttributes();
            citygml::AttributesMap::const_iterator it = attribs.begin();
            while( it != attribs.end())
            {
                ss << "  + " << it->first << ": " << it->second << std::endl;
                it++;
            }
            appGui().getTextBrowser()->setText(ss.str().c_str());
        }
    }*/

    const std::set<std::string>& getNodesPicked() const
    {
        return m_nodesPicked;
    }

    const std::set<std::string>& getNodesPickedURI() const
    {
        return m_nodesPicked;
    }

    //const std::string& getNodePicked() const;

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

    void addNodePicked(const std::string& name); // remove
    void addNodePicked(const vcity::URI& uri); // remove
    void removeNodePicked(const std::string& name); // remove

    void addNodePicked(osg::ref_ptr<osg::Node> node); // remove
    void removeNodePicked(osg::ref_ptr<osg::Node> node); // remove

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

    std::set<std::string> m_nodesPicked;    // remove
    std::set<vcity::URI> m_nodesPickedURI;  // remove
    std::vector<osg::ref_ptr<osg::Node> > m_osgNodesPicked; // remove
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGPICKING_HPP__
