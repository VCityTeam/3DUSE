#ifndef __OSGPICKING_HPP__
#define __OSGPICKING_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>
#include <QTextBrowser>
#include <QTreeWidget>
#include "core/scene.hpp"
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

    void setPickHandlerTextBox(QTextBrowser* updateText)
    {
        m_updateText = updateText;
    }

    void setPickHandlerScene(vcity::Scene* scene)
    {
        m_scene = scene;
    }

    void setPickHandlerTreeView(QTreeWidget* tree)
    {
        m_tree = tree;
    }


    void setLabel(const std::string& name)
    {
        if(m_updateText)
        {
            m_updateText->setText(name.c_str());
        }
    }

    void setLabel(citygml::CityObject* node)
    {
        if(m_updateText && node)
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
            m_updateText->setText(ss.str().c_str());
        }
    }

    const std::set<std::string>& getNodesPicked() const
    {
        return m_nodesPicked;
    }

    //const std::string& getNodePicked() const;

    void setPickingMode(int mode);

    void resetPicking();

protected:
    void pickPoint(const osgGA::GUIEventAdapter& ea, osgViewer::View* viewer);
    void pickRectangle(const osgGA::GUIEventAdapter& ea, osgViewer::View* viewer);
    void toggleSelected(osg::Node* node, osg::Group* parent = NULL, bool forceUnselect = false);

    void addNodePicked(const std::string& name);
    void removeNodePicked(const std::string& name);

    void addNodePicked(osg::ref_ptr<osg::Node> node);
    void removeNodePicked(osg::ref_ptr<osg::Node> node);


    void updateLabel();

    float m_mx, m_my;

    int m_pickingMode;  ///< 0: face, 1: building
    bool m_addToSelection;

    QTextBrowser* m_updateText;
    vcity::Scene* m_scene;
    QTreeWidget* m_tree;

    std::set<std::string> m_nodesPicked;
    std::vector<osg::ref_ptr<osg::Node> > m_osgNodesPicked;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGPICKING_HPP__
