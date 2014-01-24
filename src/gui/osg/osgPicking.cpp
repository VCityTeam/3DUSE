////////////////////////////////////////////////////////////////////////////////
#include "osgPicking.hpp"
#include "osgTools.hpp"
#include <osgFX/Scribe>
#include <iostream>
#include <algorithm>
#include "gui/moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
PickHandler::PickHandler()
    : m_mx(0.0),m_my(0.0), m_pickingMode(1), m_addToSelection(false)
{
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::addNodePicked(const std::string& name)
{
    m_nodesPicked.insert(name);
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::addNodePicked(const vcity::URI& uri)
{
    osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);
    if(node)
    {
        addNodePicked(node);
        toggleSelected(node);
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::removeNodePicked(const std::string& name)
{
    m_nodesPicked.erase(name);
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::addNodePicked(osg::ref_ptr<osg::Node> node)
{
    std::vector<osg::ref_ptr<osg::Node> >::iterator it = std::find(m_osgNodesPicked.begin(), m_osgNodesPicked.end(), node);
    if(it == m_osgNodesPicked.end())
    {
        // if not found, ok, add
        m_osgNodesPicked.push_back(node);
        addNodePicked(node->getName());
    }
    else
    {
        // already picked, need to unpick
        m_osgNodesPicked.erase(it);
        removeNodePicked(node->getName());
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::removeNodePicked(osg::ref_ptr<osg::Node> /*node*/)
{

}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::resetPicking()
{
    std::vector<osg::ref_ptr<osg::Node> >::const_iterator it = m_osgNodesPicked.begin();
    for(; it != m_osgNodesPicked.end(); ++it)
    {
        toggleSelected(*it);
    }

    m_osgNodesPicked.clear();
    m_nodesPicked.clear();
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::updateLabel()
{
    std::stringstream ss;

    std::vector<osg::ref_ptr<osg::Node> >::const_iterator it = m_osgNodesPicked.begin();
    for(; it != m_osgNodesPicked.end(); ++it)
    {
        //citygml::CityObject* obj = m_scene->findNode((*it)->getName());
        vcity::URI uri = osgTools::getURI(*it);
        citygml::CityObject* obj = appGui().getScene().getNode(uri);
        if(obj)
        {
            ss << obj->getId().c_str() << std::endl;
            citygml::AttributesMap attribs = obj->getAttributes();
            citygml::AttributesMap::const_iterator itAttr = attribs.begin();
            while( itAttr != attribs.end())
            {
                ss << "  + " << itAttr->first << ": " << itAttr->second << std::endl;
                ++itAttr;
            }
            ss << uri.getStringURI(true);
        }
    }

    appGui().getTextBrowser()->setText(ss.str().c_str());
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::updateLabel(const vcity::URI& uri)
{
    appGui().getMainWindow()->updateTextBox(uri);
}
////////////////////////////////////////////////////////////////////////////////
//osg::Node* nodePicked = 0;
bool PickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    osgViewer::View* viewer = dynamic_cast<osgViewer::View*>(&aa);
    if (!viewer) return false;

    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::PUSH):
        {
            m_mx = ea.getX();
            m_my = ea.getY();
            return false;
        }
        case(osgGA::GUIEventAdapter::MOVE):
        /*{
            m_mx = ea.getX();
            m_my = ea.getY();
            return false;
        }*/
            break;
        case(osgGA::GUIEventAdapter::RELEASE):
        {
            if((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) != 0)
            {
                std::cout << "start pick add" << std::endl;
                m_addToSelection = true;
            }
            else
            {
                std::cout << "stop pick add" << std::endl;
                m_addToSelection = false;
            }

            if(m_mx == ea.getX() && m_my == ea.getY())
            {
                // only do a pick if the mouse hasn't moved
                // TODO: add a little headroom so the mouse does not have to stand perfectly still.
                std::cout << "Pick point : (" << m_mx << ", " << m_my << ")" << std::endl;
                pickPoint(ea,viewer);
            }
            else
            {
                std::cout << "Pick rectangle : (" << m_mx << ", " << m_my << "), (" << ea.getX() << ", " << ea.getY() << ")"  << std::endl;
                std::cout << "Pick rectangle : (" << m_mx << ", " << m_my << "), (" << ea.getXnormalized() << ", " << ea.getYnormalized() << ")"  << std::endl;
                //pickRectangle(ea,viewer);
            }
            return true;
        }
        case(osgGA::GUIEventAdapter::KEYDOWN):
       {
          switch(ea.getKey())
          {
          case osgGA::GUIEventAdapter::KEY_Plus:
          case osgGA::GUIEventAdapter::KEY_KP_Add:
          case osgGA::GUIEventAdapter::KEY_P:
             std::cout << "pick parent" << std::endl;

             if(m_osgNodesPicked.size() > 0)
             {
                 toggleSelected(m_osgNodesPicked[0]); // unselect last node
                 m_osgNodesPicked[0] = (m_osgNodesPicked[0]->getNumParents() > 0) ? m_osgNodesPicked[0]->getParent(0) : m_osgNodesPicked[0];
                 toggleSelected(m_osgNodesPicked[0]);

                 std::cout << "+ picking uri : " << osgTools::getURI(m_osgNodesPicked[0]).getStringURI() << std::endl;
                 appGui().getTreeView()->selectItem(osgTools::getURI(m_osgNodesPicked[0]));
                 updateLabel(osgTools::getURI(m_osgNodesPicked[0]));
             }
             return true;
             break;
          /*case osgGA::GUIEventAdapter::KEY_Control_L:
          case osgGA::GUIEventAdapter::KEY_Control_R:
              std::cout << "start pick add" << std::endl;
              std::cout << "mask : " << ea.getModKeyMask() << std::endl;
              break;*/
          default:
             return false;
             break;
          }
       }
    /*case osgGA::GUIEventAdapter::KEYUP:
    {
        switch(ea.getKey())
        {
        case osgGA::GUIEventAdapter::KEY_Control_L:
        case osgGA::GUIEventAdapter::KEY_Control_R:
            std::cout << "stop pick add" << std::endl;
            break;
        default:
           return false;
           break;
        }
    }*/

        default:
            return false;
    }
    return false;
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::pickPoint(const osgGA::GUIEventAdapter &ea, osgViewer::View *viewer)
{
    osg::Node* scene = viewer->getSceneData();
    if(!scene) return;

    // use non dimensional coordinates - in projection/clip space
    osgUtil::LineSegmentIntersector* picker = new osgUtil::LineSegmentIntersector( osgUtil::Intersector::PROJECTION, ea.getXnormalized(),ea.getYnormalized() );
    osgUtil::IntersectionVisitor iv(picker);
    viewer->getCamera()->accept(iv);

    if(picker->containsIntersections())
    {
        osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
        //osg::notify(osg::NOTICE)<<"Picked "<<intersection.localIntersectionPoint<<std::endl;

        osg::NodePath& nodePath = intersection.nodePath;
        //node = (nodePath.size()>=1)?nodePath[nodePath.size()-1]:0;

        osg::Node* node = nodePath.back();

        osg::notify(osg::NOTICE) << "Picked: " << node->getName() << std::endl;


        //node = (nodePath.size()>=1)?nodePath[nodePath.size()-1]:0;
        //osg::Group* parent = (nodePath.size()>=2)?dynamic_cast<osg::Group*>(nodePath[nodePath.size()-2]):0;
        //osg::Group* parent = node->getParent(0);

        // get building
        if(m_pickingMode == 0) // face
        {
            // nothing to do, keep actual selection
        }
        if(m_pickingMode == 1) // building
        {
            vcity::URI uri = osgTools::getURI(node);
            citygml::CityObject* obj = appGui().getScene().getNode(uri);
            //citygml::CityObject* obj = m_scene->findNode(node->getName());
            while(obj && (obj->getTypeAsString() != "Building"))// || obj->getTypeAsString() != "TINRelief" ))
            {
                if(node->getNumParents() > 0)
                {
                    node = node->getParent(0);
                    std::cout << "parent node " << node->getName() << std::endl;
                }
                else
                    break;

                uri = osgTools::getURI(node);
                obj = appGui().getScene().getNode(uri);
                //obj = m_scene->findNode(node->getName());
            }

            if(!node) node = nodePath.back();
        }

        if(m_addToSelection)
        {
            addNodePicked(node);
            toggleSelected(node);
        }
        else
        {
            resetPicking();
            addNodePicked(node);
            toggleSelected(node);
        }

        node = node->getParent(0);
        std::cout << "picking uri : " << osgTools::getURI(node).getStringURI() << std::endl;
        appGui().getTreeView()->selectItem(osgTools::getURI(node));
        updateLabel(osgTools::getURI(node));

        //toggleSelected(node);
        //nodePicked = node;
    }
    else
    {
        resetPicking();
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::pickRectangle(const osgGA::GUIEventAdapter &ea, osgViewer::View *viewer)
{
    osg::Node* scene = viewer->getSceneData();
    if(!scene) return;

    // use non dimensional coordinates - in projection/clip space
    osgUtil::PolytopeIntersector* picker = new osgUtil::PolytopeIntersector( osgUtil::Intersector::PROJECTION, 0, 0, ea.getXnormalized(),ea.getYnormalized() );
    osgUtil::IntersectionVisitor iv(picker);
    viewer->getCamera()->accept(iv);

    if(picker->containsIntersections())
    {
        osgUtil::PolytopeIntersector::Intersections intersections = picker->getIntersections();
        //osg::notify(osg::NOTICE)<<"Picked "<<intersection.localIntersectionPoint<<std::endl;

        osgUtil::PolytopeIntersector::Intersections::const_iterator intersection = intersections.begin();
        for(; intersection != intersections.end(); ++intersection)
        {
            const osg::NodePath& nodePath = intersection->nodePath;
            //node = (nodePath.size()>=1)?nodePath[nodePath.size()-1]:0;

            std::vector<osg::Node*>::const_iterator it = nodePath.begin();
            for(; it != nodePath.end(); ++it)
            {
                osg::Node* node = *it;

                osg::notify(osg::NOTICE) << "Picked: " << node->getName() << std::endl;


                //node = (nodePath.size()>=1)?nodePath[nodePath.size()-1]:0;
                //osg::Group* parent = (nodePath.size()>=2)?dynamic_cast<osg::Group*>(nodePath[nodePath.size()-2]):0;
                //osg::Group* parent = node->getParent(0);

                // get building
                if(m_pickingMode == 0) // face
                {
                    // nothing to do, keep actual selection
                }
                if(m_pickingMode == 1) // building
                {
                    vcity::URI uri = osgTools::getURI(node);
                    citygml::CityObject* obj = appGui().getScene().getNode(uri);
                    //citygml::CityObject* obj = m_scene->findNode(node->getName());
                    while(obj && obj->getTypeAsString() != "Building")
                    {
                        if(node->getNumParents() > 0)
                        {
                            node = node->getParent(0);
                            //std::cout << "parent node " << node->getName() << std::endl;
                        }
                        else
                            break;

                        uri = osgTools::getURI(node);
                        obj = appGui().getScene().getNode(uri);
                        //obj = m_scene->findNode(node->getName());
                    }

                    if(!node) node = nodePath.back();
                }

                //if(m_addToSelection)
                {
                    addNodePicked(node);
                    toggleSelected(node);
                }
                /*else
                {
                    resetPicking();
                    addNodePicked(node);
                    toggleSelected(node);
                }*/

                updateLabel();

                //toggleSelected(node);
                //nodePicked = node;
            }
        }
    }
    else
    {
        resetPicking();
    }
}
////////////////////////////////////////////////////////////////////////////////
// toggle osgFX::Scribe to show picking
void PickHandler::toggleSelected(osg::Node *node, osg::Group *parent, bool /*forceUnselect*/)
{
    if(!node) return;

    if(!parent && node->getNumParents() > 0)
    {
        parent = node->getParent(0);
    }
    else
    {
        return;
    }

    std::cout << "  parent " << parent->className() << std::endl;

    osgFX::Scribe* parentAsSelected = dynamic_cast<osgFX::Scribe*>(parent);
    if (!parentAsSelected)
    {
        // node not already picked, so highlight it with an osgFX::Scribe
        osgFX::Scribe* selected = new osgFX::Scribe();
        selected->setName(node->getName());
        selected->addChild(node);
        parent->replaceChild(node, selected);


        //setLabel(node->getName());
        //setLabel(m_scene->findNode(node->getName()));
    }
    else
    {
        // node already picked so we want to remove scribe to unpick it.
        osg::Node::ParentList parentList = parentAsSelected->getParents();
        for(osg::Node::ParentList::iterator itr=parentList.begin();
            itr!=parentList.end();
            ++itr)
        {
            (*itr)->replaceChild(parentAsSelected, node);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
//const std::string& PickHandler::getNodePicked() const
//{
    //return nodePicked->getName();
//}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::setPickingMode(int mode)
{
    m_pickingMode = mode;
    if(m_pickingMode < 0) m_pickingMode = 0;    // force face
    if(m_pickingMode > 1) m_pickingMode = 1;    // force building
}
////////////////////////////////////////////////////////////////////////////////
