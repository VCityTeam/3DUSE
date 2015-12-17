// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgPicking.hpp"
#include "osgTools.hpp"
#include <osgFX/Scribe>
#include <osg/PolygonMode>
#include <iostream>
#include <algorithm>
#include "gui/moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
PickHandler::PickHandler()
    : m_mx(0.0),m_my(0.0), m_pickingMode(1), m_addToSelection(false)
{
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::resetPicking()
{
    for(const vcity::URI& uri : appGui().getSelectedNodes())
    {
        deselectNode(uri);
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::updateLabel(const vcity::URI& uri)
{
    appGui().getMainWindow()->updateTextBox(uri);
}
////////////////////////////////////////////////////////////////////////////////
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
                //std::cout << "start pick add" << std::endl;
                m_addToSelection = true;
            }
            else
            {
                //std::cout << "stop pick add" << std::endl;
                m_addToSelection = false;
            }

            if(m_mx == ea.getX() && m_my == ea.getY())
            {
                // only do a pick if the mouse hasn't moved
                // TODO: add a little headroom so the mouse does not have to stand perfectly still.
                //std::cout << "Pick point : (" << m_mx << ", " << m_my << ")" << std::endl;
                pickPoint(ea,viewer);
            }
            else
            {
                //std::cout << "Pick rectangle : (" << m_mx << ", " << m_my << "), (" << ea.getX() << ", " << ea.getY() << ")"  << std::endl;
                //std::cout << "Pick rectangle : (" << m_mx << ", " << m_my << "), (" << ea.getXnormalized() << ", " << ea.getYnormalized() << ")"  << std::endl;
                //pickRectangle(ea,viewer);
            }
            return true;
        }
        case(osgGA::GUIEventAdapter::KEYDOWN):
       {
          switch(ea.getKey())
          {
          case osgGA::GUIEventAdapter::KEY_Escape:
              return true;
              break;
          case osgGA::GUIEventAdapter::KEY_Plus:
          case osgGA::GUIEventAdapter::KEY_KP_Add:
          case osgGA::GUIEventAdapter::KEY_P:

            if(appGui().getSelectedNodes().size() > 0)
            {
                vcity::URI uri = appGui().getSelectedNodes()[0];
                appGui().getControllerGui().resetSelection();

                uri.popBack();
                appGui().getControllerGui().addSelection(uri);
                //std::cout << "pick parent" << std::endl;
                //std::cout << "pick parent : " << uri.getStringURI() << std::endl;

                updateLabel(uri);
            }

             /*if(m_osgNodesPicked.size() > 0)
             {
                 toggleSelected(m_osgNodesPicked[0]); // unselect last node
                 m_osgNodesPicked[0] = (m_osgNodesPicked[0]->getNumParents() > 0) ? m_osgNodesPicked[0]->getParent(0) : m_osgNodesPicked[0];
                 toggleSelected(m_osgNodesPicked[0]);

                 std::cout << "+ picking uri : " << osgTools::getURI(m_osgNodesPicked[0]).getStringURI() << std::endl;
                 appGui().getTreeView()->selectItem(osgTools::getURI(m_osgNodesPicked[0]));
                 updateLabel(osgTools::getURI(m_osgNodesPicked[0]));
             }*/
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

        //osg::notify(osg::NOTICE) << "Picked: " << node->getName() << std::endl;


        //node = (nodePath.size()>=1)?nodePath[nodePath.size()-1]:0;
        //osg::Group* parent = (nodePath.size()>=2)?dynamic_cast<osg::Group*>(nodePath[nodePath.size()-2]):0;
        //osg::Group* parent = node->getParent(0);

        // check that we are not on a geode
        if(node->asGeode())
        {
            node = node->getParent(0);
        }

        // get building
        if(m_pickingMode == 0) // face
        {
            // nothing to do, keep actual selection
        }
        if(m_pickingMode == 1) // building
        {
            // backup node
            osg::Node* nodeOri = node;

            vcity::URI uri = osgTools::getURI(node);
            citygml::CityObject* obj = appGui().getScene().getCityObjectNode(uri, true);
            //citygml::CityObject* obj = m_scene->findNode(node->getName());
            while(obj && (obj->getTypeAsString() != "Building"))// || obj->getTypeAsString() != "TINRelief" ))
            {
                if(node->getNumParents() > 0)
                {
                    node = node->getParent(0);
                    //std::cout << "parent node " << node->getName() << std::endl;
                }
                else
                    break;

                uri = osgTools::getURI(node);
                obj = appGui().getScene().getCityObjectNode(uri, true);
                //obj = m_scene->findNode(node->getName());
            }

            if(!node) node = nodePath.back();

            // check we found a building, else keep first found node
            uri = osgTools::getURI(node);
            obj = appGui().getScene().getCityObjectNode(uri, true);
            if(!obj || obj->getTypeAsString() != "Building")
            {
                node = nodeOri;
            }
        }

        if(m_addToSelection)
        {
            appGui().getControllerGui().addSelection(osgTools::getURI(node));
        }
        else
        {
            appGui().getControllerGui().resetSelection();
            appGui().getControllerGui().addSelection(osgTools::getURI(node));
        }
        updateLabel(osgTools::getURI(node));

        //node = node->getParent(0);
        //std::cout << "picking uri : " << osgTools::getURI(node).getStringURI() << std::endl;
        //appGui().getTreeView()->selectItem(osgTools::getURI(node));
        //updateLabel(osgTools::getURI(node));

        //toggleSelected(node);
        //nodePicked = node;
    }
    else
    {
        appGui().getControllerGui().resetSelection();
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::pickRectangle(const osgGA::GUIEventAdapter &ea, osgViewer::View *viewer)
{
    /*osg::Node* scene = viewer->getSceneData();
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
                //else
                //{
                //    resetPicking();
                //    addNodePicked(node);
                //    toggleSelected(node);
                //}

                updateLabel();

                //toggleSelected(node);
                //nodePicked = node;
            }
        }
    }
    else
    {
        resetPicking();
    }*/
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

    //std::cout << "  parent " << parent->className() << std::endl;
    //std::cout << "osg toggleSelected : " << node->getName() << std::endl;

    osgFX::Scribe* parentAsSelected = dynamic_cast<osgFX::Scribe*>(parent);
    if (!parentAsSelected)
    {
        // node not already picked, so highlight it with an osgFX::Scribe
        /*osgFX::Scribe* selected = new osgFX::Scribe();
        selected->setName(node->getName());
        selected->addChild(node);
        parent->replaceChild(node, selected);*/

        node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

        //setLabel(node->getName());
        //setLabel(m_scene->findNode(node->getName()));
    }
    else
    {
        // node already picked so we want to remove scribe to unpick it.
        /*osg::Node::ParentList parentList = parentAsSelected->getParents();
        for(osg::Node::ParentList::iterator itr=parentList.begin();
            itr!=parentList.end();
            ++itr)
        {
            (*itr)->replaceChild(parentAsSelected, node);
        }*/
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::toggleSelected(const vcity::URI& uri)
{
    //appGui().addSelectedNode(uri);
    std::cout << "toggleSelected : " << uri.getStringURI() << std::endl;
    toggleSelected(appGui().getOsgScene()->getNode(uri));
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::selectNode(const vcity::URI& uri)
{
    uri.resetCursor();
    osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);
    if(node)
    {
        //osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
        //pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
        //node->getOrCreateStateSet()->setAttribute(pm.get());

        osg::ref_ptr<osg::Material> mat = (osg::Material*)node->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
        if(!mat.valid())
        {
            mat = new osg::Material;
        }
        //mat->setColorMode(osg::Material::EMISSION);
        mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(1,0,0,1));
        //mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1,0,0,1));
        mat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(1,0,0,1));
        //mat->setShininess(osg::Material::FRONT_AND_BACK, 128);

        node->getStateSet()->setAttribute(mat);
        //node->getStateSet()->setAttributeAndModes( mat, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
        //node->getStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
        appGui().getOsgScene()->createInfoBubble(node);
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::deselectNode(const vcity::URI& uri)
{
    uri.resetCursor();
    osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);
    if(node)
    {
        // remove material (red highlight)
        node->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::MATERIAL);

        // remove infobubble
        osg::ref_ptr<osg::Group> grp = node->asGroup();
        if(grp)
        {
            int nb = grp->getNumChildren();
            for(int i=0; i<nb; ++i)
            {
                if(grp->getChild(i)->getName() == "infobubble")
                {
                    grp->removeChild(i);
                }
            }
        }

        /*vcity::URI uriInfo = uri;
        uriInfo.append("infobubble");
        uriInfo.resetCursor();
        appGui().getOsgScene()->deleteNode(uriInfo);
        */
    }
}
////////////////////////////////////////////////////////////////////////////////
void PickHandler::setPickingMode(int mode)
{
    m_pickingMode = mode;
    if(m_pickingMode < 0) m_pickingMode = 0;    // force face
    if(m_pickingMode > 1) m_pickingMode = 1;    // force building
}
////////////////////////////////////////////////////////////////////////////////
