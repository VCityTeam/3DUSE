// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgInfoDataType.hpp"
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

InfoDataType::InfoDataType()
{
}


InfoDataType::InfoDataType(osg::Node* n)
{
    switchRoot = n->asSwitch();
}

osg::ref_ptr<osg::Switch> InfoDataType::getSwitchRoot()
{
    return switchRoot;
}

void InfoDataType::computeDCAM(osg::Camera *cam, osgInfo* info)
{
    osg::Vec3d pos;
    osg::Vec3d target;
    osg::Vec3d up;
    cam->getViewMatrixAsLookAt(pos,target,up);
    float DCAM;

    float infoX = info->getPosition().x();
    float infoY = info->getPosition().y();
    float infoZ = info->getPosition().z();
    float camX=pos.x();
    float camY=pos.y();
    float camZ=pos.z();


    DCAM = sqrt((infoX-camX)*(infoX-camX)+(infoY-camY)*(infoY-camY)+(infoZ-camZ)*(infoZ-camZ));
    info->setDCAM(DCAM);
}

void InfoDataType::computeDSC(osg::Camera *cam, int screenX, int screenY, osgInfo* info)
{

    osg::Vec3d pos;
    osg::Vec3d target;
    osg::Vec3d up;
    cam->getViewMatrixAsLookAt(pos,target,up);

    osg::Matrix Mview = cam->getViewMatrix();
    osg::Matrix Mwin = cam->getViewport()->computeWindowMatrix();
    osg::Matrix Mproj = cam->getProjectionMatrix();

    osg::Vec3 worldcoord = info->getPosition();
    osg::Vec3 screencoord = worldcoord*Mview*Mproj*Mwin;

    int Ds = sqrt((screencoord.x()-screenX/2)*(screencoord.x()-screenX/2)+(screencoord.y()-screenY/2)*(screencoord.y()-screenY/2));
    info->setDSC(Ds);

    TVec3d dirLookAt(target.x() - pos.x(),target.y() - pos.y(),target.z() - pos.z());
    TVec3d dirDoc(info->m_currentposition.x() - pos.x(), info->m_currentposition.y() - pos.y(), info->m_currentposition.z() - pos.z());

    float scalaire = dirLookAt.dot(dirDoc);
    float angle = acos(scalaire/(dirLookAt.length()*dirDoc.length()));
    angle=angle*180/3.1415;

    osg::Vec3f normale(-dirDoc.x, -dirDoc.y, -dirDoc.z);
    osg::Vec3f axis = info->m_billboard->getAxis();
    osg::Vec3f ortho = axis.operator ^(normale);
    osg::Vec3f wCornerMax = info->m_currentposition + (ortho/ortho.length())*(info->m_width/2.0f) + (axis/axis.length())*(info->m_height/2.0f);
    osg::Vec3f wCornerMin = info->m_currentposition - (ortho/ortho.length())*(info->m_width/2.0f) - (axis/axis.length())*(info->m_height/2.0f);

    osg::Vec3 sCornerMax = wCornerMax*Mview*Mproj*Mwin;
    osg::Vec3 sCornerMin = wCornerMin*Mview*Mproj*Mwin;

    /********** RED POINTS TO WITNESS CORNERS ************/
    //                osg::Geode* cornersGeode = new osg::Geode;
    //                osg::Geometry* cornersGeom = new osg::Geometry;
    //                osg::Vec3Array* verticesPoints = new osg::Vec3Array;

    //                verticesPoints->push_back(wCornerMax);
    //                verticesPoints->push_back(wCornerMin);

    //                osg::ref_ptr<osg::Vec4Array> colorpoints = new osg::Vec4Array;
    //                colorpoints->push_back(osg::Vec4(1.0,0.0,0.0,1.0));

    //                cornersGeom->setVertexArray(verticesPoints);
    //                cornersGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,verticesPoints->size()));
    //                cornersGeom->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f), osg::StateAttribute::ON);

    //                cornersGeom->setColorArray(colorpoints, osg::Array::BIND_OVERALL);
    //                cornersGeode->addDrawable(cornersGeom);

    //                if(info->getGroup()->getNumChildren()>3)
    //                {
    //                    info->getGroup()->removeChild(info->getGroup()->getNumChildren()-1);
    //                    info->getGroup()->addChild(cornersGeode);
    //                }
    //                else
    //                    info->getGroup()->addChild(cornersGeode);

    /* Determine if document on screen or not according to the borders of the screen */
    if(sCornerMax.x()>0.0 && sCornerMin.x()<screenX && sCornerMax.y()>0.0 && sCornerMin.y()<screenY && angle < 90.0)
        info->setonScreen(true);
    else if(sCornerMax.x()<0.0 && sCornerMin.x()>screenX && sCornerMax.y()<0.0 && sCornerMin.y()>screenY && angle < 90.0)
        info->setonScreen(true);
    else
        info->setonScreen(false);


    /* Crop document coordinates to tell the exact surface inside the screen */
    if(sCornerMax.x()>screenX)
        sCornerMax.x()=screenX;
    if(sCornerMax.x()<0.0)
        sCornerMax.x()=0.0;
    if(sCornerMax.y()>screenY)
        sCornerMax.y()=screenY;
    if(sCornerMax.y()<0.0)
        sCornerMax.y()=0.0;
    if(sCornerMin.x()<0.0)
        sCornerMin.x()=0.0;
    if(sCornerMin.x()>screenX)
        sCornerMin.x()=screenX;
    if(sCornerMin.y()<0.0)
        sCornerMin.y()=0.0;
    if(sCornerMin.y()>screenY)
        sCornerMin.y()=screenY;

    float screenwidth = sCornerMax.x()-sCornerMin.x();
    float screenheight = sCornerMax.y()-sCornerMin.y();
    float Da = abs(screenwidth*screenheight);

    info->m_sCornerMax=sCornerMax;
    info->m_sCornerMin=sCornerMin;

    info->setDa(Da);


    //info->UpdateTetra(normale/normale.length(), axis/axis.length(), ortho/ortho.length());


}

void InfoDataType::computeOVa(int screenX, int screenY, std::map<float, osgInfo*> m_info)
{
   std::vector< std::vector<float> > screen;
   bool found;

    for(int i = 0; i<screenX; i++)
    {
        std::vector<float> col;
        for(int j=0; j<screenY; j++)
        {
            found=false;
            for (std::map<float,osgInfo*>::iterator it=m_info.begin(); it!=m_info.end(); ++it)
            {
                if(i>=(int)it->second->m_sCornerMin.x() && i<=(int)it->second->m_sCornerMax.x() && j>=(int)it->second->m_sCornerMin.y() && j<(int)it->second->m_sCornerMax.y())
                {
                    col.push_back(it->first);
                    found = true;
                    break;
                }
            }
            if(!found)
                col.push_back(0.0f);

        }
        screen.push_back(col);
    }


    for (std::map<float,osgInfo*>::iterator it=m_info.begin(); it!=m_info.end(); ++it)
    {
        it->second->m_OVa=0;
        for(int i = (int)it->second->m_sCornerMin.x(); i<(int)it->second->m_sCornerMax.x(); i++)
        {
            for(int j=(int)it->second->m_sCornerMin.y(); j<(int)it->second->m_sCornerMax.y(); j++)
            {
                if(screen[i][j]!=it->first && screen[i][j]!=0)
                {
                    it->second->m_OVa+=1;
                }
            }
        }
        if(it->second->m_OVa>it->second->m_Da)
            it->second->m_OVa=it->second->m_Da;

    }

    screen.clear();
}

void InfoDataType::setDisplayability(osgInfo* info)
{
    if(info->getInfoLOD()=="street")
    {
        if(info->m_DCAM<=500)
        {
            info->setDisplayable(true);
            info->setTransparency(1.0f);
        }
        else if (info->m_DCAM>500 && info->m_DCAM<1500)
        {
            info->setDisplayable(true);
            info->setTransparency(0.2f);
        }
        else
        {
            info->setDisplayable(false);
        }
    }

    if(info->getInfoLOD()=="building")
    {
        if(info->m_DCAM<=1500 && info->m_DCAM>=500)
        {
            info->setDisplayable(true);
            info->setTransparency(1.0f);
        }
        else if(info->m_DCAM>1500 &&  info->m_DCAM<6000)
        {
            info->setDisplayable(true);
            info->setTransparency(0.2f);
        }
        else
        {
            info->setDisplayable(false);
            info->setTransparency(0.2f);
        }
    }

    if(info->getInfoLOD()=="district")
    {
        if(info->m_DCAM<=6000 && info->m_DCAM>=1500)
        {
            info->setDisplayable(true);
            info->setTransparency(1.0f);
        }
        else if(info->m_DCAM>6000 && info->m_DCAM<30000)
        {
            info->setDisplayable(true);
            info->setTransparency(0.2f);
        }
        else
        {
            info->setDisplayable(false);
            info->setTransparency(0.2f);
        }

    }
    if(info->getInfoLOD()=="city")
    {
        if(info->m_DCAM<=30000 && info->m_DCAM>=6000)
        {
            info->setDisplayable(true);
            info->setTransparency(1.0f);
        }
        else if(info->m_DCAM>30000)
        {
            info->setDisplayable(true);
            info->setTransparency(0.2f);
        }
        else
        {
            info->setDisplayable(false);
            info->setTransparency(0.2f);
        }

    }

}

void InfoDataType::updateScale(osgInfo* info, int screenX, int screenY)
{
    //Function of DCAM
    //float scale = info->m_DCAM/50450.1;

    //Function of DSC
    float scale;
    float maxscale = std::sqrt(screenX*screenX+screenY*screenY);
    if(info->m_DSC)
        scale = info->m_DSC/maxscale;
    info->Scaling(scale);

}

void InfoDataType::stairedDisplay(std::map<float, osgInfo *> m_info)
{
    std::vector<osgInfo*> tmp_info;
    for (std::map<float,osgInfo*>::iterator it=m_info.begin(); it!=m_info.end(); ++it)
    {
        tmp_info.push_back(it->second);
    }
    for(int i=0; i<tmp_info.size(); i++)
    {
       osgInfo* info = tmp_info[0];
       info->getGroup()->getChild(0)->asTransform()->asPositionAttitudeTransform()->setPosition(info->m_initposition);
       info->UpdatePosition(info->m_initposition);
       if(i>0)
       {
           osgInfo* c_info = tmp_info[i];
           osgInfo* p_info = tmp_info[i-1];
           osg::Vec3 newPos = osg::Vec3(c_info->m_currentposition.x(), c_info->m_currentposition.y(),p_info->m_currentposition.z()+p_info->m_height/2+c_info->m_height/2);
           c_info->getGroup()->getChild(0)->asTransform()->asPositionAttitudeTransform()->setPosition(newPos);
           c_info->UpdatePosition(newPos);
           c_info->UpdateAnchoringLine(newPos.z());
       }
    }
}

void InfoDataType::display()
{
    for(unsigned int i=0; i<switchRoot->getNumChildren(); i++)
    {
        osg::ref_ptr<osg::Switch> subSwitch = switchRoot->getChild(i)->asSwitch();
        if(subSwitch)
        {
            for(unsigned int j=0; j<subSwitch->getNumChildren();j++)
            {
                osg::Node* node = subSwitch->getChild(j);
                osgInfo* info = dynamic_cast<osgInfo*>(node);
                if(info->isDisplayable() && info->isRequested())
                {
                    subSwitch->setValue(j,true);
                }
                else
                {
                    subSwitch->setValue(j,false);
                }
            }
        }

    }
}
