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

void InfoDataType::stairedDisplay(std::map<float, osgInfo *> m_info)
{
    std::vector<osgInfo*> tmp_info;
    for (std::map<float,osgInfo*>::iterator it=m_info.begin(); it!=m_info.end(); ++it)
    {
        tmp_info.push_back(it->second);
    }
    for(size_t i=0; i<tmp_info.size(); ++i)
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
