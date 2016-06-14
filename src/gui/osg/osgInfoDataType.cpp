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
        it->second->m_OVaMatrix.clear();
        it->second->m_OVa=0;
        std::vector<float> col;
        for(int i = (int)it->second->m_sCornerMin.x(); i<(int)it->second->m_sCornerMax.x(); ++i)
        {
            col.clear();
            for(int j=(int)it->second->m_sCornerMin.y(); j<(int)it->second->m_sCornerMax.y(); ++j)
            {
//                if(it->second->m_name == "opera")
//                    std::cout<<"Matrice de profondeur ["<<i<<"]["<<j<<"] = "<<screen[i][j]<<std::endl;
                col.push_back(screen[i][j]);
                if(screen[i][j]!=it->first)
                {
                    it->second->m_OVa+=1;
                }
            }
            it->second->m_OVaMatrix.push_back(col);
        }
        if(it->second->m_OVa>it->second->m_Da)
            it->second->m_OVa=it->second->m_Da;

    }

//    for (std::map<float,osgInfo*>::iterator it=m_info.begin(); it!=m_info.end(); ++it)
//    {
//        if(it->second->m_name=="opera")
//        {
//            std::cout<<"Opera profondeur matrice : "<<std::endl;
//            for(int i = 0; i<it->second->m_OVaMatrix.size(); ++i)
//            {
//                for(int j = 0; j<it->second->m_OVaMatrix[i].size(); ++j)
//                {
//                    std::cout<<it->second->m_OVaMatrix[i][j]<<" ";
//                }
//                std::cout<<std::endl;
//            }
//        }
//    }


    screen.clear();
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

void InfoDataType::OVaDisplay(std::map<float, osgInfo *> m_info)
{
    std::vector<float> OVlist;
    for (std::map<float,osgInfo*>::iterator it=m_info.begin(); it!=m_info.end(); ++it)
    {
        osgInfo* c_info = it->second;
        OVlist.clear();
        for(int i = 0; i<c_info->m_OVaMatrix.size(); ++i)
        {
            for(int j=0; j<c_info->m_OVaMatrix[i].size(); ++j)
            {
                if(c_info->m_name!=m_info.find(c_info->m_OVaMatrix[i][j])->second->m_name)
                {
                    OVlist.push_back(c_info->m_OVaMatrix[i][j]);
                    if(c_info->m_name=="opera")
                        std::cout<<c_info->m_OVaMatrix[i][j]<<" DCAM -> "<<m_info.find(c_info->m_OVaMatrix[i][j])->second->m_name<<std::endl;
                }
            }

            if(c_info->m_OVa/c_info->m_Da>=0.3)
            {
                float newZ = m_info.find(OVlist[0])->second->m_currentposition.z();
                float newHeight = m_info.find(OVlist[0])->second->m_height;
                osg::Vec3 newPos = osg::Vec3(c_info->m_currentposition.x(),c_info->m_currentposition.y(),newZ+newHeight/2+c_info->m_height/2);
                if(c_info->m_name=="opera")
                {
//                    std::cout<<"Opera is overlapped by more than 0.3 by "<<OVlist[0]<<" DCAM"<<std::endl;
//                    std::cout<<"    current Z : "<<c_info->m_currentposition.z()<<std::endl;
//                    std::cout<<"    new Z : "<<newZ+newHeight/2+c_info->m_height/2<<std::endl;
                }
                c_info->getGroup()->getChild(0)->asTransform()->asPositionAttitudeTransform()->setPosition(newPos);
                c_info->UpdatePosition(newPos);
            }
            else
            {
                c_info->getGroup()->getChild(0)->asTransform()->asPositionAttitudeTransform()->setPosition(c_info->m_initposition);
                c_info->UpdatePosition(c_info->m_initposition);
            }
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
