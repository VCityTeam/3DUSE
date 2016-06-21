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

void InfoDataType::computeDepthMap(int screenX, int screenY, std::map<float, osgInfo*> m_info)
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
                 osgInfo* c_info = it->second;
                 if(i>=floor(c_info->m_currentsCornerMin.x()) && i<=floor(c_info->m_currentsCornerMax.x()) && j>=floor(c_info->m_currentsCornerMin.y()) && j<floor(c_info->m_currentsCornerMax.y()))
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
         osgInfo* c_info = it->second;
         c_info->computeOVaMatrix(screen);
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

void InfoDataType::OVaDisplay(int screenX, int screenY, std::map<float, osgInfo *> m_info)
{
    float newZ;
    float newHeight;
    float farthestDCAM;
    for (std::map<float,osgInfo*>::iterator it1=m_info.begin(); it1!=m_info.end(); ++it1)
    {
        farthestDCAM=0;
        osgInfo* c_info = it1->second;
//        if(c_info->m_name=="opera")
//            std::cout<<"Opera OVa% = "<<c_info->m_initOVa/c_info->m_Da;

        if((c_info->m_initOVa/c_info->m_Da)>0.3)//if document hidden at its init position
        {
            if(c_info->m_name=="opera")
                std::cout<<"...in"<<std::endl;

            //move to free space
            for(size_t i = 0; i<c_info->m_OVaMatrix.size(); ++i)
            {
                for(size_t j=0; j<c_info->m_OVaMatrix[i].size(); ++j)
                {
                    //                        if(c_info->m_name=="opera")
                    //                        {
                    //                            std::cout<<"Opera DCAM = "<<c_info->m_DCAM<<std::endl;
                    //                            std::cout<<"Opera farthestDCAM "<<farthestDCAM<<std::endl;
                    //                        }
                    if(c_info->m_OVaMatrix[i][j]!=c_info->m_DCAM && c_info->m_OVaMatrix[i][j]>=farthestDCAM)
                    {
                        farthestDCAM=c_info->m_OVaMatrix[i][j];
                    }
                }
            }
            if(farthestDCAM!=0)
            {
                newZ = m_info.find(farthestDCAM)->second->m_currentposition.z();
                newHeight = m_info.find(farthestDCAM)->second->m_height;
                osg::Vec3 newPos = osg::Vec3(c_info->m_initposition.x(),c_info->m_initposition.y(),newZ+newHeight/2+c_info->m_height/2);
                c_info->UpdatePosition(newPos);
            }

            computeDepthMap(screenX, screenY, m_info);

        }
        else //if not
        {
            //move to init position
            c_info->getGroup()->getChild(0)->asTransform()->asPositionAttitudeTransform()->setPosition(c_info->m_initposition);
            c_info->UpdatePosition(c_info->m_initposition);

            computeDepthMap(screenX, screenY, m_info);
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
