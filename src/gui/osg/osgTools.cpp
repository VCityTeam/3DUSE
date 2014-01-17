////////////////////////////////////////////////////////////////////////////////
#include "osgTools.hpp"
#include <osg/Geometry>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> osgTools::buildBBox(osg::Vec3 lowerBound, osg::Vec3 upperBound)
{
    osg::Vec3 step = upperBound - lowerBound;

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    for(int x=0; x<=1; ++x)
    {
        for(int y=0; y<=1; ++y)
        {
            for(int z=0; z<=1; ++z)
            {
                vertices->push_back(osg::Vec3(lowerBound.x()+ x*step.x(), lowerBound.y() + y*step.y(), lowerBound.z() + z*step.z()));
                //std::cout << lowerBound.x()+ x*step.x() << " " << lowerBound.y() + y*step.y() << " " << lowerBound.z() + z*step.z() << std::endl;
            }
        }
    }

    indices->push_back(0); indices->push_back(1);
    indices->push_back(0); indices->push_back(2);
    indices->push_back(0); indices->push_back(4);
    indices->push_back(3); indices->push_back(1);
    indices->push_back(3); indices->push_back(2);
    indices->push_back(3); indices->push_back(7);
    indices->push_back(5); indices->push_back(1);
    indices->push_back(5); indices->push_back(4);
    indices->push_back(5); indices->push_back(7);
    indices->push_back(6); indices->push_back(2);
    indices->push_back(6); indices->push_back(4);
    indices->push_back(6); indices->push_back(7);

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geode->addDrawable(geom);

    return geode;
}
////////////////////////////////////////////////////////////////////////////////
std::string osgTools::getURI(osg::Node* node)
{
    osg::Node* parent = node;
    std::string URI = node->getName();
    while((parent = (osg::Node*)(parent->getParent(0))) != NULL)
    {
        if(parent->getName() == "layers")
            break;
        URI.insert(0, parent->getName());
    }

    return URI;
}
////////////////////////////////////////////////////////////////////////////////
