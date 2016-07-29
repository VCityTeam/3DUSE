#ifndef OSGSKYBOX_HPP
#define OSGSKYBOX_HPP

#include <osg/TextureCubeMap>
#include <osg/NodeCallback>
#include <osg/TexMat>
#include <osg/Transform>

struct TexMatCallback : public osg::NodeCallback
{
public:
    TexMatCallback(osg::TexMat& tm);

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

    osg::TexMat& _texMat;
};

osg::TextureCubeMap* readCubeMap();

class MoveEarthySkyWithEyePointTransform : public osg::Transform
{
public:
    /** Get the transformation matrix which moves from local coords to world coords.*/
    bool computeLocalToWorldMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const;

    /** Get the transformation matrix which moves from world coords to local coords.*/
    bool computeWorldToLocalMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const;

};

osg::Node* createSkybox();

#endif // OSGSKYBOX_HPP
