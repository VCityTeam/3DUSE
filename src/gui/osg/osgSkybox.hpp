// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
