// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGAssimp_HPP__
#define __OSGAssimp_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <osgDB/ReaderWriter>
////////////////////////////////////////////////////////////////////////////////
typedef std::map<std::string, osg::ref_ptr<osg::Texture> > TextureMap;

typedef osgDB::ReaderWriter::ReadResult ReadResult;
ReadResult readNode(const std::string& file, const osgDB::ReaderWriter::Options* options);
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGAssimp_HPP__
