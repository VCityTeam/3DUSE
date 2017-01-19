## Library dependencies of 3DUse

| Package         |    License    | Included headers / Notes |
| --------------- | ------------- | ------------------------ |
|[ASSIMP](http://assimp.sourceforge.net/main_doc.html) | [BSD](http://assimp.sourceforge.net/main_license.html)|[OSGPL](http://trac.openscenegraph.org/projects/osg//wiki/Legal) an LGPL variant|
|[Boost libraries](http://www.boost.org/) | [Boost license 1.0](http://www.boost.org/users/license.html) | License seems to fall within the permissive [MIT license grade/category](http://law.stackexchange.com/questions/91/is-there-any-difference-in-meaning-between-the-boost-and-mit-software-licenses). Packages:date_time, filesystem.|
|[GDAL](http://www.gdal.org/) |[X11/MIT style](https://trac.osgeo.org/gdal/wiki/FAQGeneral#WhatlicensedoesGDALOGRuse)| Depends on proj.4, .|
|[GEOS](https://trac.osgeo.org/geos/)|[LGPLv2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html)| For GDL. |
|[LASlib]() | [LGPLv2.1](https://github.com/LAStools/LAStools/blob/master/LICENSE.txt) | |
|[OpenSceneGraph (OSG)](http://www.openscenegraph.org/)|[OSGPL (LGPL like)](http://trac.openscenegraph.org/projects/osg/wiki/Legal)|Used sub-libraries: osgDB, osgGA, osgFX, osgQT, osgShadow, osgText,osgUtil osgViewer...|
|[Proj.4](http://proj4.org/)|[MIT](http://proj4.org/license.html)| For GDAL.|
|[Qt (4 or 5)](https://en.wikipedia.org/wiki/Qt_(software))| [LGPLV2.1, LGPLv3, but some modules are GPL](https://www.qt.io/licensing/). | QDir, QDirIterator, QFile, QString. |


## Library dependencies specific to the libCityGML component
Note: many of libCityGML library [sub-dependencies could be removed](https://github.com/MEPP-team/VCity/issues/69)

| Package         |    License    | Included headers / Notes |
| --------------- | ------------- | ------------------------ |
|[ASSIMP](http://assimp.sourceforge.net/main_doc.html) | [BSD](http://assimp.sourceforge.net/main_license.html)|[OSGPL](http://trac.openscenegraph.org/projects/osg//wiki/Legal) an LGPL variant|
|[Boost libraries](http://www.boost.org/) | [Boost license 1.0](http://www.boost.org/users/license.html) | License seems to fall within the permissive [MIT license grade/category](http://law.stackexchange.com/questions/91/is-there-any-difference-in-meaning-between-the-boost-and-mit-software-licenses) |
|[GLUT](https://en.wikipedia.org/wiki/OpenGL_Utility_Toolkit)|Implementation dependent: MIT for [FreeGLut](https://en.wikipedia.org/wiki/FreeGLUT), [SGI's GLUT](ftp://ftp.sgi.com/opengl/glut/index.html)|For GL/glu.h|
|[libXml2](http://www.xmlsoft.org/) |[MIT licence](http://www.xmlsoft.org/)|-|
|[OpenGL](https://www.opengl.org/)|[BSD/X/Mozilla like](https://www.sgi.com/tech/opengl/)|License depends on harware vendor.|
|[OpenSceneGraph (OSG)](http://www.openscenegraph.org/)|[OSGPL (LGPL like)](http://trac.openscenegraph.org/projects/osg/wiki/Legal)|Used sub-library: osgDB.|
|[Qt (4 or 5)](https://en.wikipedia.org/wiki/Qt_(software)| [LGPLV2.1, LGPLv3, but some modules are GPL!](https://www.qt.io/licensing/). | QDir, QDirIterator, QFile, QString. |

**Warning**: `grep -rih "#include" src/libcitygml | sort -rn  | uniq` only tells one part of the story because it includes headers from non sub-directories like utils, DataStrcutures...

## Optional dependencies
* PCL extensions:

| Package         |    License    | Included headers / Notes |
| --------------- | ------------- | ------------------------ |
|[PCL (Point Cloud Library)](https://en.wikipedia.org/wiki/Point_Cloud_Library)| [BSD](http://pointclouds.org/)| |
|[Boost libraries](http://www.boost.org/) | [Boost license 1.0](http://www.boost.org/users/license.html) | License seems to fall within the permissive [MIT license grade/category](http://law.stackexchange.com/questions/91/is-there-any-difference-in-meaning-between-the-boost-and-mit-software-licenses). Packages:system, filesystem, thread, date_time, iostreams, serialization, chrono .|
