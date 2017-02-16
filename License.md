## Library dependencies of 3DUse

| Package         |    License    | Included headers / Notes |
| --------------- | ------------- | ------------------------ |
|[ASSIMP](http://assimp.sourceforge.net/main_doc.html) | [BSD](http://assimp.sourceforge.net/main_license.html)|[OSGPL](http://trac.openscenegraph.org/projects/osg//wiki/Legal) an LGPL variant|
|[Boost libraries](http://www.boost.org/) | [Boost license 1.0](http://www.boost.org/users/license.html) | License seems to fall within the permissive [MIT license grade/category](http://law.stackexchange.com/questions/91/is-there-any-difference-in-meaning-between-the-boost-and-mit-software-licenses). Packages:date_time, filesystem.|
|[GDAL](http://www.gdal.org/) |[X11/MIT style](https://trac.osgeo.org/gdal/wiki/FAQGeneral#WhatlicensedoesGDALOGRuse)| Depends on proj.4.|
|[GEOS](https://trac.osgeo.org/geos/)|[LGPLv2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html)| For GDL. |
|[LASlib]() | [LGPLv2.1](https://github.com/LAStools/LAStools/blob/master/LICENSE.txt) | |
|[OpenSceneGraph (OSG)](http://www.openscenegraph.org/)|[OSGPL (LGPL like)](http://trac.openscenegraph.org/projects/osg/wiki/Legal)|Used sub-libraries: osgDB, osgGA, osgFX, osgQT, osgShadow, osgText,osgUtil osgViewer...|
|[Proj.4](http://proj4.org/)|[MIT](http://proj4.org/license.html)| For GDAL.|
|[Qt (4 or 5)](https://en.wikipedia.org/wiki/Qt_(software))| [LGPLV2.1, LGPLv3, but some modules are GPL](https://www.qt.io/licensing/). | [Core: LGPLV2 or V3](http://doc.qt.io/qt-5/qtcore-index.html#licenses-and-attributions), [Gui: LGPLV2 or V3](http://doc.qt.io/qt-5/qtgui-index.html#licenses-and-attributions), QTWidgets: license ?? (References: [QTWidgets module](http://doc.qt.io/qt-5/qtwidgets-index.html), [licenses in QT](http://doc.qt.io/archives/qt-5.5/licensing.html#licenses-used-in-qt)) |

Note concerning the QT modules:
 * [Licenses used in QT](http://doc.qt.io/archives/qt-5.5/licensing.html#licenses-used-in-qt) gateway.
 * List of used packages within modules:
    * **Core module**: [QDebug](https://doc.qt.io/archives/qt-5.5/qdebug.html), [QDir](http://doc.qt.io/qt-5/qdir.html), [QDirIterator](http://doc.qt.io/qt-5/qdiriterator.html), [QEvent](http://doc.qt.io/qt-5/qevent.html), [QFile](http://doc.qt.io/qt-5/qfile.html) and [QFileInfo](http://doc.qt.io/qt-5/qfileinfo.html), [QMutex](http://doc.qt.io/qt-5/qmutex.html), [QPluginLoader](http://doc.qt.io/qt-5/qpluginloader.html), [QPointer](http://doc.qt.io/qt-5/qpointer.html), [QQueue](http://doc.qt.io/qt-5/qqueue.html), [QSet](http://doc.qt.io/qt-5/qset.html), [QSettings](http://doc.qt.io/qt-5/qsettings.html), [QString](http://doc.qt.io/qt-5/QString.html), [QStringList](http://doc.qt.io/qt-5/qstringlist.html), [Qtime](http://doc.qt.io/qt-5/qtime.html) and [QTimer](http://doc.qt.io/qt-5/qtimer.html), [QTextStream](http://doc.qt.io/qt-5/qtextstream.html), 
    * **Gui module (includes OpenGL)**: [QFont](http://doc.qt.io/qt-5/qfont.html), [QGLWidget](http://doc.qt.io/qt-5/qglwidget.html), [QIcon](http://doc.qt.io/qt-5/QIcon.html), [QImage](http://doc.qt.io/qt-5/qimage.html) and [QImageReader](http://doc.qt.io/qt-5/qimagereader.html), [QInputEvent](http://doc.qt.io/qt-5/qinputevent.html), [QResizeEvent](http://doc.qt.io/archives/qt-5.5/qresizeevent.html), 
    * **Widgets module**: [QDialog](http://doc.qt.io/qt-5/QDialog.html), [QFileDialog](http://doc.qt.io/qt-5/qfiledialog.html), [QGridLayout](http://doc.qt.io/qt-5/qgridlayout.html), [QGraphicScene](http://doc.qt.io/qt-5/qgraphicsscene.html) and [QGraphicView](http://doc.qt.io/qt-5/qgraphicsview.html), [QHeaderView](http://doc.qt.io/qt-5/qheaderview.html), [QLabel](http://doc.qt.io/qt-5/qlabel.html), [QLineEdit](http://doc.qt.io/qt-5/qlineedit.html), [QListView](http://doc.qt.io/qt-5/qlistview.html) and [QListWidget](http://doc.qt.io/qt-5/qlistwidget.html), [QMainWindow](http://doc.qt.io/qt-5/qmainwindow.html), [QMenu](http://doc.qt.io/qt-5/qmenu.html), [QMessageBox](http://doc.qt.io/qt-5/qmessagebox.html), [QPainter](http://doc.qt.io/qt-5/qpainter.html), [QProgressDialog](http://doc.qt.io/qt-5/qprogressdialog.html), [QPushButton](http://doc.qt.io/qt-5/QPushButton.html), [QTextBrowser](http://doc.qt.io/qt-5/qtextbrowser.html), [QTreeWidget](http://doc.qt.io/archives/qt-5.5/qtreewidgetitem.html),
    * **unknown module**: [QtGlobal](http://doc.qt.io/qt-5/qtglobal.html), [QtPlugin](http://doc.qt.io/qt-5/qtplugin.html), 
    * used by osgQt: [QtWebKit](https://wiki.qt.io/Qt_WebKit), QtWebKitWidgets 
 * Obtaining the above list of used QT modules is merely a comment of the `grep -rh include src ui | grep -i Q | grep -v CityGML | grep -v QtEvents | grep -v osgQt | sort | uniq` command (QtEvents being used by `osgQt/QGraphicsViewAdapter` and `osgQt` belonging to [OSG](https://github.com/openscenegraph/osgQt) ).

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
