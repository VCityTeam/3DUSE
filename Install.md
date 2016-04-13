## Mandatory dependencies
 * QT: VCity seems to work with two versions 
   * [QT version 4.8](http://doc.qt.io/qt-4.8/) 
   * [QT version 5](http://www.qt.io/qt5-5/) with the [following packages](https://github.com/MEPP-team/VCity/blob/master/CMakeLists.txt#L119)
      * Qt5Core, Qt5Widgets, Qt5Xml, Qt5OpenGL, Qt53D
   QT which depends on
 * [X11 server](https://en.wikipedia.org/wiki/X_Window_System) (a [QT sub-dependency](http://doc.qt.io/qt-4.8/requirements-x11.html)
 * [Open Scene Graph](http://www.openscenegraph.org/) (a.k.a. OSG)
 * [GDAL](http://www.gdal.org/) (Geospatial Data Abstraction Library)
 * [Proj4](https://github.com/OSGeo/proj.4/wiki)
 * [GEOS](https://trac.osgeo.org/geos/)
 * [Assimp](http://www.assimp.org)
 * [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib): VCity [embeds its own copy](https://github.com/MEPP-team/VCity/tree/master/externals/laslib)

Open questions: some MTO installation docs (in emails) also mention
 * Python: really needed ? Sub-dependency ? Not needed at all ?
 * xerces-c: really needed ? Sub-dependency ? Not needed at all ?
 
## Mac OS X install
 * Obtain and install [Homebrew](http://brew.sh/)
 * `brew install Caskroom/cask/xquartz`(X11 server)
 * `brew install gdal`
 * `brew install open-scene-graph --with-qt gdal`
 * `brew install qt` (which installs qt4. Use `brew install qt5` is you opt for version 5)
 * `brew instal proj` (which is [proj4](https://github.com/OSGeo/proj.4/wiki))
 * `brew install geos` 
 * `brew install assimp`
 cd externals/laslib
mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release; cmake .. -DCMAKE_BUILD_TYPE=Release; make; sudo make install

## Ubuntu install
 libgdal-dev libproj-dev
libassimp-dev libgeos++-dev

