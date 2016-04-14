find .. -name CMakeLists.txt -exec grep FIND_PACKAGE {} \; | sed 's/[ ]*FIND_PACKAGE(//' | sed 's/ *//' | sed 's/)$//' where the <TAB> (third sed) is obtained with CTRL+V followed by t (macOS crap)

## Mandatory dependencies
 * QT: VCity seems to work with two versions 
   * [QT version 4.8](http://doc.qt.io/qt-4.8/) 
   * [QT version 5](http://www.qt.io/qt5-5/) with the [following packages](https://github.com/MEPP-team/VCity/blob/master/CMakeLists.txt#L119)
      * Qt5Core, Qt5Widgets, Qt5Xml, Qt5OpenGL, Qt53D
   QT which depends on
 * [X11 server](https://en.wikipedia.org/wiki/X_Window_System) as [QT sub-dependency](http://doc.qt.io/qt-4.8/requirements-x11.html)
 * [Open Scene Graph](http://www.openscenegraph.org/) (a.k.a. OSG)
 * [GDAL](http://www.gdal.org/) (Geospatial Data Abstraction Library)
 * [Proj4](https://github.com/OSGeo/proj.4/wiki) as gdal sub-dependency
 * [GEOS](https://trac.osgeo.org/geos/) as gdal sub-dependency
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
 * Install QtCreator : `apt-get install qtcreator`
 * Then all the depencies : `apt-get install qt4-default libopenscenegraph-dev libgdal-dev libassimp-dev libproj-dev libgeos++-dev`

 You now need to manually instal LASlib
 * If you are a poor intern at LIRIS without any right on your computer follow these steps (btw you might need to add `sudo` before previous installs) :
  1. `cd externals/`
  2. `sudo zip laslib.zip laslib`
  3. `sudo cp laslib.zip /root` (enter your password as required)
  4. `sudo su`
  5. `cd ~`
  6. `unzip laslib.zip`
  7. `cd laslib`
  8. `mkdir build`
  9. `cd build`
  10. `cmake .. -DCMAKE_BUILD_TYPE=Release; cmake .. -DCMAKE_BUILD_TYPE=Release; make`
  11. `sudo make install`

 * If you are on your own linux distribution skip steps from 2 to 6
