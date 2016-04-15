
## Mandatory dependencies
**Direct dependencies**
 * [QT version 4.8](http://doc.qt.io/qt-4.8/) (Concerning QT5 possible support refer to [issue #91](https://github.com/MEPP-team/VCity/issues/91))
 * [Open Scene Graph](http://www.openscenegraph.org/) (a.k.a. OSG)
 * [GDAL](http://www.gdal.org/) (Geospatial Data Abstraction Library)
 * [Assimp](http://www.assimp.org)
 * [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib): note that VCity [embarks its own copy](https://github.com/MEPP-team/VCity/tree/master/externals/laslib)
 * [Doxygen](http://www.stack.nl/~dimitri/doxygen/index.html): when building the documentation (optional).

Depending on your packaging system you **might need** to manually pull the following indirect dependencies:
 * [X11 server](https://en.wikipedia.org/wiki/X_Window_System) as [QT sub-dependency](http://doc.qt.io/qt-4.8/requirements-x11.html)
 * [Proj4](https://github.com/OSGeo/proj.4/wiki) as gdal sub-dependency
 * [GEOS](https://trac.osgeo.org/geos/) as gdal sub-dependency

Dependencies under discussion (open questions for MTO):
 * `Find_Packages` found in VCity CMakeLists:
   * OpenMesh
 * some MTO's installation docs (exchanged through emails) also mention
   * Python: really needed ? Sub-dependency ? Not needed at all ?
   * xerces-c: really needed ? Sub-dependency ? Not needed at all ?

## Ubuntu install
 * Classic package installation: 
    * `apt-get install qt4-default libopenscenegraph-dev libgdal-dev libassimp-dev libproj-dev libgeos++-dev`
 * Manual installation of LASlib
   * With access rigths to `/usr/local/`:
     * `cd VCity/externals/laslib`
     * `mkdir Build && cd Build`
     * `cmake .. -DCMAKE_BUILD_TYPE=Release && make`
     * `cmake .. -DCMAKE_BUILD_TYPE=Debug && make`
     * `sudo make install`
     * Proceed with building of Vcity
   * Without access rights to `/usr/local/`:
     * `cd VCity/externals/laslib`
     * `mkdir Build && cd Build`
     * `cmake .. -DCMAKE_INSTALL_PREFIX=~/local/LASlib` (or choose an alternate installation directory in your home directory or a stable direcory why proper access rights)
     * `cd VCity`
     * `mkdir Build && cd Build`
     * `ccmake .. -DLASLIB_INCLUDE_DIR=~/local/include -DLASLIB_LIBRARY=~/local/lib/liblaslib.a`

Post-install goodies (not VCity related):
 * QtCreator : `apt-get install qtcreator`

## Mac OS X install
 * Obtain and install [Homebrew](http://brew.sh/)
 * `brew install Caskroom/cask/xquartz`(X11 server)
 * `brew install gdal`
 * `brew install open-scene-graph --with-qt gdal`
 * `brew install qt` (which installs qt4)
 * `brew install assimp`
 * Install laslib:
   * The careful version with stow (`brew install stow`):
     * `cd VCity/externals/laslib`
     * `mkdir Build && cd Build`
     * `cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/stow/laslib`
     * `make install`
     * `cd /usr/local/stow``
     * `stow laslib``
   * The careless version:
     * `cd VCity/externals/laslib`
     * `mkdir Build && cd Build`
     * `cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/stow/laslib`
     * `make install`

Assert the following dependencies where intalled (e.g. with `brew info`) and otherwise install them manually:
 * `brew instal proj` (which is [proj4](https://github.com/OSGeo/proj.4/wiki))
 * `brew install geos`
