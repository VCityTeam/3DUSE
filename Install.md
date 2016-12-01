# Installing VCity 

## List of VCity dependencies
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

## General introduction to building VCity
VCity is build (compiled, linked, installed) [`cmake`](https://cmake.org/runningcmake/). Here is a short list of option flags that can be used to customize the building VCity:
 * `BUILD_GUI_QT4` and `BUILD_GUI_QT5` enable to build the Graphical User Interface (GUI) of VCity respectively with using [QT4](http://doc.qt.io/qt-4.8/) or [QT5](http://qt-project.org/qt5)
 * Flags of the form `BUILD_CityGML<something>QtPlugin` (e.g. `BUILD_CityGMLCutQtPlugin` or `BUILD_CityGMLSunlightQtPlugin`) toggle the build of optional VCity plugins (based on QT plugin mechanism). Unsurprisingly enough the `BUILD_ALL_PLUGINS` option flag will trigger the build of all plugins.
 * `BUILD_DOCUMENTATION` enables the building of the [Doxygen](http://www.doxygen.org/) based documentation of VCity.
 * `BUILD_EMBARKED_OSG-QT_32` and `BUILD_EMBARKED_OSG-QT_34` enable to build a VCity embarked version of the interface of [OpenSceneGraph (OSG)](http://www.openscenegraph.org/) within [QT](http://qt-project.org/)


## Ubuntu install (Ubuntu 14.04)
### Installing dependencies
 * Classic package installation with `apt-get` command:
    * `apt-get install qt4-default libopenscenegraph-dev libgdal-dev libassimp-dev`
    * `apt-get install libboost-all-dev`
    * When building with the [PCL](http://pointclouds.org/) extension (`BUILD_PCL` set to ON within cmake):
      * Install sub-dependencies: `apt-get install libeigen3-dev libflann-dev libqhull-dev`
      * Easy (recommended for [newbie](https://en.wikipedia.org/wiki/Newbie)) Point Cloud library installation out of VCity provided [tarball](https://en.wikipedia.org/wiki/Tar_(computing) (pre-configured sources):
        * `cd /tmp && wget https://download.gforge.liris.cnrs.fr/meppbin/travis-trusty/pcl-pcl-1.7.2.travis-trusty.tgz`
        * `tar zxf pcl-pcl-1.7.2.travis-trusty.tgz && cd pcl-pcl-1.7.2/buildR`
        * `sudo make install`
      * Alternatively (more advanced installer) proceed with PCL standard installation (out of git repository): refer e.g. to [LarryLisky's install notes](https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/).
 * Manual installation of LASlib
   * With access rigths to `/usr/local/`:
     * `cd VCity/externals/laslib`
     * `mkdir Build && cd Build`
     * `cmake .. -DCMAKE_BUILD_TYPE=Release && make`
     * `sudo make install`
     * Proceed with building of Vcity
   * Without access rights to `/usr/local/`:
     * `cd VCity/externals/laslib`
     * `mkdir Build && cd Build`
     * `cmake .. -DCMAKE_INSTALL_PREFIX=~/local/LASlib` (or choose an alternate installation directory in your home directory or a stable direcory why proper access rights)
     * `cd VCity`
     * `mkdir Build && cd Build`
     * `ccmake .. -DLASLIB_INCLUDE_DIR=~/local/include -DLASLIB_LIBRARY=~/local/lib/liblaslib.a`

### Building from sources
 * `git clone https://github.com/MEPP-team/VCity.git`
 * `cd VCity`
 * `Build && cd Build`
 * `cmake ..`
 * `make`

Post-install goodies (not VCity related):
 * QtCreator : `apt-get install qtcreator`

## Mac OS X install
### Installing dependencies
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

Assert that proj and geos sub-dependencies where installed (e.g. with `brew list | grep <pkg>` or `brew info <pkg>`). Otherwise install them manually:
 * `brew instal proj` (which is [proj4](https://github.com/OSGeo/proj.4/wiki))
 * `brew install geos`

### Building from sources
 * `git clone https://github.com/MEPP-team/VCity.git`
 * `cd VCity``
 * `Build && cd Build`
 * `cmake ..`
 * `make`

## Windows install
### Supported Windows platforms:
   * Windows versions: [Windows 7](https://en.wikipedia.org/wiki/Windows_7), [Windows 8](https://en.wikipedia.org/wiki/Windows_8), [Windows 8.1](https://en.wikipedia.org/wiki/Windows_8.1) or [Windows 10](https://en.wikipedia.org/wiki/Windows_10)
   * Be sure to apply all required "services packs" and "windows updates"

### Installing dependencies
 * [Visual Studio Express 2015](https://en.wikipedia.org/wiki/Microsoft_Visual_Studio_Express):
   * dowload [LIRIS local copy](https://download.gforge.liris.cnrs.fr/meppbin/windows/vs2015/Visual%20Studio%20Express%202015/Visual%20Studio%20Express%202015%20pour%20Windows%20Desktop.rar)
   * Unrar the downloaded archive file (e.g. with [7-zip](http://www.7-zip.org/)
   * Enter the extracted folder (`Express 2015 up2 pour Wndowds Desktop`) and launch the installer `wdexpress_full.exe`
   * On first invocation Visual Studio will ask for an email/passwd to "Connect to Visual Studio". Simply close this connection sub-window and proceed. You will be automatically granted with a 30 days free evaluation license. After this trial period: 
     * lauch Visual Studio,
     * go to the `Help` menu,
     * select the "A propos Microsoft Visual Studio..." entry
     * select the "State of the license" link (at the top of window)
     * on the right column (not the Connect youself one) proceed by providing e.g. some Outlook junk email (google on Outlook email in order to create it).
   * That pesky Express 2015 might complain from time to time that your licensed expired. In fact it didn't but MicroSpank is just "making sure" (hey it's a free version but they still keep you leash). In the right section of the pop-up window select "Search for a license update" provide the same email you already gave away (refer to previous section) and proceed with MicroSpank's lenghty (you might have to give your password three times, receive email codes...) "verification process".
 * VCity binary kit:
   * download [binary installer](https://download.gforge.liris.cnrs.fr/meppbin/windows/vs2015/VCITY/kits/VCITY_local_vs2015_64.7z)
   * extract content (installation will require 3.1 Go of free disk space) to a target directory which full path name length (from `C:\`) must be shorter than 50 characters: placing the extracted directory, named `VCITY_local_vs2015_64`,  into `C:\` or `C:\Programs` is ok (avoid unstable directories like "Desktop" or "My Dcouments").
   * Note: the extracted directory name (`VCITY_local_vs2015_64`) can be renamed if you need to.
 * Obtain VCity sources:
   * Install (from github) the [git for Windows](https://git-for-windows.github.io/) client
 * [Cmake version 3.4.3.](https://cmake.org/cmake/help/v3.4/release/3.4.html) (this version is Visual Studio 2015 aware):
   * Extract cmake from `VCITY_local_vs2015_64\_utils_\cmake-3.4.3-win32-x86.zip`
   * Extract NSIS from `VCITY_local_vs2015_64\_utils_\nsis-2.50-setup.exe``
 * Setting VCity related [environment variables](https://msdn.microsoft.com/en-us/library/windows/desktop/ms682653%28v=vs.85%29.aspx):
   * Warnings: for those upgrading from a previous VCity version, make sure to remove any `3DUse` related variable. Also make sure to remove any previously set `vs2012` variable

**Optional dependencies**
 * [Nullsoft Scriptable Install System (NSIS)](https://en.wikipedia.org/wiki/Nullsoft_Scriptable_Install_System)
   * only needed if you need to package VCity versions for redistribution
 * In order to build the documentation:
   * [Doxygen](http://www.stack.nl/~dimitri/doxygen/)
   * [Graphviz](http://www.graphviz.org/)

### Set up environment variables
In order to **build the "solution"** you will first need to setup the following new environment variables (in order to define the building context i.e. in order to indicate where the dependency libraries are to be found):
 * `VCITY_KIT_ROOT` to be `C:/VCITY_local_vs2015_64`.
 * `BOOST_ROOT` to be `%VCITY_KIT_ROOT%/boost_1_59_0`
 * When building with Qt5: `QT5_DIR` to be `%VCITY_KIT_ROOT%/Qt/Qt5.6.0/5.6/msvc2015_64`

**Watch out**: notice that for the above variables used path definitions where the separator is a  (`/`) character and NOT backslash (`\`) character ! (Note: this is because those variables are used by CMake which internally uses the Unix convention for path separators)

Once the "solution" is build (see below), and for the impatient user, in order to **run the application** you will also need to:
 * Add a traling`";C:\VCITY_local_vs2015_64\_bin_"` to your `PATH` environment variable (in order for the dynamic loader to find the dynamic libraries of the application dependencies)
 * Assuming you build the solution in e.g. the `c:\Users\MyLogin\VCity\MyBuild` directory add the following entries to your `PATH` environment variable:
  * Release mode `c:\Users\MyLogin\VCity\MyBuild\Release`
  * Debug mode `c:\Users\MyLogin\VCity\MyBuild\Debug`

**Watch out**: notice that for the above run time variables the separator is now classically for Windows a backslash (`\`) character !

**Tips and notes**:
 * in order to assert that the environement variables are properly set open a dos command and either use `set` and look for the variable(s) you are checking or check a specific variable with e.g. `echo %BOOST_ROOT%`.
 * When editing your `PATH` environment variable (and for versions of Windows requiring the edition of the pull `PATH` as a string) don't forget to add the proper `";" as  path separator
 * Note that prepending (as opposed to trailing) your `PATH` variable with the above mentionned directory paths might be safer (it will avoid possible conflicts with otherwise installed versions of QT or Graphviz...)

### Building from sources
 * Clone the [sources](https://github.com/MEPP-team/VCity.git) with a git client (e.g. [Tortoise git](https://tortoisegit.org/)

* Proceed with using cmake ([`cmake-gui.exe`](https://cmake.org/runningcmake/))
   * **Configure stage warning**: on the pop-up window that raises when configuring the cmake project assert that cmake detects the generator as being "Visual Studio 14 2015 **Win64**". Not only assert that the generator is Visual Studio 2015 (which is the 14th of Visual Studio) but also **assert that the generated code is 64 bits (Win64)**. If it is not properly set then set it manually (with the rolling down menu).
   * Set the optional cmake build flag `BUILD_EMBARKED_OSG-QT_34`to `ON`.
   * Unless you are a developer working on improving the regression tests, turn the cmake build flag `BUILD_UNMATURE_TESTS`to `OFF`.
   * Open (with Visual Studio) the resulting project `3DUSE.sln` located in your build subdirectory (`Bin` most often) and generate the solution

### Run the regresion tests
Select the `RUN_TESTS` project and launch (right click) the tests by invoking `Debug->Start a new instance`
