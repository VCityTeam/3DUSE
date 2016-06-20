
## Concerning OSG-QT extension
**The context**:
 * VCity supports both QT4 and QT5.
 * **Coherence constraint**:
   Not surprisingly OSG-QT depends both on OSG and QT. As Robert Osfield puts it (refer to this [OSG discussion thread](http://forum.openscenegraph.org/viewtopic.php?t=13713)):
   ```
   If you application is using the osgQt library and this lib is linked
   to Qt4 then mixing the viability of mixing Qt5 libs in your application
   will be down to Qt4/5 compatibility, something I can't answer as I'm no
   Qt expert but I guess it's reasonable to assume that this could cause
   problems.
   If you can't mix Qt5 and Qt4 within a single application then you'll
   either have to rebuild the OSG against Qt5 or avoid using components
   of the OSG that use Qt - primarily this is osgQt but there a few
   plugins use it too.
   ```
   **Conclusion (coherence constraint)**: if we build our application to say QT4 then the OSG-QT extension the application has to link against has to be build against QT4 also (that is the same QT version)
 * **Packager's choice constraint**
   As of 2016, QT4 is still the dominant version in terms of usages. Binary distribution packagers facing the QT4/QT5 choice, when they do provide OSG with the QT extension, will thus most often choose to link against QT4. Whatever choice the packagers make, if one wishes to build VCity against the "other" version of QT, then a mismatch will appear.
 * **Travis is unable to recompile OSG-QT `--with-qt` option**
   The OSG developers respect the `DESIRED_QT_VERSION` good practice ([more]( http://stackoverflow.com/questions/20317354/how-to-compile-open-scene-graph-3-2-with-qt-5-1-and-cmake)) which enables packagers to provide optional builds from sources. For example on OSX, OSGQT can be obtained with `brew install open-scene-graph --with-qt` for QT4 linking and `brew install open-scene-graph --with-qt5` for QT5 linking. Alas such rebuilds of OSG from source take quite some time and Travis usually exits from the above brew install directives prior to completion (with a message like `No output has been received in the last 10m0s, this potentially indicates a stalled build or something wrong with the build itself.`).
   **Conclusion (partial)**: there will some contexts were we have to deal with the default binary package choices.
 * **Discovering the underlying QT version of OSGQT is hard for CMake**
   Alas it is not straightforward for a CMake project to know whether the installed OSG-QT extension they discover natively with `find_package()` was built against QT4 or QT5.  
 * **Mismatches are discovered only at runtime**
  As [this discussion](http://forum.openscenegraph.org/viewtopic.php?t=14999) explains it, if there is a **mismatch between the QT version** of your application and the version used by the OSG-QT binary package, then your application won't detect it at build/link stage and your **application will crash at run time.**
 * Note: this QT4/QT5 incompatibility mix is symmetric. For example if you application requires linking against QT4 but the installed version of OSG QT extension was linked against QT5, then you will have to rebuild OSG against QT4.

**Discussion**:
VCity's CMakeFile designer has to choose among various possible options the one he decides to offer to his users:
 1. the **you're alone** service level:
   this consists in requiring the project builder to provide an OSG-Qt extension linked against the same version of Qt that he wishes VCity to use. Failure to provide OSG-Qt (at all) will be reported by cmake and build stopped. But failure to provide a mathcing version won't be detected by CMake at build time. Then regression tests (as well as any trial to execute things) will fail but leaving the user with very few clues of what is going on. Note that such level won't allow VCity to be built on Travis for OSX (as of 10.11.X Homebrew provided OSG binary package doesn't come up with any QT support at all, and rebuilding from source is killed on duty by Travis as above mentioned).
 2. the **embark OSGQT** service level:
   VCity can choose to embark its own copy of the OSG-Qt extension. CMake could then ignore any packaging system installed version of OSG-Qt and always use its own copy:
    * The advantage of this strategy is that there is no possible mismatch since OSG-Qt extension is always recompiled by VCity against the chosen Qt version. Hence build and run are robust.
    * The drawback of systematically ignoring the native OSG-Qt is for VCity developers not being able to confront VCity to the current version of OSG-Qt. At some point the OSG provided code for OSG-Qt will drift apart from VCity's embarked version without convenient any easy build/test means to measure the drift of VCity towards the official OSG version.
 3. the **embark OS-QT and letting the user manually choose** service level:
   This level can be described as the above "embark OSG-Qt" with the right to choose for the CMake user. This solution still has:
   * the "embark/fork OSG-Qt" associated risks and also the "you're alone" inconveniences i.e. a "not so aware" user will face run (load) time failures without much clues...
   * the advantage that an aware user (including developers) will have the freedom to make his own trials against the natively build OSG-Qt. Besides VCity' CMakeFiles won't need to be polluted with obfuscated `TRAVIS` internalized flags to "do the right thing" when on Travis.
 4. the **automatic plan B** service level:
   The CMakeFiles are here able to detect the mismatch and automatically fold back to the embarked copy of OSG-Qt as plan B:
    * the advantage is build/run robustness
    * the drawback is that the CMakeFile developer is facing the hard/daunting task of detecting (in portable manner) which version of QT underlies OSG-Qt.
 5. the **automatic yet [unpluggable](https://en.wiktionary.org/wiki/unpluggable) plan B** service level:
    This level is basically the previous one with the additional possibility of manually switching to the embarked version.

**Conclusion**
For the time being VCity **resolves to the "embark OSGQT and letting the user manually choose"** service level (mainly because automatic detection looks very hard to realize).

**Deployment notes**:
 * Ubuntu 14.04:
   * Qt4:
     * QT4 is the default
     * OSG package (named libopenscenegraph-dev) comes by default with qt support (library `/usr/lib/libosgQt.so`)
   * Qt5
     * Installing Qt5: `sudo apt-get -y install qtbase5-dev qt5-default`
     * How to recompile OpenSceneGraph with Qt5 support ?
 * OSX:
   * Qt4 is the default
   * OSG packages comes without qt support
   * Rebuilding OSG with Qt support is done respectively with `brew install open-scene-graph --with-qt` or `brew install open-scene-graph --with-qt5` 