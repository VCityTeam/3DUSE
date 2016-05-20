mkdir Bin && cmake ..
 * `make`
 * `./mystest` is effective
   * observe the resulting rpathes with `otool -L mytest libmylib.dylib subdir/libmylib2.dylib`
 * `make install` places its result within /tmp/junk

The problemS:
 1. the installed executable is brain damaged 
   If one disactivates the install(CODE) section (whose purpose is to
   supposedly to build an App) and that one applies the make install
   then the execution of `/tmp/junk/mystest` is BROKEN.
   For some reason the LC_RPATH entries of mytest seem to be altered
   when the installation takes place. To assert this compare the
   respectives results of:
    * `otool -l mytest | grep LC_RPATH` (which point to subdir)
    * `otool -l /tmp/junk/mytest | grep LC_RPATH` (which is empty)
 2. No App (application) is being build

The possible cause:
  What happens is that the libmylib2.dylib target is not yet installed
  when the packaging is required to be build ?

On debugging purposes:
  cmake --debug-output --trace

Concerning run-path
 * https://developer.apple.com/library/mac/documentation/DeveloperTools/Conceptual/DynamicLibraries/100-Articles/RunpathDependentLibraries.html
 * http://stackoverflow.com/questions/13656033/how-do-you-add-an-rpath-to-an-executable-in-cmake-on-build-not-install-on-osx
