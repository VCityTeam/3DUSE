mkdir Bin && cmake ..
  Mettre un install prefix e.g. dans tmp
make
  Regarder les rpath obtenus
make install
  Les rpath precedents on changé

Il n'y a pas d'App créé

WARNING: 

On debugging purposes:
  cmake --debug-output --trace

Concerning run-path
 * https://developer.apple.com/library/mac/documentation/DeveloperTools/Conceptual/DynamicLibraries/100-Articles/RunpathDependentLibraries.html
 * http://stackoverflow.com/questions/13656033/how-do-you-add-an-rpath-to-an-executable-in-cmake-on-build-not-install-on-osx
