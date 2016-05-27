add_library(mylib2 SHARED mylib2.c)
install(TARGETS mylib2
 BUNDLE  DESTINATION .
 LIBRARY DESTINATION .
 )
