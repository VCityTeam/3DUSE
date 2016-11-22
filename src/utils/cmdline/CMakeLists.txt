configure_file( 
  Application-config.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}-config.cpp
  @ONLY )

add_executable( ${APP_NAME}-config ${APP_NAME}-config.cpp )
add_test( TEST_PRINT_CONFIG ${APP_NAME}-config )
