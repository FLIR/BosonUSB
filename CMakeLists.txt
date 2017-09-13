cmake_minimum_required(VERSION 2.8)
project( BosonUSB )
include_directories( ${MY_SOURCE_DIR}/src )
find_package( OpenCV REQUIRED )
add_executable( BosonUSB BosonUSB.cpp )
target_link_libraries( BosonUSB ${OpenCV_LIBS} )

