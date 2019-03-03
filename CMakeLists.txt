cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER    "gcc")
set(CMAKE_CXX_COMPILER  "g++")
# set(CMAKE_BUILD_TYPE Debug)

project( normalise )
find_package( OpenCV REQUIRED )
add_executable( normalise src/main.cpp )
target_link_libraries( normalise ${OpenCV_LIBS} )
