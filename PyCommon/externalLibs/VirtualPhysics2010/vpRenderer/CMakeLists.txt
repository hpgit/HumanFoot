cmake_minimum_required(VERSION 2.8.6)
project(vpRenderer)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_compile_options(-fPIC)

find_package(OpenGL REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${CMAKE_HOME_DIRECTORY}/usr/include)

file(GLOB srcs "*.c" "*.cpp" "*.h")

add_library(vpRenderer ${srcs})
target_link_libraries(vpRenderer vpLib ${OPENGL_LIBRARIES})
