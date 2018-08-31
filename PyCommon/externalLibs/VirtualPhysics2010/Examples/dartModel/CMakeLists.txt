cmake_minimum_required(VERSION 2.8.6)
project(dartModel)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})

file(GLOB srcs "*.cpp")

find_package(tinyxml2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${TinyXML2_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

add_executable(dartModel ${srcs})
target_link_libraries(dartModel vpLib vpRenderer tinyxml2 ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})