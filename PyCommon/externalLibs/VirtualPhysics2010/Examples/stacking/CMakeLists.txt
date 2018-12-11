cmake_minimum_required(VERSION 2.8.6)
project(stacking)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})

file(GLOB srcs "main.cpp")

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

add_executable(stacking ${srcs})
target_link_libraries(stacking vpLib vpRenderer ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})