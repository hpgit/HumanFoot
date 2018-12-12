cmake_minimum_required(VERSION 3.12)

project(GUI)



include_directories(${Python3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.h" srcs)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_library(csDump MODULE ${srcs})
target_link_libraries(csDump ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_gl_LIBRARY})
set_target_properties(csDump PROPERTIES PREFIX "")