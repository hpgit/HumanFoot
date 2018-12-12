cmake_minimum_required(VERSION 3.12)

project(Math)

include_directories(${Python3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.h" srcs)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_library(csMath MODULE ${srcs})
target_link_libraries(csMath ${Python3_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(csMath PROPERTIES PREFIX "")
