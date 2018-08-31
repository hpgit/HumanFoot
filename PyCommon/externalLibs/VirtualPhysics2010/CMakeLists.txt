cmake_minimum_required(VERSION 2.8.6)
project(VirtualPhysics2010)

set(default_build_type "Release")

include_directories(${CMAKE_HOME_DIRECTORY}/usr/include)

add_subdirectory(vpLib)
#add_subdirectory(vpRenderer)
#add_subdirectory(Examples)
