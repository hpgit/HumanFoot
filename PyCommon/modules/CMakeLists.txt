cmake_minimum_required(VERSION 3.12)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost 1.62.0 REQUIRED COMPONENTS python3)
find_package(OpenGL REQUIRED)

add_subdirectory(GUI)
add_subdirectory(Math)
#add_subdirectory(Mesh)
#add_subdirectory(Optimization)
#add_subdirectory(Renderer)
#add_subdirectory(Simulator)
#add_subdirectory(VirtualPhysics)