project(modules)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost 1.62.0 REQUIRED COMPONENTS python3 numpy3)
find_package(OpenGL REQUIRED)
find_package(VirtualPhysics REQUIRED)

option(USE_OPENMP "Use OpenMP" OFF)
if (USE_OPENMP)
    find_package(OpenMP)
endif ()

include_directories(usr/include)
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

if (APPLE)
    set(SHARED_TYPE MODULE)
else()
    set(SHARED_TYPE SHARED)
endif ()

add_subdirectory(GUI)
add_subdirectory(Math)
#add_subdirectory(Mesh)
#add_subdirectory(Optimization)
add_subdirectory(Renderer)
add_subdirectory(Simulator)
#add_subdirectory(VirtualPhysics)