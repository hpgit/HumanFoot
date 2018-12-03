cmake_minimum_required(VERSION 2.8.6)
project(vpLib)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_compile_options(-fPIC)

#include_directories(${CMAKE_HOME_DIRECTORY}/usr/include)

OPTION(USE_OpenMP "Use OpenMP" OFF)
IF(USE_OpenMP)
    FIND_PACKAGE(OpenMP)
    IF(OPENMP_FOUND)
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    ELSEIF(APPLE)
        # -DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include" -DOpenMP_CXX_LIB_NAMES="omp" -DOpenMP_omp_LIBRARY=/usr/local/opt/libomp/lib/libomp.dylib
        SET(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
        SET(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
        SET(OpenMP_CXX_LIB_NAMES "omp")
        SET(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib")
        add_compile_definitions(__APPLE_OMP__)
    ENDIF()
ENDIF()

file(GLOB srcs "*.cpp")

add_library(vpLib ${srcs})
if(USE_OpenMP)
    target_link_libraries(vpLib ${OpenMP_omp_LIBRARY})
else()
    target_link_libraries(vpLib)
endif()
