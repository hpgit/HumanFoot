file(GLOB srcs "myGeom.cpp" "myGeom.h" "stdafx.cpp" "stdafx.h")
file(GLOB vp_world_srcs "csVpWorld.cpp" "csVpWorld.h")
file(GLOB vp_model_srcs "csVpModel.cpp" "csVpModel.h")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Simulator)

if (OpenMP_CXX_FOUND)
    add_compile_definitions(USE_OPENMP)
endif ()

add_library(csVpWorld ${SHARED_TYPE} ${vp_world_srcs} ${srcs})
if (OpenMP_CXX_FOUND)
    target_link_libraries(csVpWorld ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${VIRTUALPHYSICS_LIBRARIES} OpenMP::OpenMP_CXX)
else()
    target_link_libraries(csVpWorld ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${VIRTUALPHYSICS_LIBRARIES})
endif ()
set_target_properties(csVpWorld PROPERTIES PREFIX "")

add_library(csVpModel ${SHARED_TYPE} ${vp_model_srcs} ${srcs})
target_link_libraries(csVpModel ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${VIRTUALPHYSICS_LIBRARIES})
set_target_properties(csVpModel PROPERTIES PREFIX "")
