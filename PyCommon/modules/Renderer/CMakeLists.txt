file(GLOB srcs "csRenderer.cpp" "csRenderer.h" "stdafx.cpp" "stdafx.h")
file(GLOB ims_srcs "csIMSRenderer.cpp" "csIMSRenderer.h" "stdafx.cpp" "stdafx.h")
file(GLOB vp_srcs "csVpRenderer.cpp" "csVpRenderer.h" "stdafx.cpp" "stdafx.h")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Renderer)

include_directories(${OPENGL_INCLUDE_DIRS})

add_library(csRenderer ${SHARED_TYPE} ${srcs})
target_link_libraries(csRenderer ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES})
set_target_properties(csRenderer PROPERTIES PREFIX "")

add_library(csIMSRenderer ${SHARED_TYPE} ${ims_srcs})
target_link_libraries(csIMSRenderer ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES})
set_target_properties(csIMSRenderer PROPERTIES PREFIX "")

add_library(csVpRenderer ${SHARED_TYPE} ${vp_srcs})
target_link_libraries(csVpRenderer ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${VIRTUALPHYSICS_LIBRARIES})
set_target_properties(csVpRenderer PROPERTIES PREFIX "")
