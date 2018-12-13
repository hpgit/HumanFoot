file(GLOB srcs "*.cpp" "*.h" srcs)
file(GLOB srcs "*.cpp" "*.h" srcs)
file(GLOB srcs "*.cpp" "*.h" srcs)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Renderer)

add_library(csIMSRenderer MODULE ${srcs})
target_link_libraries(csIMSRenderer ${Python3_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(csIMSRenderer PROPERTIES PREFIX "")

add_library(csRenderer MODULE ${srcs})
target_link_libraries(csRenderer ${Python3_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(csRenderer PROPERTIES PREFIX "")

add_library(csVpRenderer MODULE ${srcs})
target_link_libraries(csVpRenderer ${Python3_LIBRARIES} ${Boost_LIBRARIES} )
set_target_properties(csVpRenderer PROPERTIES PREFIX "")
