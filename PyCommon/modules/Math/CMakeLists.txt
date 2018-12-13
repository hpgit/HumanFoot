file(GLOB srcs "*.cpp" "*.h" srcs)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Math)

add_library(csMath ${SHARED_TYPE} ${srcs})
target_link_libraries(csMath ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${VIRTUALPHYSICS_LIBRARIES})
set_target_properties(csMath PROPERTIES PREFIX "")
