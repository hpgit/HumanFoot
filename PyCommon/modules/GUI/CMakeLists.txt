file(GLOB srcs "*.cpp" "*.h" srcs)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/GUI)

add_library(csDump ${SHARED_TYPE} ${srcs})
target_link_libraries(csDump ${Python3_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_gl_LIBRARY})
set_target_properties(csDump PROPERTIES PREFIX "")