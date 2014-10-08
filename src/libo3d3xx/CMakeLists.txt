file(GLOB O3D3XX_SOURCES *.cpp)
add_library(o3d3xx SHARED ${O3D3XX_SOURCES})

find_library(LIB_glog NAMES glog)
find_library(LIB_boost_system NAMES boost_system)
find_library(LIB_boost_program_options NAMES boost_program_options)

target_link_libraries(
  o3d3xx
  ${LIB_glog}
  ${LIB_boost_system}
  ${LIB_boost_program_options}
  xmlrpc++
  xmlrpc_client++
  ${PCL_COMMON_LIBRARIES}
  ${OpenCV_LIBS}
  )
set_target_properties(o3d3xx PROPERTIES
  VERSION
  "${O3D3XX_VERSION_MAJOR}.${O3D3XX_VERSION_MINOR}.${O3D3XX_VERSION_PATCH}")
install(
  TARGETS o3d3xx
  DESTINATION lib
  EXPORT o3d3xx-targets
  )
