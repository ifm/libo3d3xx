find_library(LIB_glog NAMES glog)
find_library(LIB_boost_system NAMES boost_system)
find_library(LIB_boost_program_options NAMES boost_program_options)
find_library(LIB_xmlrpcxx NAMES xmlrpc++)
find_library(LIB_xmlrpc_clientxx NAMES xmlrpc_client++)
find_library(LIB_opencv_core NAMES opencv_core)
find_library(LIB_opencv_imgproc NAMES opencv_imgproc)

file(GLOB O3D3XX_SOURCES *.cpp)

################################################
## Compile source once for both shared and static libs
################################################
add_library(o3d3xx OBJECT ${O3D3XX_SOURCES})
set_property(TARGET o3d3xx PROPERTY POSITION_INDEPENDENT_CODE 1)

################################################
##
## NOTE: While we could create a libary of the
## same name for both shared and static, it is
## my understading that while this will work
## fine on both Linux and Mac OS X, Windows will
## have a problem with that. So, the strategy
## is to produce a `libo3d3xx.so' and a
## `libo3d3xx_static.a'.
##
################################################

################################################
## Create the shared library
################################################
if(BUILD_SHARED_LIBS)
  add_library(o3d3xx_shared SHARED $<TARGET_OBJECTS:o3d3xx>)
    target_link_libraries(
      o3d3xx_shared
      ${LIB_glog}
      ${LIB_boost_system}
      ${LIB_boost_program_options}
      ${LIB_xmlrpcxx}
      ${LIB_xmlrpc_clientxx}
      ${LIB_opencv_core}
      ${LIB_opencv_imgproc}
      ${CMAKE_THREAD_LIBS_INIT}
      )

  set_target_properties(o3d3xx_shared PROPERTIES
    VERSION
    "${O3D3XX_VERSION_MAJOR}.${O3D3XX_VERSION_MINOR}.${O3D3XX_VERSION_PATCH}"
    OUTPUT_NAME o3d3xx
    LINKER_LANGUAGE CXX
    )

  install(
    TARGETS o3d3xx_shared
    DESTINATION lib
    EXPORT o3d3xx-targets
    )
endif(BUILD_SHARED_LIBS)

################################################
## Create the static library
################################################
if(BUILD_STATIC_LIBS)
  add_library(o3d3xx_static STATIC $<TARGET_OBJECTS:o3d3xx>)
  target_link_libraries(
    o3d3xx_static
    ${LIB_glog}
    ${LIB_boost_system}
    ${LIB_boost_program_options}
    ${LIB_xmlrpcxx}
    ${LIB_xmlrpc_clientxx}
    ${LIB_opencv_core}
    ${LIB_opencv_imgproc}
    )

  set_target_properties(o3d3xx_static PROPERTIES
    LINKER_LANGUAGE CXX
    )

  install(
    TARGETS o3d3xx_static
    DESTINATION lib
    EXPORT o3d3xx-targets
    )
endif(BUILD_STATIC_LIBS)
