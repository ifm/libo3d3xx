find_library(LIB_glog NAMES glog)
find_library(LIB_xmlrpcxx NAMES xmlrpc++)
find_library(LIB_xmlrpc_clientxx NAMES xmlrpc_client++)

file(GLOB O3D3XX_FRAMEGRABBER_SOURCES *.cpp)

################################################
## Compile source once for both shared and static libs
################################################
add_library(o3d3xx_framegrabber OBJECT ${O3D3XX_FRAMEGRABBER_SOURCES})
set_property(TARGET o3d3xx_framegrabber PROPERTY POSITION_INDEPENDENT_CODE 1)
if(CMAKE_CROSSCOMPILING)
  set_target_properties(o3d3xx_framegrabber PROPERTIES
                        COMPILE_FLAGS "${CROSSTOOL_COMPILE_FLAGS}")
endif(CMAKE_CROSSCOMPILING)

################################################
## Create the shared library
################################################
if(BUILD_SHARED_LIBS)
  add_library(o3d3xx_framegrabber_shared
              SHARED $<TARGET_OBJECTS:o3d3xx_framegrabber>)
    target_link_libraries(
      o3d3xx_framegrabber_shared
      ${O3D3XX_CAMERA_LIBRARIES}
      ${LIB_glog}
      ${Boost_LIBRARIES}
      ${LIB_xmlrpcxx}
      ${LIB_xmlrpc_clientxx}
      ${CMAKE_THREAD_LIBS_INIT}
      )

  set_target_properties(o3d3xx_framegrabber_shared PROPERTIES
    VERSION
    "${O3D3XX_VERSION_MAJOR}.${O3D3XX_VERSION_MINOR}.${O3D3XX_VERSION_PATCH}"
    OUTPUT_NAME o3d3xx_framegrabber
    LINKER_LANGUAGE CXX
    )

  install(
    TARGETS o3d3xx_framegrabber_shared
    DESTINATION lib
    EXPORT o3d3xx_framegrabber-targets
    )
endif(BUILD_SHARED_LIBS)

################################################
## Create the static library
################################################
if(BUILD_STATIC_LIBS)
  add_library(o3d3xx_framegrabber_static
              STATIC $<TARGET_OBJECTS:o3d3xx_framegrabber>)
  target_link_libraries(
    o3d3xx_framegrabber_static
    ${O3D3XX_CAMERA_LIBRARIES}
    ${LIB_glog}
    ${Boost_LIBRARIES}
    ${LIB_xmlrpcxx}
    ${LIB_xmlrpc_clientxx}
    )

  set_target_properties(o3d3xx_framegrabber_static PROPERTIES
    LINKER_LANGUAGE CXX
    )

  install(
    TARGETS o3d3xx_framegrabber_static
    DESTINATION lib
    EXPORT o3d3xx_framegrabber-targets
    )
endif(BUILD_STATIC_LIBS)
