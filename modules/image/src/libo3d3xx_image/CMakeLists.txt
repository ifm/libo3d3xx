find_library(LIB_glog NAMES glog)
find_library(LIB_opencv_core NAMES opencv_core)
find_library(LIB_opencv_imgproc NAMES opencv_imgproc)
find_library(LIB_xmlrpcxx NAMES xmlrpc++)
find_library(LIB_xmlrpc_clientxx NAMES xmlrpc_client++)

file(GLOB O3D3XX_IMAGE_SOURCES *.cpp)

################################################
## Compile source once for both shared and static libs
################################################
add_library(o3d3xx_image OBJECT ${O3D3XX_IMAGE_SOURCES})
set_property(TARGET o3d3xx_image PROPERTY POSITION_INDEPENDENT_CODE 1)
if(CMAKE_CROSSCOMPILING)
  set_target_properties(o3d3xx_image PROPERTIES
                        COMPILE_FLAGS "${CROSSTOOL_COMPILE_FLAGS}")
endif(CMAKE_CROSSCOMPILING)

################################################
## Create the shared library
################################################
if(BUILD_SHARED_LIBS)
  add_library(o3d3xx_image_shared
              SHARED $<TARGET_OBJECTS:o3d3xx_image>)
    target_link_libraries(
      o3d3xx_image_shared
      ${O3D3XX_CAMERA_LIBRARIES}
      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
      ${LIB_glog}
      ${LIB_opencv_core}
      ${LIB_opencv_imgproc}
      ${LIB_xmlrpcxx}
      ${LIB_xmlrpc_clientxx}
      )

  set_target_properties(o3d3xx_image_shared PROPERTIES
    VERSION
    "${O3D3XX_VERSION_MAJOR}.${O3D3XX_VERSION_MINOR}.${O3D3XX_VERSION_PATCH}"
    OUTPUT_NAME o3d3xx_image
    LINKER_LANGUAGE CXX
    )

  install(
    TARGETS o3d3xx_image_shared
    DESTINATION lib
    EXPORT o3d3xx_image-targets
    )
endif(BUILD_SHARED_LIBS)

################################################
## Create the static library
################################################
if(BUILD_STATIC_LIBS)
  add_library(o3d3xx_image_static
              STATIC $<TARGET_OBJECTS:o3d3xx_image>)
  target_link_libraries(
    o3d3xx_image_static
    ${O3D3XX_CAMERA_LIBRARIES}
    ${O3D3XX_FRAMEGRABBER_LIBRARIES}
    ${LIB_glog}
    ${LIB_opencv_core}
    ${LIB_opencv_imgproc}
    ${LIB_xmlrpcxx}
    ${LIB_xmlrpc_clientxx}
    )

  set_target_properties(o3d3xx_image_static PROPERTIES
    LINKER_LANGUAGE CXX
    )

  install(
    TARGETS o3d3xx_image_static
    DESTINATION lib
    EXPORT o3d3xx_image-targets
    )
endif(BUILD_STATIC_LIBS)
