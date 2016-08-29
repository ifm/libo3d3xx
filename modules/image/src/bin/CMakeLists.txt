################################################
## Build the command-line tools
################################################

# Empty list
set(O3D3XX_IMAGE_EXE_TARGETS)

find_library(LIB_boost_program_options NAMES boost_program_options)

#-----------------------------------------------
# o3d3xx-viewer
#-----------------------------------------------
if(BUILD_EXE_VIEWER)
  add_executable(o3d3xx-viewer o3d3xx-viewer.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-viewer
      o3d3xx_image_shared
      ${O3D3XX_CAMERA_LIBRARIES}
      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
      ${PCL_VISUALIZATION_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${LIB_boost_program_options}
      )
  else()
    target_link_libraries(o3d3xx-viewer
      o3d3xx_image_static
      ${O3D3XX_CAMERA_LIBRARIES}
      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
      ${PCL_VISUALIZATION_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${LIB_boost_program_options}
      )
  endif(BUILD_SHARED_LIBS)

  set_target_properties(o3d3xx-viewer PROPERTIES
    AUTOMOC TRUE
    COMPILE_FLAGS "-Wno-deprecated"
    )

  install(TARGETS o3d3xx-viewer DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-viewer)
else()
  message(STATUS "Skipping o3d3xx-viewer")
endif(BUILD_EXE_VIEWER)

# !!!!!!!!!!!! REMOVE THIS !!!!!!!!!!!!!!!
add_executable(o3d3xx-crash-test o3d3xx-crash-test.cpp)
target_link_libraries(o3d3xx-crash-test
  o3d3xx_image_shared
  boost_system
  pthread
  opencv_core
      )
install(TARGETS o3d3xx-crash-test DESTINATION bin)
list(APPEND O3D3XX_EXE_TARGETS o3d3xx-crash-test)

#-----------------------------------------------
#-----------------------------------------------
#-----------------------------------------------
if(CMAKE_CROSSCOMPILING)
  foreach(exe_target ${O3D3XX_IMAGE_EXE_TARGETS})
    set_target_properties(${exe_target} PROPERTIES
                          LINK_FLAGS "${CROSSTOOL_EXE_LINKER_FLAGS}"
                          COMPILE_FLAGS "${CROSSTOOL_COMPILE_FLAGS}")
  endforeach(exe_target)
endif(CMAKE_CROSSCOMPILING)
