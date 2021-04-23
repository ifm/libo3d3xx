################################################
## Build the command-line tools
################################################

# Empty list
set(O3D3XX_IMAGE_EXE_TARGETS)

find_library(LIB_boost_program_options NAMES boost_program_options)
find_library(LIB_boost_system NAMES boost_system)

#-----------------------------------------------
# o3d3xx-viewer
#-----------------------------------------------
if(BUILD_EXE_VIEWER)
  add_executable(o3d3xx-viewer o3d3xx-viewer.cpp)

  if(BUILD_SHARED_LIBS)

    target_compile_definitions(o3d3xx-viewer PUBLIC ${PCL_DEFINITIONS})
    target_compile_options(o3d3xx-viewer PUBLIC ${PCL_COMPILE_OPTIONS})
    target_link_libraries(o3d3xx-viewer
      o3d3xx_image_shared
      ${O3D3XX_CAMERA_LIBRARIES}
      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
      ${PCL_VISUALIZATION_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${LIB_boost_program_options}
      ${LIB_boost_system}
      )
  else()
    target_link_libraries(o3d3xx-viewer
      o3d3xx_image_static
      ${O3D3XX_CAMERA_LIBRARIES}
      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
      ${PCL_VISUALIZATION_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${LIB_boost_program_options}
      ${LIB_boost_system}
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
