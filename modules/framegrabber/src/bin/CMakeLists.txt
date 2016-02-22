################################################
## Build the command-line tools
################################################
find_library(LIB_boost_program_options NAMES boost_program_options)

# Empty list
set(O3D3XX_FRAMEGRABBER_EXE_TARGETS)

#-----------------------------------------------
# o3d3xx-hz
#-----------------------------------------------
if(BUILD_EXE_HZ)
  add_executable(o3d3xx-hz o3d3xx-hz.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-hz
                          o3d3xx_framegrabber_shared
                          ${O3D3XX_CAMERA_LIBRARIES}
                          ${LIB_boost_program_options})
  else()
    target_link_libraries(o3d3xx-hz
                          o3d3xx_framegrabber_static
                          ${O3D3XX_CAMERA_LIBRARIES}
                          ${LIB_boost_program_options}
                          )
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-hz DESTINATION bin)

  list(APPEND O3D3XX_FRAMEGRABBER_EXE_TARGETS o3d3xx-hz)
else()
  message(STATUS "Skipping o3d3xx-hz")
endif(BUILD_EXE_HZ)

#-----------------------------------------------
# o3d3xx-schema
#-----------------------------------------------
if(BUILD_EXE_SCHEMA)
  add_executable(o3d3xx-schema o3d3xx-schema.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-schema
                          o3d3xx_framegrabber_shared
                          ${O3D3XX_CAMERA_LIBRARIES}
                          ${LIB_boost_program_options})
  else()
    target_link_libraries(o3d3xx-schema
                          o3d3xx_framegrabber_static
                          ${O3D3XX_CAMERA_LIBRARIES}
                          ${LIB_boost_program_options}
                          )
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-schema DESTINATION bin)

  list(APPEND O3D3XX_FRAMEGRABBER_EXE_TARGETS o3d3xx-schema)
else()
  message(STATUS "Skipping o3d3xx-schema")
endif(BUILD_EXE_SCHEMA)

#-----------------------------------------------
#-----------------------------------------------
#-----------------------------------------------
if(CMAKE_CROSSCOMPILING)
  foreach(exe_target ${O3D3XX_FRAMEGRABBER_EXE_TARGETS})
    set_target_properties(${exe_target} PROPERTIES
                          LINK_FLAGS "${CROSSTOOL_EXE_LINKER_FLAGS}"
                          COMPILE_FLAGS "${CROSSTOOL_COMPILE_FLAGS}")
  endforeach(exe_target)
endif(CMAKE_CROSSCOMPILING)
