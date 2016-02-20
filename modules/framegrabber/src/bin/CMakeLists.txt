################################################
## Build the command-line tools
################################################

# Empty list
set(O3D3XX_FRAMEGRABBER_EXE_TARGETS)

find_library(LIB_o3d3xx_camera
             NAMES o3d3xx_camera libo3d3xx_camera_static.a)

#-----------------------------------------------
# o3d3xx-hz
#-----------------------------------------------
if(BUILD_EXE_HZ)
  add_executable(o3d3xx-hz o3d3xx-hz.cpp)

  target_link_libraries(o3d3xx-hz
                        o3d3xx_framegrabber_shared
                        ${LIB_o3d3xx_camera}
                        boost_program_options)

  install(TARGETS o3d3xx-hz DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-hz)
else()
  message(STATUS "Skipping o3d3xx-hz")
endif(BUILD_EXE_HZ)

#-----------------------------------------------
# o3d3xx-schema
#-----------------------------------------------
if(BUILD_EXE_SCHEMA)
  add_executable(o3d3xx-schema o3d3xx-schema.cpp)

  target_link_libraries(o3d3xx-schema
                        o3d3xx_framegrabber_shared
                        ${LIB_o3d3xx_camera}
                        boost_program_options)

  install(TARGETS o3d3xx-schema DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-schema)
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
