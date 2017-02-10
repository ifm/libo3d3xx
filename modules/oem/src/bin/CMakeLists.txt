################################################
## Build the command-line tools
################################################
find_library(LIB_boost_program_options NAMES boost_program_options)

#-----------------------------------------------
# o3d3xx-oem-jitter
#-----------------------------------------------
if(BUILD_EXE_JITTER)
  add_executable(o3d3xx-oem-jitter o3d3xx-oem-jitter.cpp)

  target_link_libraries(o3d3xx-oem-jitter
                        o3d3xx_oem_shared
                        ${OpenCV_LIBRARIES}
                        ${O3D3XX_CAMERA_LIBRARIES}
                        ${LIB_boost_program_options})

  install(TARGETS o3d3xx-oem-jitter DESTINATION bin)
else()
  message(STATUS "Skipping o3d3xx-oem-jitter")
endif()
