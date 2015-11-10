################################################
## Build the command-line tools
##
## If `BUILD_SHARED_LIBS' is `ON', we link against
## the shared library. If it is not, we link
## against the static library.
##
################################################

# Empty list
set(O3D3XX_EXE_TARGETS)

#-----------------------------------------------
# o3d3xx-version
#-----------------------------------------------
if(BUILD_EXE_VERSION)
  add_executable(o3d3xx-version o3d3xx-version.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-version o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-version o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-version DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-version)
else()
  message(STATUS "Skipping o3d3xx-version")
endif(BUILD_EXE_VERSION)

#-----------------------------------------------
# o3d3xx-reset
#-----------------------------------------------
if(BUILD_EXE_RESET)
  add_executable(o3d3xx-reset o3d3xx-reset.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-reset o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-reset o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-reset DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-reset)
else()
  message(STATUS "Skipping o3d3xx-reset")
endif(BUILD_EXE_RESET)

#-----------------------------------------------
# o3d3xx-ls
#-----------------------------------------------
if(BUILD_EXE_LS)
  add_executable(o3d3xx-ls o3d3xx-ls.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-ls o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-ls o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-ls DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-ls)
else()
  message(STATUS "Skipping o3d3xx-ls")
endif(BUILD_EXE_LS)

#-----------------------------------------------
# o3d3xx-viewer
#-----------------------------------------------
if(BUILD_EXE_VIEWER)
  add_executable(o3d3xx-viewer o3d3xx-viewer.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-viewer
      o3d3xx_shared
      ${PCL_VISUALIZATION_LIBRARIES}
      ${OpenCV_LIBS}
      )
  else()
    target_link_libraries(o3d3xx-viewer
      o3d3xx_static
      ${PCL_VISUALIZATION_LIBRARIES}
      ${OpenCV_LIBS}
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
# o3d3xx-dump
#-----------------------------------------------
if(BUILD_EXE_DUMP)
  add_executable(o3d3xx-dump o3d3xx-dump.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-dump o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-dump o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-dump DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-dump)
else()
  message(STATUS "Skipping o3d3xx-dump")
endif(BUILD_EXE_DUMP)

#-----------------------------------------------
# o3d3xx-config
#-----------------------------------------------
if(BUILD_EXE_CONFIG)
  add_executable(o3d3xx-config o3d3xx-config.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-config o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-config o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-config DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-config)
else()
  message(STATUS "Skipping o3d3xx-config")
endif(BUILD_EXE_CONFIG)

#-----------------------------------------------
# o3d3xx-rm
#-----------------------------------------------
if(BUILD_EXE_RM)
  add_executable(o3d3xx-rm o3d3xx-rm.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-rm o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-rm o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-rm DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-rm)
else()
  message(STATUS "Skipping o3d3xx-rm")
endif(BUILD_EXE_RM)

#-----------------------------------------------
# o3d3xx-reboot
#-----------------------------------------------
if(BUILD_EXE_REBOOT)
  add_executable(o3d3xx-reboot o3d3xx-reboot.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-reboot o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-reboot o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-reboot DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-reboot)
else()
  message(STATUS "Skipping o3d3xx-reboot")
endif(BUILD_EXE_REBOOT)

#-----------------------------------------------
# o3d3xx-hz
#-----------------------------------------------
if(BUILD_EXE_HZ)
  add_executable(o3d3xx-hz o3d3xx-hz.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-hz o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-hz o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-hz DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-hz)
else()
  message(STATUS "Skipping o3d3xx-hz")
endif(BUILD_EXE_HZ)

#-----------------------------------------------
# o3d3xx-imager-types
#-----------------------------------------------
if(BUILD_EXE_IMAGER_TYPES)
  add_executable(o3d3xx-imager-types o3d3xx-imager-types.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-imager-types o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-imager-types o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-imager-types DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-imager-types)
else()
  message(STATUS "Skipping o3d3xx-imager-types")
endif(BUILD_EXE_IMAGER_TYPES)

#-----------------------------------------------
# o3d3xx-ifm-import
#-----------------------------------------------
if(BUILD_EXE_IFM_IMPORT)
  add_executable(o3d3xx-ifm-import o3d3xx-ifm-import.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-ifm-import o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-ifm-import o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-ifm-import DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-ifm-import)
else()
  message(STATUS "Skipping o3d3xx-ifm-import")
endif(BUILD_EXE_IFM_IMPORT)

#-----------------------------------------------
# o3d3xx-ifm-export
#-----------------------------------------------
if(BUILD_EXE_IFM_EXPORT)
  add_executable(o3d3xx-ifm-export o3d3xx-ifm-export.cpp)

  if(BUILD_SHARED_LIBS)
    target_link_libraries(o3d3xx-ifm-export o3d3xx_shared)
  else()
    target_link_libraries(o3d3xx-ifm-export o3d3xx_static)
  endif(BUILD_SHARED_LIBS)

  install(TARGETS o3d3xx-ifm-export DESTINATION bin)

  list(APPEND O3D3XX_EXE_TARGETS o3d3xx-ifm-export)
else()
  message(STATUS "Skipping o3d3xx-ifm-export")
endif(BUILD_EXE_IFM_EXPORT)

#-----------------------------------------------
#-----------------------------------------------
#-----------------------------------------------
if(CMAKE_CROSSCOMPILING)
  foreach(exe_target ${O3D3XX_EXE_TARGETS})
    set_target_properties(${exe_target} PROPERTIES
                          LINK_FLAGS "${CROSSTOOL_EXE_LINKER_FLAGS}")
  endforeach(exe_target)
endif(CMAKE_CROSSCOMPILING)
