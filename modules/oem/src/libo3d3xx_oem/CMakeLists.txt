find_library(LIB_glog NAMES glog)
find_library(LIB_opencv_core NAMES opencv_core)
find_library(LIB_resultsync NAMES resultsync)

file(GLOB O3D3XX_OEM_SOURCES *.cpp)

add_library(o3d3xx_oem OBJECT ${O3D3XX_OEM_SOURCES})
set_property(TARGET o3d3xx_oem PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(o3d3xx_oem_shared SHARED $<TARGET_OBJECTS:o3d3xx_oem>)
target_link_libraries(o3d3xx_oem_shared
                      ${LIB_glog}
                      ${LIB_opencv_core}
                      ${LIB_resultsync}
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT}
                      )

set_target_properties(o3d3xx_oem_shared PROPERTIES
  VERSION
  "${O3D3XX_VERSION_MAJOR}.${O3D3XX_VERSION_MINOR}.${O3D3XX_VERSION_PATCH}"
  OUTPUT_NAME o3d3xx_oem
  LINKER_LANGUAGE CXX
  )

install(TARGETS o3d3xx_oem_shared DESTINATION lib)
