project(O3D3XX_EXAMPLES)
cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_BUILD_TYPE Release) # Release or Debug

set(CMAKE_MODULE_PATH
    ${O3D3XX_EXAMPLES_SOURCE_DIR}/../../cmake/modules
    ${CMAKE_MODULE_PATH}
    )

# force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

################################################
## Bring in dependent projects
################################################
include(o3d3xx_version)
find_package(o3d3xx_camera ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_framegrabber ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_image ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(PCL 1.7.1 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${O3D3XX_IMAGE_INCLUDE_DIRS}
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_IMAGE_LIBRARY_DIR}
  ${O3D3XX_CAMERA_LIBRARY_DIR}
  ${O3D3XX_FRAMEGRABBER_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  )
add_definitions(
  ${PCL_DEFINITIONS}
  ${OpenCV_DEFINITIONS}
  )

##
## ex-100k.cpp
##
add_executable(ex-100k ex-100k.cpp)
target_link_libraries(ex-100k
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES}
                      ${OpenCV_LIBS}
                      )
