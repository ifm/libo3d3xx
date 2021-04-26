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

# get version of ubuntu
include(ubuntu_version)
get_ubuntu_version(${UBUNTU_VERSION})

################################################
## Bring in dependent projects
################################################
include(o3d3xx_version)
find_package(o3d3xx_camera ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_framegrabber ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_image ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_pcicclient ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(PCL 1.7.1 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Threads REQUIRED)
find_library(LIB_boost_system NAMES boost_system)

# Fix this: https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
if(NOT (UBUNTU_VERSION VERSION_LESS "16.04"))
  list(REMOVE_ITEM PCL_IO_LIBRARIES "vtkproj4")
endif()

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

################################################
## Build the examples
################################################

add_executable(ex-100k ex-100k.cpp)
target_link_libraries(ex-100k
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )

add_executable(ex-cartesian ex-cartesian.cpp)
target_link_libraries(ex-cartesian
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${LIB_boost_system}
                      )
add_executable(ex-cartesian_compare ex-cartesian_compare.cpp)
target_link_libraries(ex-cartesian_compare
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${PCL_IO_LIBRARIES}
                      ${LIB_boost_system}
                      )

add_executable(ex-file_io ex-file_io.cpp)
target_link_libraries(ex-file_io
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_IO_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )

add_executable(ex-framegrabber_recycling ex-framegrabber_recycling.cpp)
target_link_libraries(ex-framegrabber_recycling
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${LIB_boost_system}
                      )

add_executable(ex-exposure_times ex-exposure_times.cpp)
target_link_libraries(ex-exposure_times
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )
add_executable(ex-change_ip ex-change_ip.cpp)
target_link_libraries(ex-change_ip
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${LIB_boost_system}
                      )
add_executable(ex-pcicclient_set_io ex-pcicclient_set_io.cpp)
target_link_libraries(ex-pcicclient_set_io
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_PCICCLIENT_LIBRARIES}
                      ${LIB_boost_system}
                      )

add_executable(ex-pcicclient_async_messages ex-pcicclient_async_messages.cpp)
target_link_libraries(ex-pcicclient_async_messages
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_PCICCLIENT_LIBRARIES}
                      ${LIB_boost_system}
                      )

add_executable(ex-fast_app_switch ex-fast_app_switch.cpp)
target_link_libraries(ex-fast_app_switch
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${O3D3XX_PCICCLIENT_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${LIB_boost_system}
                      )

add_executable(ex-oem_is_booted ex-oem_is_booted.cpp)
target_link_libraries(ex-oem_is_booted
                      ${O3D3XX_CAMERA_LIBRARIES}
                      )

add_executable(ex-pcic_dos_attack ex-pcic_dos_attack.cpp)
target_link_libraries(ex-pcic_dos_attack
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${O3D3XX_PCICCLIENT_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${LIB_boost_system}
                      ${CMAKE_THREAD_LIBS_INIT}
                      )

add_executable(issue-142 issue-142.cpp)
target_link_libraries(issue-142
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${O3D3XX_PCICCLIENT_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${LIB_boost_system}
                      ${CMAKE_THREAD_LIBS_INIT}
                      )
