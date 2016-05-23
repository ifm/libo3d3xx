project(O3D3XX_IMAGE)
cmake_minimum_required(VERSION 2.8.12)
set(O3D3XX_IMAGE_MODULE_NAME "libo3d3xx-image")

set(CPACK_SET_DESTDIR true)
if(NOT CMAKE_CROSSCOMPILING)
  set(CMAKE_INSTALL_PREFIX /usr)
else()
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SYSROOT}/usr)
  set(CPACK_INSTALL_PREFIX /usr)
endif()

set(CMAKE_BUILD_TYPE Release) # Release or Debug

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_STATIC_LIBS "Build static libraries" ON)
if(NOT CMAKE_CROSSCOMPILING)
  option(BUILD_EXE_VIEWER "Build o3d3xx-viewer" ON)
else()
  option(BUILD_EXE_VIEWER "Build o3d3xx-viewer" OFF)
endif(NOT CMAKE_CROSSCOMPILING)

set(TARGET_IP "192.168.0.68")
set(TARGET_USER "lovepark")
set(TARGET_DIR "/home/lovepark/debs/")

################################################
################################################
## Most users should have no concern about what
## is beyond this line. Edit at your own risk.
################################################
################################################
if(NOT (BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS))
  message(FATAL_ERROR
    "One of `BUILD_STATIC_LIBS' or `BUILD_SHARED_LIBS' must be `ON'")
endif(NOT (BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS))

set(CMAKE_MODULE_PATH
    ${O3D3XX_IMAGE_SOURCE_DIR}/../../cmake/modules
    ${CMAKE_MODULE_PATH}
    )

# force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

################################################
## Variable-interpolate the version number
################################################
include(o3d3xx_version)
file(COPY
  ${O3D3XX_IMAGE_SOURCE_DIR}/o3d3xx_image-config-version.cmake.in
  DESTINATION ${O3D3XX_IMAGE_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_IMAGE_BINARY_DIR}/o3d3xx_image-config-version.cmake.in
  ${O3D3XX_IMAGE_BINARY_DIR}/o3d3xx_image-config-version.cmake
  @ONLY
  )

################################################
## Auto-detect the version of Ubuntu ... this
## can be overridden on the command line with:
##
## cmake -DUBUNTU_VERSION:STRING=16.04 ..
##
################################################
include(ubuntu_version)
get_ubuntu_version(${UBUNTU_VERSION})

################################################
## Bring in dependent projects
################################################

find_package(o3d3xx_camera ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_framegrabber ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
if (BUILD_EXE_VIEWER)
  find_package(PCL 1.7 REQUIRED COMPONENTS visualization)
  if(NOT (UBUNTU_VERSION VERSION_LESS "16.04"))
    # https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
    list(REMOVE_ITEM PCL_VISUALIZATION_LIBRARIES "vtkproj4")
  endif()

  find_package(Boost REQUIRED COMPONENTS program_options system)
endif(BUILD_EXE_VIEWER)
find_package(OpenCV 2.4 REQUIRED)

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

file(COPY ${O3D3XX_IMAGE_SOURCE_DIR}/include
  DESTINATION ${O3D3XX_IMAGE_BINARY_DIR}
  )

include_directories(
  ${O3D3XX_IMAGE_BINARY_DIR}/include
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_IMAGE_BINARY_DIR}/lib
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
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping unit tests")
endif(BUILD_TESTS)

################################################
## Manage installation process
################################################
if(BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS)
  install(
    EXPORT o3d3xx_image-targets DESTINATION lib/o3d3xx_image)
endif(BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS)

install(DIRECTORY ${O3D3XX_IMAGE_BINARY_DIR}/include
  DESTINATION .
  PATTERN "*.in" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "*~" EXCLUDE
  )

install(FILES
  ${O3D3XX_IMAGE_BINARY_DIR}/o3d3xx_image-config-version.cmake
  DESTINATION lib/o3d3xx_image
  )

set(CPACK_SET_DESTDIR "on")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${O3D3XX_IMAGE_MODULE_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "IFM Efector O3D3xx OpenCV/PCL Image Lib")
set(CPACK_PACKAGE_DESCRIPTION
  "This package bridges the O3D3xx camera to OpenCV and PCL.")
set(CPACK_PACKAGE_VENDOR "Love Park Robotics, LLC")
set(CPACK_PACKAGE_CONTACT "Tom Panzarella <tom@loveparkrobotics.com>")
set(CPACK_PACKAGE_VERSION_MAJOR
  "${O3D3XX_VERSION_MAJOR}"
  )
set(CPACK_PACKAGE_VERSION_MINOR
  "${O3D3XX_VERSION_MINOR}"
  )
set(CPACK_PACKAGE_VERSION_PATCH
  "${O3D3XX_VERSION_PATCH}"
  )

find_program(DPKG_CMD dpkg)
if(NOT DPKG_CMD)
  if(NOT CMAKE_CROSSCOMPILING)
    message(STATUS "Can not find dpkg in your path, default to ${CMAKE_SYSTEM_PROCESSOR}.")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
  endif(NOT CMAKE_CROSSCOMPILING)
else()
  if(NOT CMAKE_CROSSCOMPILING)
    execute_process(COMMAND "${DPKG_CMD}" --print-architecture
      OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
  endif(NOT CMAKE_CROSSCOMPILING)
endif(NOT DPKG_CMD)
set(CPACK_PACKAGE_FILE_NAME
  "${O3D3XX_IMAGE_MODULE_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_SECTION Libraries)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
  "libgoogle-glog-dev,
   libo3d3xx-camera (= ${O3D3XX_VERSION_STRING}),
   libo3d3xx-framegrabber (= ${O3D3XX_VERSION_STRING}),
   libopencv-core-dev,
   libopencv-imgproc-dev")
if(BUILD_EXE_VIEWER)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "${CPACK_DEBIAN_PACKAGE_DEPENDS},
     libvtk6-dev,
     libpcl1.7,
     libopencv-highgui-dev,
     libopencv-contrib-dev")
endif(BUILD_EXE_VIEWER)
include(CPack)

set(DEB_FILE
    "${O3D3XX_IMAGE_MODULE_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}.deb")

if(CMAKE_CROSSCOMPILING)
  find_program(SCP_EXE scp)
  if(SCP_EXE)
    add_custom_target(deploy
      COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target package --config ${CMAKE_BUILD_TYPE}
      COMMAND ${SCP_EXE} ${DEB_FILE} ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR})
  else()
    message("Could not create deploy target: 'scp' not found.")
  endif(SCP_EXE)
endif(CMAKE_CROSSCOMPILING)
