project(O3D3XX_CAMERA)
cmake_minimum_required(VERSION 2.8.12)
set(O3D3XX_CAMERA_MODULE_NAME "libo3d3xx-camera")

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
option(BUILD_EXE_VERSION "Build o3d3xx-version" ON)
option(BUILD_EXE_RESET "Build o3d3xx-reset" ON)
option(BUILD_EXE_LS "Build o3d3xx-ls" ON)
option(BUILD_EXE_DUMP "Build o3d3xx-dump" ON)
option(BUILD_EXE_CONFIG "Build o3d3xx-config" ON)
option(BUILD_EXE_RM "Build o3d3xx-rm" ON)
option(BUILD_EXE_REBOOT "Build o3d3xx-reboot" ON)
option(BUILD_EXE_IMAGER_TYPES "Build o3d3xx-imager-types" ON)
option(BUILD_EXE_IFM_IMPORT "Build o3d3xx-ifm-import" ON)
option(BUILD_EXE_IFM_EXPORT "Build o3d3xx-ifm-export" ON)

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
    ${O3D3XX_CAMERA_SOURCE_DIR}/../../cmake/modules
    ${CMAKE_MODULE_PATH}
    )

# force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

# library version
include(o3d3xx_version)

################################################
## Variable-interpolate the version number
################################################
file(COPY ${O3D3XX_CAMERA_SOURCE_DIR}/include
  DESTINATION ${O3D3XX_CAMERA_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_CAMERA_BINARY_DIR}/include/o3d3xx_camera/version.h.in
  ${O3D3XX_CAMERA_BINARY_DIR}/include/o3d3xx_camera/version.h
  @ONLY
  )

file(COPY
  ${O3D3XX_CAMERA_SOURCE_DIR}/o3d3xx_camera-config-version.cmake.in
  DESTINATION ${O3D3XX_CAMERA_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_CAMERA_BINARY_DIR}/o3d3xx_camera-config-version.cmake.in
  ${O3D3XX_CAMERA_BINARY_DIR}/o3d3xx_camera-config-version.cmake
  @ONLY
  )

include(ubuntu_version)
get_ubuntu_version(${UBUNTU_VERSION})

################################################
## Bring in dependent projects
################################################
find_package(XMLRPC REQUIRED c++ client)
find_package(Boost REQUIRED COMPONENTS program_options system)

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${O3D3XX_CAMERA_BINARY_DIR}/include
  ${XMLRPC_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_CAMERA_BINARY_DIR}/lib
  ${Boost_LIBRARY_DIRS}
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
    EXPORT o3d3xx_camera-targets DESTINATION lib/o3d3xx_camera)
endif(BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS)

install(DIRECTORY ${O3D3XX_CAMERA_BINARY_DIR}/include
  DESTINATION .
  PATTERN "*.in" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "*~" EXCLUDE
  )

install(FILES
  ${O3D3XX_CAMERA_BINARY_DIR}/o3d3xx_camera-config-version.cmake
  DESTINATION lib/o3d3xx_camera
  )

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${O3D3XX_CAMERA_MODULE_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "IFM Efector O3D3xx Camera Interface")
set(CPACK_PACKAGE_DESCRIPTION
  "Driver and utilities for usage with o3d3xx cameras built and sold
by IFM Efector. The o3d3xx cameras are 3D cameras based on the PMD
Tech Photonic Mixer Device time-of-flight imager. This package implements the
XML-RPC protocol used by the camera to configure its operating parameters.")
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
string(CONCAT CPACK_PACKAGE_FILE_NAME
       "${O3D3XX_CAMERA_MODULE_NAME}" "_"
       "${O3D3XX_VERSION_STRING}" "_"
       "ubuntu-${UBUNTU_VERSION}" "_"
       "${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_SECTION Libraries)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
  "libgoogle-glog-dev,
   libboost-dev (>= 1.54),
   libxmlrpc-c++8-dev (>= 1.33)")
include(CPack)

string(CONCAT DEB_FILE
       "${O3D3XX_CAMERA_MODULE_NAME}" "_"
       "${O3D3XX_VERSION_STRING}" "_"
       "ubuntu-${UBUNTU_VERSION}" "_"
       "${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}" ".deb")
MESSAGE(STATUS "DEB_FILE: " ${DEB_FILE})

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
