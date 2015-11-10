project(O3D3XX)
cmake_minimum_required(VERSION 2.8.11)
set(GIT_PROJECT_NAME "libo3d3xx")

################################################
##
## User-tunable build parameters
## -----------------------------
##
## Before editing this file directly, realize
## that you can set these on the command line.
## E.g.,
##
## $ cmake -DBUILD_EXE_VIEWER=OFF ..
## $ make
##
################################################
set(CMAKE_INSTALL_PREFIX /opt/${GIT_PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release) # Release or Debug

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_STATIC_LIBS "Build static libraries" ON)

# Fine granularity over which cmd line tools get built
if(NOT CMAKE_CROSSCOMPILING)
  option(BUILD_EXE_VIEWER "Build o3d3xx-viewer" ON)
else()
  option(BUILD_EXE_VIEWER "Build o3d3xx-viewer" OFF)
endif(NOT CMAKE_CROSSCOMPILING)
option(BUILD_EXE_VERSION "Build o3d3xx-version" ON)
option(BUILD_EXE_RESET "Build o3d3xx-reset" ON)
option(BUILD_EXE_LS "Build o3d3xx-ls" ON)
option(BUILD_EXE_DUMP "Build o3d3xx-dump" ON)
option(BUILD_EXE_CONFIG "Build o3d3xx-config" ON)
option(BUILD_EXE_RM "Build o3d3xx-config" ON)
option(BUILD_EXE_REBOOT "Build o3d3xx-reboot" ON)
option(BUILD_EXE_HZ "Build o3d3xx-hz" ON)
option(BUILD_EXE_IMAGER_TYPES "Build o3d3xx-imager-types" ON)
option(BUILD_EXE_IFM_IMPORT "Build o3d3xx-ifm-import" ON)
option(BUILD_EXE_IFM_EXPORT "Build o3d3xx-ifm-export" ON)

# If you are cross-compiling, the scp-deploy target
# below (see bottom of this file) uses these
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
    ${O3D3XX_SOURCE_DIR}/cmake/modules
    ${CMAKE_MODULE_PATH}
    )

# force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

################################################
## This is the single place to manage API
## version numbers
################################################
set(O3D3XX_VERSION_MAJOR 0)
set(O3D3XX_VERSION_MINOR 2)
set(O3D3XX_VERSION_PATCH 0)
file(COPY ${O3D3XX_SOURCE_DIR}/include
  DESTINATION ${O3D3XX_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_BINARY_DIR}/include/o3d3xx/version.h.in
  ${O3D3XX_BINARY_DIR}/include/o3d3xx/version.h
  @ONLY
  )

################################################
## Bring in dependent projects
################################################
find_package(Threads REQUIRED)
find_package(XMLRPC REQUIRED c++ client)
find_package(Boost REQUIRED COMPONENTS
             program_options system)
find_package(PCL 1.7.1 REQUIRED COMPONENTS common)
if (BUILD_EXE_VIEWER)
  find_package(PCL 1.7.1 REQUIRED COMPONENTS visualization)
endif(BUILD_EXE_VIEWER)
find_package(OpenCV REQUIRED)

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${O3D3XX_BINARY_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${XMLRPC_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_BINARY_DIR}/lib
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
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
    EXPORT o3d3xx-targets DESTINATION lib/o3d3xx)
endif(BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS)

file(COPY ${O3D3XX_SOURCE_DIR}/etc
  DESTINATION ${O3D3XX_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_BINARY_DIR}/etc/setup.bash.in
  ${O3D3XX_BINARY_DIR}/etc/setup.bash
  @ONLY
  )
install(DIRECTORY ${O3D3XX_BINARY_DIR}/etc
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  PATTERN "*.in" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "*~" EXCLUDE
  )
install(DIRECTORY ${O3D3XX_BINARY_DIR}/include
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  PATTERN "*.in" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "*~" EXCLUDE
  )

# `make package`
configure_file(
  ${O3D3XX_SOURCE_DIR}/debian/conffiles.in
  ${O3D3XX_BINARY_DIR}/debian/conffiles
  @ONLY
  )
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
  "${O3D3XX_BINARY_DIR}/debian/conffiles"
  )
set(CPACK_SET_DESTDIR "on")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${GIT_PROJECT_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Driver for IFM Efector O3D3xx Cameras")
set(CPACK_PACKAGE_DESCRIPTION
  "Driver and utilities for usage with o3d3xx cameras built and sold
by IFM Efector. The o3d3xx cameras are 3D cameras based on the PMD
Tech Photonic Mixer Device time-of-flight imager. This toolbox bridges the IFM
hardware to the open source computer vision packages: OpenCV and PCL.")
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
set(O3D3XX_VERSION_STRING
    "${O3D3XX_VERSION_MAJOR}.${O3D3XX_VERSION_MINOR}.${O3D3XX_VERSION_PATCH}")

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
  "${GIT_PROJECT_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_SECTION Libraries)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
  "libgoogle-glog0,
   libboost-system1.54.0,
   libboost-program-options1.54.0,
   libxmlrpc-c++8,
   libopencv-core2.4,
   libopencv-imgproc2.4,
   libstdc++6,
   libc6,
   libgcc1")
if(BUILD_EXE_VIEWER)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "${CPACK_DEBIAN_PACKAGE_DEPENDS},
     libvtk5.8,
     libpcl-visualization-1.7,
     libpcl-common-1.7,
     libopencv-highgui2.4,
     libopencv-contrib2.4")
endif(BUILD_EXE_VIEWER)
include(CPack)

set(DEB_FILE
    "${GIT_PROJECT_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}.deb")

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
