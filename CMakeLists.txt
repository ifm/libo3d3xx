project(O3D3XX)
cmake_minimum_required(VERSION 2.8.11)
set(GIT_PROJECT_NAME "libo3d3xx")
set(CMAKE_INSTALL_PREFIX /opt/${GIT_PROJECT_NAME})
set(GTEST_CMAKE_DIR "/usr/src/gtest")

# force an out-of-source build of the code
set(CMAKE_MODULE_PATH
    ${O3D3XX_SOURCE_DIR}/cmake/modules
    ${CMAKE_MODULE_PATH}
    )
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build("Please build ${PROJECT_NAME} out-of-source")

# settings for controlling debug vs. release builds
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

# if you want to customize the compiler flags based on build type, do that here
set(CMAKE_CXX_FLAGS "-std=c++11") # turn on C++11 for g++
# CMAKE_CXX_FLAGS_RELEASE
# CMAKE_CXX_FLAGS_DEBUG

# This is the single place to manage API version numbers
set(O3D3XX_VERSION_MAJOR 0)
set(O3D3XX_VERSION_MINOR 1)
set(O3D3XX_VERSION_PATCH 7)
file(COPY ${O3D3XX_SOURCE_DIR}/include
  DESTINATION ${O3D3XX_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_BINARY_DIR}/include/o3d3xx/version.h.in
  ${O3D3XX_BINARY_DIR}/include/o3d3xx/version.h
  @ONLY
  )

# bring dependent projects
find_package(PCL 1.7.1 REQUIRED COMPONENTS common visualization filters)
find_package(OpenCV REQUIRED)

# Manage compiler/linker flags
include_directories(
  ${O3D3XX_BINARY_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_BINARY_DIR}/lib
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  )
add_definitions(
  ${PCL_DEFINITIONS}
  ${OpenCV_DEFINITIONS}
  )

# turn on ctest (unit testing gives us `make test' in top-level)
enable_testing()

# process child CMakeLists.txt files
add_subdirectory(src)
add_subdirectory(test)
install(
  EXPORT o3d3xx-targets DESTINATION lib/o3d3xx
  )

# `make install`
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
# uncomment if you have documentation in `docs` to install
# install(DIRECTORY ${O3D3XX_SOURCE_DIR}/doc
#   DESTINATION ${CMAKE_INSTALL_PREFIX}
#   PATTERN ".git" EXCLUDE
#   )

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
hardware to the state-of-the-art open source computer vision packages: OpenCV
and PCL.")
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
  message(STATUS "Can not find dpkg in your path, default to ${CMAKE_SYSTEM_PROCESSOR}.")
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
else()
  execute_process(COMMAND "${DPKG_CMD}" --print-architecture
    OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif(NOT DPKG_CMD)
set(CPACK_PACKAGE_FILE_NAME
  "${GIT_PROJECT_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_SECTION Libraries)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
  "libboost-dev (>= 1.54),
   libgoogle-glog-dev,
   libxmlrpc-c++8,
   libxmlrpc-c++8-dev,
   libopencv-core2.4,
   libopencv-core-dev,
   libpcl-1.7-all,
   libpcl-1.7-all-dev")
include(CPack)
