project(O3D3XX_OEM CXX)
cmake_minimum_required(VERSION 2.8.12)
set(O3D3XX_OEM_MODULE_NAME "libo3d3xx-oem")

if(NOT CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR
          "This build assumes you are cross-compiling for the camera")
endif()

set(CPACK_SET_DESTDIR true)
set(CMAKE_INSTALL_PREFIX "${CMAKE_SYSROOT}/usr"
    CACHE STRING "CMake install prefix")
set(CPACK_INSTALL_PREFIX "/usr" CACHE STRING "CPack install prefix")
set(O3D3XX_OEM_ETC_DIR "/opt/oem/etc/o3d3xx_oem"
    CACHE STRING "Config file directory")

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXE_JITTER "Build o3d3xx-oem-jitter" ON)
option(INSTALL_HEADERS "Add headers to install package" OFF)

# For deploying to the camera
set(TARGET_IP "192.168.0.69" CACHE STRING "IP address of the camera")
set(TARGET_USER "oem" CACHE STRING "User account on the camera")
set(TARGET_DIR "/tmp" CACHE STRING "Remote working directory on camera")
set(SSH_KEY "~/.ssh/id_rsa.oem"
    CACHE FILEPATH "SSH private key for user account on camera")

################################################
################################################
## Most users should have no concern about what
## is beyond this line. Edit at your own risk.
################################################
################################################

set(CMAKE_MODULE_PATH
    ${O3D3XX_OEM_SOURCE_DIR}/../../cmake/modules
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
  ${O3D3XX_OEM_SOURCE_DIR}/o3d3xx_oem-config-version.cmake.in
  DESTINATION ${O3D3XX_OEM_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_OEM_BINARY_DIR}/o3d3xx_oem-config-version.cmake.in
  ${O3D3XX_OEM_BINARY_DIR}/o3d3xx_oem-config-version.cmake
  @ONLY
  )

################################################
## Bring in dependent projects
################################################
find_package(Threads REQUIRED)
find_package(o3d3xx_camera REQUIRED)
find_package(o3d3xx_framegrabber REQUIRED)
find_package(o3d3xx_image REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
#
# NOTE: As of this writing, the file:
#
# <rootfs>/usr/share/OpenCV/OpenCVModules.cmake in the Poky SDK
# is broken. Going to explicitly link to the OpenCV
# components we need in the child CMakeLists.txt file
# ... this is to avoid asking users to create a patch
# to OpenCVModules.cmake prior to building.
#
#find_package(OpenCV 3 REQUIRED)

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_BUILD_TYPE Release) # Release or Debug
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++14, ON -> -std=gnu++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-literal-suffix -s")

file(COPY ${O3D3XX_OEM_SOURCE_DIR}/include
  DESTINATION ${O3D3XX_OEM_BINARY_DIR}
  )

include_directories(
  ${O3D3XX_OEM_BINARY_DIR}/include
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${O3D3XX_IMAGE_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_OEM_BINARY_DIR}/lib
  ${O3D3XX_CAMERA_LIBRARY_DIR}
  ${O3D3XX_FRAMEGRABBER_LIBRARY_DIR}
  ${O3D3XX_IMAGE_LIBRARY_DIR}
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
if(INSTALL_HEADERS)
  install(DIRECTORY ${O3D3XX_OEM_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()

install(FILES
  ${O3D3XX_OEM_BINARY_DIR}/o3d3xx_oem-config-version.cmake
  DESTINATION lib/o3d3xx_oem
  )

file(COPY ${O3D3XX_OEM_SOURCE_DIR}/etc
  DESTINATION ${O3D3XX_OEM_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_OEM_BINARY_DIR}/etc/setup.bash.in
  ${O3D3XX_OEM_BINARY_DIR}/etc/setup.bash
  @ONLY
  )
install(DIRECTORY ${O3D3XX_OEM_BINARY_DIR}/etc/
  DESTINATION ${O3D3XX_OEM_ETC_DIR}
  PATTERN "*.in" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "*~" EXCLUDE
  )

################################################
## Manage packaging
################################################
set(CPACK_PACKAGE_NAME ${O3D3XX_OEM_MODULE_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OEM-extensions to libo3d3xx")
set(CPACK_PACKAGE_DESCRIPTION
  "Optimizations for oem algorithm developers using libo3d3xx")
set(CPACK_PACKAGE_VENDOR "Love Park Robotics, LLC")
set(CPACK_PACKAGE_CONTACT "Tom Panzarella <tom@loveparkrobotics.com>")
set(CPACK_PACKAGE_VERSION_MAJOR ${O3D3XX_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${O3D3XX_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${O3D3XX_VERSION_PATCH})
set(CPACK_PACKAGE_FILE_NAME
  "${O3D3XX_OEM_MODULE_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)

################################################
## Manage deployment process
################################################
set(TGZ_FILE
  "${O3D3XX_OEM_MODULE_NAME}_${O3D3XX_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}.tar.gz")
find_program(SCP_EXE scp)
find_program(SSH_EXE ssh)

file(COPY ${O3D3XX_OEM_SOURCE_DIR}/cmake/utils/oem-install.sh.in
  DESTINATION ${O3D3XX_OEM_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_OEM_BINARY_DIR}/oem-install.sh.in
  ${O3D3XX_OEM_BINARY_DIR}/oem-install.sh
  @ONLY
  )

file(COPY ${O3D3XX_OEM_SOURCE_DIR}/cmake/utils/oem-uninstall.sh.in
  DESTINATION ${O3D3XX_OEM_BINARY_DIR}
  )
configure_file(
  ${O3D3XX_OEM_BINARY_DIR}/oem-uninstall.sh.in
  ${O3D3XX_OEM_BINARY_DIR}/oem-uninstall.sh
  @ONLY
  )

if(SCP_EXE AND SSH_EXE)
  add_custom_target(deploy
    COMMAND ${SCP_EXE} -i ${SSH_KEY}
    ${CMAKE_BINARY_DIR}/${TGZ_FILE}
    ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}

    COMMAND ${SCP_EXE} -i ${SSH_KEY}
    ${CMAKE_BINARY_DIR}/install_manifest.txt
    ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}

    COMMAND cat ${CMAKE_BINARY_DIR}/oem-install.sh |
    ${SSH_EXE} -i ${SSH_KEY} ${TARGET_USER}@${TARGET_IP}
    )

  add_custom_target(uninstall
    COMMAND cat ${CMAKE_BINARY_DIR}/oem-uninstall.sh |
    ${SSH_EXE} -i ${SSH_KEY} ${TARGET_USER}@${TARGET_IP}
    )
endif()
