project(O3D3XX)
cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_MODULE_PATH
    ${O3D3XX_SOURCE_DIR}/cmake/modules
    ${CMAKE_MODULE_PATH}
    )

# force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

# how should we install?
find_program(DPKG_CMD dpkg)
if(NOT DPKG_CMD)
  set(O3D3XX_INSTALL_CMD "$(MAKE)")
else()
  set(O3D3XX_INSTALL_CMD ${O3D3XX_SOURCE_DIR}/cmake/utils/install_all_debs)
endif(NOT DPKG_CMD)

include(ExternalProject)

################################################
## o3d3xx-camera module
################################################
ExternalProject_Add(O3D3XX_CAMERA
  SOURCE_DIR ${O3D3XX_SOURCE_DIR}/modules/camera
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=/usr
  TEST_BEFORE_INSTALL 1
  TEST_COMMAND make check
  INSTALL_COMMAND sudo ${O3D3XX_INSTALL_CMD} install
  )

ExternalProject_Add_Step(O3D3XX_CAMERA
  package
  DEPENDEES test
  DEPENDERS install
  COMMAND ${CMAKE_COMMAND} -E chdir .  make package
  WORKING_DIRECTORY O3D3XX_CAMERA-prefix/src/O3D3XX_CAMERA-build
  )

################################################
## o3d3xx-framegrabber module
################################################

ExternalProject_Add(O3D3XX_FRAMEGRABBER
  DEPENDS O3D3XX_CAMERA
  SOURCE_DIR ${O3D3XX_SOURCE_DIR}/modules/framegrabber
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=/usr
  TEST_BEFORE_INSTALL 1
  TEST_COMMAND make check
  INSTALL_COMMAND sudo ${O3D3XX_INSTALL_CMD} install
  )

ExternalProject_Add_Step(O3D3XX_FRAMEGRABBER
  package
  DEPENDEES test
  DEPENDERS install
  COMMAND ${CMAKE_COMMAND} -E chdir .  make package
  WORKING_DIRECTORY O3D3XX_FRAMEGRABBER-prefix/src/O3D3XX_FRAMEGRABBER-build
  )

################################################
## o3d3xx-image module
################################################

ExternalProject_Add(O3D3XX_IMAGE
  DEPENDS O3D3XX_CAMERA O3D3XX_FRAMEGRABBER
  SOURCE_DIR ${O3D3XX_SOURCE_DIR}/modules/image
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=/usr
  TEST_BEFORE_INSTALL 1
  TEST_COMMAND make check
  INSTALL_COMMAND sudo ${O3D3XX_INSTALL_CMD} install
  )

ExternalProject_Add_Step(O3D3XX_IMAGE
  package
  DEPENDEES test
  DEPENDERS install
  COMMAND ${CMAKE_COMMAND} -E chdir .  make package
  WORKING_DIRECTORY O3D3XX_IMAGE-prefix/src/O3D3XX_IMAGE-build
  )
