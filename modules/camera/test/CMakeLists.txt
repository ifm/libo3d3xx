set(GTEST_CMAKE_DIR "/usr/src/gtest")
enable_testing()
add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin EXCLUDE_FROM_ALL)
file(GLOB CAMERA_TEST_SOURCES *.cpp)
add_executable(o3d3xx-camera-tests ${CAMERA_TEST_SOURCES})
if(BUILD_SHARED_LIBS)
  target_link_libraries(o3d3xx-camera-tests gtest_main o3d3xx_camera_shared)
else()
  target_link_libraries(o3d3xx-camera-tests gtest_main o3d3xx_camera_static)
endif(BUILD_SHARED_LIBS)

if(CMAKE_CROSSCOMPILING)
  set_target_properties(o3d3xx-camera-tests PROPERTIES
                        COMPILE_FLAGS "${CROSSTOOL_COMPILE_FLAGS}"
                        LINK_FLAGS "${CROSSTOOL_EXE_LINKER_FLAGS}")
endif(CMAKE_CROSSCOMPILING)

################################################
## If we are NOT cross-compiling, we create
## targets to run the tests. If we are cross-
## compiling, we install the built test suite
## so it can be run on the target machine.
################################################
if(NOT CMAKE_CROSSCOMPILING)
  #
  # For each test:
  # NAME = <some logical name CTest will use to report on the test>
  # COMMAND = <an executable target added with `add_executable'>
  #
  # NOTE: We can leverage gtest's "stress tesing" like:
  #
  # add_test(NAME mytest COMMAND o3d3xx-camera-tests --gtest_repeat=100000)
  #
  add_test(NAME o3d3xx-camera-tests COMMAND o3d3xx-camera-tests)
  add_custom_target(check
                    COMMAND o3d3xx-camera-tests
                    DEPENDS o3d3xx-camera-tests)
  add_custom_target(stress
                    COMMAND o3d3xx-camera-tests --gtest_repeat=100
                    DEPENDS o3d3xx-camera-tests)

  #
  # NOTE: If you just want to run one test (or a subset of the tests) you can,
  # for example:
  #
  # $ GTEST_FILTER=DenseImageTest.ImageSize make check
  #
else()
  install(TARGETS o3d3xx-camera-tests DESTINATION test)
  install(FILES
          ${O3D3XX_CAMERA_BINARY_DIR}/test/gtest_bin/libgtest.so
          ${O3D3XX_CAMERA_BINARY_DIR}/test/gtest_bin/libgtest_main.so
          DESTINATION test/gtest_bin)

  install(FILES
          ${O3D3XX_CAMERA_SOURCE_DIR}/test/env.sh
          DESTINATION test
          PERMISSIONS OWNER_EXECUTE
                      OWNER_WRITE
                      OWNER_READ
                      GROUP_EXECUTE
                      GROUP_READ
                      WORLD_EXECUTE
                      WORLD_READ)

endif(NOT CMAKE_CROSSCOMPILING)
