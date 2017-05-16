set(GTEST_CMAKE_DIR "/usr/src/gtest")
enable_testing()
add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin)
file(GLOB PCICCLIENT_TEST_SOURCES *.cpp)
add_executable(o3d3xx-pcicclient-tests
               ${PCICCLIENT_TEST_SOURCES})
if(BUILD_SHARED_LIBS)
  target_link_libraries(o3d3xx-pcicclient-tests
                        gtest_main o3d3xx_pcicclient_shared
                        ${O3D3XX_CAMERA_LIBRARIES}
                        )
else()
  target_link_libraries(o3d3xx-pcicclient-tests
                        gtest_main o3d3xx_pcicclient_static
                        ${O3D3XX_CAMERA_LIBRARIES}
                        )
endif(BUILD_SHARED_LIBS)

if(CMAKE_CROSSCOMPILING)
  set_target_properties(o3d3xx-pcicclient-tests PROPERTIES
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
  add_test(NAME o3d3xx-pcicclient-tests COMMAND o3d3xx-pcicclient-tests)
  add_custom_target(check
                    COMMAND o3d3xx-pcicclient-tests
                    DEPENDS o3d3xx-pcicclient-tests)
  add_custom_target(stress
                    COMMAND o3d3xx-pcicclient-tests --gtest_repeat=100
                    DEPENDS o3d3xx-pcicclient-tests)
else()
  install(TARGETS o3d3xx-pcicclient-tests DESTINATION test)
  #install(FILES
  #        ${O3D3XX_PCICCLIENT_BINARY_DIR}/test/gtest_bin/libgtest.so
  #        ${O3D3XX_PCICCLIENT_BINARY_DIR}/test/gtest_bin/libgtest_main.so
  #        DESTINATION test/gtest_bin)

endif(NOT CMAKE_CROSSCOMPILING)
