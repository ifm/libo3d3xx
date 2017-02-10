set(GTEST_CMAKE_DIR "${SYSROOT_CONTRIB}/usr/src/gtest")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${SYSROOT_CONTRIB}/usr/include")

enable_testing()
add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin)
file(GLOB TEST_SOURCES *.cpp)
add_executable(o3d3xx-oem-tests ${TEST_SOURCES})
target_link_libraries(o3d3xx-oem-tests
  gtest_main o3d3xx_oem_shared
  ${O3D3XX_FRAMEGRABBER_LIBRARIES}
  ${O3D3XX_CAMERA_LIBRARIES}
  ${O3D3XX_IMAGE_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install(TARGETS o3d3xx-oem-tests DESTINATION bin)
