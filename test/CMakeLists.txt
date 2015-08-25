add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin)
file(GLOB TEST_SOURCES *.cpp)
add_executable(o3d3xx-tests ${TEST_SOURCES})
target_link_libraries(o3d3xx-tests gtest_main o3d3xx)

#
# For each test:
# NAME = <some logical name CTest will use to report on the test>
# COMMAND = <an executable target added with `add_executable'>
#
# NOTE: We can leverage gtest's "stress tesing" like:
#
# add_test(NAME mytest COMMAND o3d3xx-tests --gtest_repeat=100000)
#
add_test(NAME o3d3xx-tests COMMAND o3d3xx-tests)
#
# This is why we are doing this:
# http://stackoverflow.com/questions/733475/cmake-ctest-make-test-doesnt-build-tests
#
add_custom_target(check COMMAND o3d3xx-tests DEPENDS o3d3xx-tests)
add_custom_target(stress COMMAND o3d3xx-tests --gtest_repeat=100
  DEPENDS o3d3xx-tests)

#
# NOTE: If you just want to run one test (or a subset of the tests):
# https://code.google.com/p/googletest/wiki/AdvancedGuide#Running_Test_Programs%3a_Advanced_Options
#
# For example, if you only wanted to run the DenseImageTest.ImageSize test you
# could:
#
# $ GTEST_FILTER=DenseImageTest.ImageSize make check
#
#