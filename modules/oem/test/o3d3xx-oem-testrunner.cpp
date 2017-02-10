#include <gtest/gtest.h>
#include <o3d3xx_camera.h>

int main(int argc, char **argv)
{
  o3d3xx::Logging::Init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
