#include "o3d3xx.h"
#include <glog/logging.h>
#include "gtest/gtest.h"

int main(int argc, char **argv)
{
  o3d3xx::Logging::Init();
  // keeps the unit test output cleaner
  google::SetStderrLogging(google::FATAL);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}