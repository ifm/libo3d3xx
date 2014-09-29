#include "o3d3xx.h"
#include <cstdint>
#include <vector>
#include "gtest/gtest.h"

TEST(FrameGrabber_Tests, Ctor)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));
}

TEST(FrameGrabber_Tests, Run)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  // grab 10 frames ... waiting no more than .5 sec for a frame
  int i = 0;
  std::vector<std::uint8_t> buff;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff, 500));
      i++;
    }

  EXPECT_EQ(i, 10);

  fg->Stop();
  EXPECT_FALSE(fg->WaitForFrame(buff, 500));
}

//
// XXX: Put in some performance unit tests
//
