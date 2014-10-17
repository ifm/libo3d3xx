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

TEST(FrameGrabber_Tests, WaitForFrame)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  int i = 0;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
      i++;
    }

  EXPECT_EQ(i, 10);
  fg->Stop();
  EXPECT_FALSE(fg->WaitForFrame(img.get(), 500));
}

//
// XXX: Put in some performance unit tests
//
