#include "o3d3xx.h"
#include <cstdint>
#include <vector>
#include "gtest/gtest.h"
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

  // grab 10 frames ... waiting no more than .5 sec for a frame
  int i = 0;
  std::vector<std::uint8_t> buff;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff, 1000));
      i++;
    }

  EXPECT_EQ(i, 10);

  fg->Stop();
  EXPECT_FALSE(fg->WaitForFrame(buff, 500));
}

TEST(FrameGrabber_Tests, WaitForCloud)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  int i = 0;
  pcl::PointCloud<o3d3xx::PointT>::Ptr
    cloud(new pcl::PointCloud<o3d3xx::PointT>());

  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForCloud(cloud, 1000));
      i++;
    }

  EXPECT_EQ(i, 10);

  fg->Stop();
  EXPECT_FALSE(fg->WaitForCloud(cloud, 500));
}

TEST(FrameGrabber_Tests, WaitForDepthImage)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  int i = 0;
  cv::Mat img;

  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForDepthImage(img, 1000));
      i++;
    }

  EXPECT_EQ(i, 10);

  fg->Stop();
  EXPECT_FALSE(fg->WaitForDepthImage(img, 500));
}

TEST(FrameGrabber_Tests, _WaitForFrame)
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
      EXPECT_TRUE(fg->_WaitForFrame(img, 1000));
      i++;
    }

  EXPECT_EQ(i, 10);
  fg->Stop();
  EXPECT_FALSE(fg->_WaitForFrame(img, 500));
}

//
// XXX: Put in some performance unit tests
//
