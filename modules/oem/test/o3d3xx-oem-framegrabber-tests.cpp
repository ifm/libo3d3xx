#include <algorithm>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <o3d3xx_camera.h>
#include <o3d3xx_framegrabber.h>
#include <o3d3xx_image.h>
#include <o3d3xx_oem.h>

TEST(OEM, BasicFramegrabbing)
{
  auto cam = std::make_shared<o3d3xx::Camera>();
  auto fg = std::make_shared<o3d3xx::oem::FrameGrabber>(cam);
  auto im = std::make_shared<o3d3xx::oem::ImageBuffer>();

  for (int i = 0; i < 10; ++i)
    {
      EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));
    }
}

//
// Helper for `PixelValidity` test
//
template<typename T>
bool pixel_cmp(const cv::Mat& arr_tcp, const cv::Mat& arr_zc)
{
  if ((arr_tcp.cols != arr_zc.cols) || (arr_tcp.rows != arr_zc.rows))
    {
      return false;
    }

  return std::equal(arr_tcp.begin<T>(), arr_tcp.end<T>(), arr_zc.begin<T>());
}

//
// Exploit async software triggering to test the pixel data acquired by
// the zero-copy framegrabber against the pixel data acquired by the TCP
// framegrabber (used as ground truth)
//
TEST(OEM, PixelValidity)
{
  // get all the data
  std::uint16_t mask = 0xFFFF;

  // a single camera can be used for both the tcp fg and zero-copy fg
  auto cam = std::make_shared<o3d3xx::Camera>();

  // set active application to s/w trigger -- we assume there is an application
  // at index=1
  std::string json =
    R"({"o3d3xx":{"Device":{"ActiveApplication": "1"},
                  "Apps":[{"TriggerMode": "2","Index": "1"}]}})";
  ASSERT_NO_THROW(cam->FromJSON(json));

  // create tcp structures
  auto fg_tcp = std::make_shared<o3d3xx::FrameGrabber>(cam, mask);
  auto im_tcp = std::make_shared<o3d3xx::ImageBuffer>();

  // create zero-copy structures
  auto fg_zc = std::make_shared<o3d3xx::oem::FrameGrabber>(cam, mask);
  auto im_zc = std::make_shared<o3d3xx::oem::ImageBuffer>();

  // before we do any triggering, make sure both framegrabbers timeout at least
  // once, effectively draining the frame-pool so that on the next trigger, both
  // frame-grabbers will receive the same image data
  while (fg_tcp->WaitForFrame(im_tcp.get(), 500)) { }
  while (fg_zc->WaitForFrame(im_zc.get(), 500)) { }

  // launch a thread for each of the tcp and zero-copy framegrabbers
  // to wait for new data
  auto fut_tcp = std::async(std::launch::async,
                            [fg_tcp,im_tcp]()->bool
                            {return fg_tcp->WaitForFrame(im_tcp.get(), 5000);});
  auto fut_zc = std::async(std::launch::async,
                           [fg_zc,im_zc]()->bool
                           {return fg_zc->WaitForFrame(im_zc.get(), 5000);});

  // send a s/w trigger using the tcp framegrabber -- both
  // subscribed clients should see the data
  fg_tcp->SWTrigger();

  //
  // Compare the results -- pixel-by-pixel, they should be identical
  //

  // Did they both get a frame?
  ASSERT_TRUE(fut_tcp.get());
  ASSERT_TRUE(fut_zc.get());

  // compare the OpenCV arrays
  EXPECT_TRUE(pixel_cmp<float>(im_tcp->UnitVectors(), im_zc->UnitVectors()));
  EXPECT_TRUE(pixel_cmp<std::uint16_t>(im_tcp->DepthImage(),
                                       im_zc->DepthImage()));
  EXPECT_TRUE(pixel_cmp<std::uint16_t>(im_tcp->AmplitudeImage(),
                                       im_zc->AmplitudeImage()));
  EXPECT_TRUE(pixel_cmp<std::uint16_t>(im_tcp->RawAmplitudeImage(),
                                       im_zc->RawAmplitudeImage()));
  EXPECT_TRUE(pixel_cmp<std::uint8_t>(im_tcp->ConfidenceImage(),
                                      im_zc->ConfidenceImage()));

  // compare the cartesian array discretely by spatial image plane
  std::vector<cv::Mat> xyz_tcp(3);
  std::vector<cv::Mat> xyz_zc(3);
  cv::split(im_tcp->XYZImage(), xyz_tcp);
  cv::split(im_zc->XYZImage(), xyz_zc);
  EXPECT_TRUE(pixel_cmp<std::int16_t>(xyz_tcp[0], xyz_zc[0])); // x
  EXPECT_TRUE(pixel_cmp<std::int16_t>(xyz_tcp[1], xyz_zc[1])); // y
  EXPECT_TRUE(pixel_cmp<std::int16_t>(xyz_tcp[2], xyz_zc[2])); // z

  // compare the pcl point clouds
  pcl::PointCloud<o3d3xx::PointT>::Ptr tcp_cloud = im_tcp->Cloud();
  pcl::PointCloud<o3d3xx::oem::PointT>::Ptr zc_cloud = im_zc->Cloud();
  ASSERT_EQ(tcp_cloud->width, zc_cloud->width);
  ASSERT_EQ(tcp_cloud->height, zc_cloud->height);
  std::size_t n_pts = tcp_cloud->points.size();
  ASSERT_EQ(n_pts, zc_cloud->points.size());
  for (std::size_t i = 0; i < n_pts; ++i)
    {
      EXPECT_FLOAT_EQ(tcp_cloud->points[i].x, zc_cloud->points[i].x);
      EXPECT_FLOAT_EQ(tcp_cloud->points[i].y, zc_cloud->points[i].y);
      EXPECT_FLOAT_EQ(tcp_cloud->points[i].z, zc_cloud->points[i].z);
      EXPECT_FLOAT_EQ(tcp_cloud->points[i].intensity,
                      zc_cloud->points[i].intensity);
    }

  // compare the extrinsics
  const std::vector<float>& tcp_extrinsics = im_tcp->Extrinsics();
  const std::vector<float>& zc_extrinsics = im_zc->Extrinsics();
  EXPECT_TRUE(std::equal(tcp_extrinsics.begin(), tcp_extrinsics.end(),
                         zc_extrinsics.begin()));

  //
  // XXX: Uncomment once we have the means to acquire exposure times
  //      via resultsync. Working with ifm on this. For now, let's
  //      emit a reminder to do that
  std::cerr << "Acquiring exposure times from resultsync not imlemented!"
            << std::endl;
  //
  // const std::vector<std::uint32_t>& tcp_exptime = im_tcp->ExposureTimes();
  // const std::vector<std::uint32_t>& zc_exptime = im_zc->ExposureTimes();
  // EXPECT_TRUE(std::equal(tcp_exptime.begin(), tcp_exptime.end(),
  //                        zc_exptime.begin()));

  // reset active application back to free-run
  json =
    R"({"o3d3xx":{"Device":{"ActiveApplication": "1"},
                  "Apps":[{"TriggerMode": "1","Index": "1"}]}})";
  ASSERT_NO_THROW(cam->FromJSON(json));
}
