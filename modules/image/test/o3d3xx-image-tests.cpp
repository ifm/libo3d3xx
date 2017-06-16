#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include "gtest/gtest.h"
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

// utility function to compare to opencv images
bool imgs_eq(cv::Mat a, cv::Mat b)
{
  return cv::countNonZero(a != b) == 0;
}

TEST(ImageBuffers_Tests, Cloud)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

  pcl::PointCloud<o3d3xx::PointT>::Ptr cloud = img->Cloud();
  EXPECT_TRUE(cloud.use_count() == 2);

  int width = cloud->width;
  int height = cloud->height;

  // create a new scope
  {
    o3d3xx::ImageBuffer::Ptr img2 =
      o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

    EXPECT_TRUE(fg->WaitForFrame(img2.get(), 1000));

    cloud = img2->Cloud();

    EXPECT_FALSE(img->Cloud() == cloud);
    EXPECT_TRUE(cloud.use_count() == 2);
  }

  EXPECT_TRUE(cloud.use_count() == 1);
  EXPECT_TRUE(cloud->width == width);
  EXPECT_TRUE(cloud->height == height);
}

TEST(ImageBuffers_Tests, XYZImage)
{
  o3d3xx::Camera::Ptr cam =
    std::make_shared<o3d3xx::Camera>();

  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam);

  o3d3xx::ImageBuffer::Ptr buff =
    std::make_shared<o3d3xx::ImageBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));

  pcl::PointCloud<o3d3xx::PointT>::Ptr cloud = buff->Cloud();
  cv::Mat img = buff->XYZImage();

  int num_points = cloud->height * cloud->width;
  int col = 0;
  int matrix_col = 0;
  int row = -1;
  std::int16_t* row_ptr;

  for (std::size_t i = 0; i < num_points; ++i)
    {
      o3d3xx::PointT& pt = cloud->points[i];

      col = i % cloud->width;
      matrix_col = col * 3;
      if (col == 0)
        {
          row += 1;
          row_ptr = img.ptr<std::int16_t>(row);
        }

      if (std::isnan(pt.x))
        {
          ASSERT_TRUE(std::isnan(pt.y));
          ASSERT_TRUE(std::isnan(pt.z));

          ASSERT_EQ(row_ptr[matrix_col],
                    std::numeric_limits<std::int16_t>::quiet_NaN());
          ASSERT_EQ(row_ptr[matrix_col + 1],
                    std::numeric_limits<std::int16_t>::quiet_NaN());
          ASSERT_EQ(row_ptr[matrix_col + 2],
                    std::numeric_limits<std::int16_t>::quiet_NaN());
        }
      else
        {
          ASSERT_FLOAT_EQ(pt.x * 1000.0, (float) row_ptr[matrix_col]);
          ASSERT_FLOAT_EQ(pt.y * 1000.0, (float) row_ptr[matrix_col + 1]);
          ASSERT_FLOAT_EQ(pt.z * 1000.0, (float) row_ptr[matrix_col + 2]);
        }
    }
}

TEST(ImageBuffers_Tests, DepthImage)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

  cv::Mat dimg = img->DepthImage();
  cv::Mat dimg2 = img->DepthImage();
  EXPECT_TRUE(imgs_eq(dimg, dimg2));

  {
    o3d3xx::ImageBuffer::Ptr img2 =
      o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

    EXPECT_TRUE(fg->WaitForFrame(img2.get(), 1000));

    dimg2 = img2->DepthImage();
    EXPECT_FALSE(imgs_eq(dimg, dimg2));
  }

  // img2 is now out of scope, we want to check that
  // dimg2 is still valid but not the same as dimg

  EXPECT_FALSE(imgs_eq(dimg, dimg2));
  EXPECT_EQ(dimg.size().height, dimg2.size().height);
  EXPECT_EQ(dimg.size().width, dimg2.size().width);
}

TEST(ImageBuffers_Tests, AmplitudeImage)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

  cv::Mat aimg = img->AmplitudeImage();
  cv::Mat aimg2 = img->AmplitudeImage();
  EXPECT_TRUE(imgs_eq(aimg, aimg2));

  {
    o3d3xx::ImageBuffer::Ptr img2 =
      o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

    EXPECT_TRUE(fg->WaitForFrame(img2.get(), 1000));

    aimg2 = img2->AmplitudeImage();
    EXPECT_FALSE(imgs_eq(aimg, aimg2));
  }

  // make sure aimg2 is still valid even though img2 is now out-of-scope

  EXPECT_FALSE(imgs_eq(aimg, aimg2));
  EXPECT_EQ(aimg.size().height, aimg2.size().height);
  EXPECT_EQ(aimg.size().width, aimg2.size().width);
}

TEST(ImageBuffers_Tests, RawAmplitudeImage)
{
  std::uint16_t mask = o3d3xx::DEFAULT_SCHEMA_MASK;
  mask |= o3d3xx::IMG_RAMP;

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();

  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam, mask);

  o3d3xx::ImageBuffer::Ptr img =
    std::make_shared<o3d3xx::ImageBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

  cv::Mat aimg = img->RawAmplitudeImage();
  cv::Mat aimg2 = img->RawAmplitudeImage();
  EXPECT_TRUE(imgs_eq(aimg, aimg2));

  {
    o3d3xx::ImageBuffer::Ptr img2 =
      std::make_shared<o3d3xx::ImageBuffer>();

    EXPECT_TRUE(fg->WaitForFrame(img2.get(), 1000));

    aimg2 = img2->RawAmplitudeImage();
    EXPECT_FALSE(imgs_eq(aimg, aimg2));
  }

  // make sure aimg2 is still valid even though img2 is now out-of-scope

  EXPECT_FALSE(imgs_eq(aimg, aimg2));
  EXPECT_EQ(aimg.size().height, aimg2.size().height);
  EXPECT_EQ(aimg.size().width, aimg2.size().width);
}

TEST(ImageBuffers_Tests, ConfidenceImage)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

  cv::Mat cimg = img->ConfidenceImage();
  cv::Mat cimg2 = img->ConfidenceImage();
  EXPECT_TRUE(imgs_eq(cimg, cimg2));

  {
    o3d3xx::ImageBuffer::Ptr img2 =
      o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

    EXPECT_TRUE(fg->WaitForFrame(img2.get(), 1000));

    cimg2 = img2->ConfidenceImage();
    EXPECT_FALSE(imgs_eq(cimg, cimg2));
  }

  // make sure cimg2 is still valid even though img2 is now out-of-scope

  EXPECT_FALSE(imgs_eq(cimg, cimg2));
  EXPECT_EQ(cimg.size().height, cimg2.size().height);
  EXPECT_EQ(cimg.size().width, cimg2.size().width);
}

TEST(ImageBuffers_Tests, Bytes)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

  std::vector<std::uint8_t> b1 = img->Bytes();
  std::vector<std::uint8_t> b2 = img->Bytes();

  EXPECT_EQ(b1, b2);
  EXPECT_NO_THROW(b1[0]+=1);
  EXPECT_NE(b1, b2);

  std::vector<std::uint8_t> b3;
  EXPECT_EQ(0, b3.size());
  {
    o3d3xx::ImageBuffer::Ptr img2 =
      o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

    EXPECT_TRUE(fg->WaitForFrame(img2.get(), 1000));

    b3 = img2->Bytes();
  }

  EXPECT_NE(0, b3.size());
  EXPECT_NE(b1, b3);

  img->SetBytes(b3);
  b1 = img->Bytes();
  EXPECT_NE(b1, b3);

  img->SetBytes(b3, true); // set copy flag
  b1 = img->Bytes();
  EXPECT_EQ(b1, b3);
}

TEST(ImageBuffers_Tests, CopyConstructor)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  int i = 0;
  while (i < 5)
    {
      EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

      o3d3xx::ImageBuffer::Ptr img2 =
        o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer(*(img.get())));

      EXPECT_TRUE(img2->Bytes() == img->Bytes());

      i++;
    }
}

TEST(ImageBuffers_Tests, CopyAssignmentOperator)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::FrameGrabber::Ptr fg =
    o3d3xx::FrameGrabber::Ptr(new o3d3xx::FrameGrabber(cam));

  o3d3xx::ImageBuffer::Ptr img =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  o3d3xx::ImageBuffer::Ptr img2 =
    o3d3xx::ImageBuffer::Ptr(new o3d3xx::ImageBuffer());

  int i = 0;
  while (i < 5)
    {
      EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));

      *(img2.get()) = *(img.get());

      EXPECT_TRUE(img2->Bytes() == img->Bytes());

      i++;
    }
}

TEST(ImageBuffers_Tests, Extrinsics)
{
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  //
  // Helpers to generate rotation matrices
  //
  const auto& radians = [](double deg) -> double { return deg*M_PI/180.; };

  const auto& rot_x = [](float alpha) -> cv::Mat
    {
      cv::Mat r = (cv::Mat_<float>(3,3) <<
                   1., 0., 0.,
                   0., std::cos(alpha), -std::sin(alpha),
                   0., std::sin(alpha), std::cos(alpha));
      return r;
    };

  const auto& rot_y = [](float alpha) -> cv::Mat
    {
      cv::Mat r = (cv::Mat_<float>(3,3) <<
                   std::cos(alpha), 0., std::sin(alpha),
                   0., 1., 0.,
                   -std::sin(alpha), 0., std::cos(alpha));
      return r;
    };

  const auto& rot_z = [](float alpha) -> cv::Mat
    {
      cv::Mat r = (cv::Mat_<float>(3,3) <<
                   std::cos(alpha), -std::sin(alpha), 0.,
                   std::sin(alpha), std::cos(alpha), 0.,
                   0., 0., 1.);
      return r;
    };

  // We assume the XML-RPC extrinsics are: 0,0,0,0,0,0
  // so, we get a reference set of extrincs that encode the transform
  // from the PMD chip to the camera glass
  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  std::vector<float> reference_extrinsics = img->Extrinsics();

  // For clarity, we unpack the extrinsics into their scalar components
  float t_ix = reference_extrinsics.at(0);
  float t_iy = reference_extrinsics.at(1);
  float t_iz = reference_extrinsics.at(2);
  float alpha_ix = reference_extrinsics.at(3);
  float alpha_iy = reference_extrinsics.at(4);
  float alpha_iz = reference_extrinsics.at(5);

  // Now construct R_i and t_i
  cv::Mat R_i =  rot_x(radians(alpha_ix)) *
                 rot_y(radians(alpha_iy)) *
                 rot_z(radians(alpha_iz));

  cv::Mat t_i = (cv::Mat_<float>(3,1) << t_ix, t_iy, t_iz);

  // Now, we set a dummy extrinsic calibration on the camera
  fg->Stop();
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
  float t_ux = 42.1;
  float t_uy = 44.2;
  float t_uz = 46.3;
  float alpha_ux = 5.6;
  float alpha_uy = 7.8;
  float alpha_uz = 9.1;
  dev->SetExtrinsicCalibTransX(t_ux);
  dev->SetExtrinsicCalibTransY(t_uy);
  dev->SetExtrinsicCalibTransZ(t_uz);
  dev->SetExtrinsicCalibRotX(alpha_ux);
  dev->SetExtrinsicCalibRotY(alpha_uy);
  dev->SetExtrinsicCalibRotZ(alpha_uz);
  cam->SetDeviceConfig(dev.get());
  cam->SaveDevice();
  cam->CancelSession();

  // Now construct R_u and t_u
  cv::Mat R_u = rot_x(radians(alpha_ux)) *
                rot_y(radians(alpha_uy)) *
                rot_z(radians(alpha_uz));
  cv::Mat t_u = (cv::Mat_<float>(3,1) << t_ux, t_uy, t_uz);

  // Restart the frame grabber and get the new extrinsics which should
  // contain R_total and t_total.
  fg.reset(new o3d3xx::FrameGrabber(cam));
  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  std::vector<float> new_extrinsics = img->Extrinsics();

  float t_totx = new_extrinsics.at(0);
  float t_toty = new_extrinsics.at(1);
  float t_totz = new_extrinsics.at(2);
  float alpha_totx = new_extrinsics.at(3);
  float alpha_toty = new_extrinsics.at(4);
  float alpha_totz = new_extrinsics.at(5);

  cv::Mat R_total_expected = R_u * R_i;
  cv::Mat R_total_actual = rot_x(radians(alpha_totx)) *
                           rot_y(radians(alpha_toty)) *
                           rot_z(radians(alpha_totz));

  cv::Mat t_total_expected = R_u * t_i + t_u;
  cv::Mat t_total_actual = (cv::Mat_<float>(3,1) << t_totx, t_toty, t_totz);

  // Now check that our expect values agree with the actuals to w/in a
  // tolerance ... for now, 3 decimal places
  float tol = .001;
  EXPECT_TRUE(std::equal(R_total_actual.begin<float>(),
                         R_total_actual.end<float>(),
                         R_total_expected.begin<float>(),
                         [tol](float a, float b) -> bool
                         { return std::abs(a - b) <= tol; }));

  EXPECT_TRUE(std::equal(t_total_actual.begin<float>(),
                         t_total_actual.end<float>(),
                         t_total_expected.begin<float>(),
                         [tol](float a, float b) -> bool
                         { return std::abs(a - b) <= tol; }));

  // now put it all back and validate that it is equal to the orignal
  // reference extrinsics
  fg->Stop();
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  dev = cam->GetDeviceConfig();
  dev->SetExtrinsicCalibTransX(0.);
  dev->SetExtrinsicCalibTransY(0.);
  dev->SetExtrinsicCalibTransZ(0.);
  dev->SetExtrinsicCalibRotX(0.);
  dev->SetExtrinsicCalibRotY(0.);
  dev->SetExtrinsicCalibRotZ(0.);
  cam->SetDeviceConfig(dev.get());
  cam->SaveDevice();
  cam->CancelSession();

  fg.reset(new o3d3xx::FrameGrabber(cam));
  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  new_extrinsics = img->Extrinsics();

  EXPECT_FLOAT_EQ(reference_extrinsics[0], new_extrinsics[0]);
  EXPECT_FLOAT_EQ(reference_extrinsics[1], new_extrinsics[1]);
  EXPECT_FLOAT_EQ(reference_extrinsics[2], new_extrinsics[2]);
  EXPECT_FLOAT_EQ(reference_extrinsics[3], new_extrinsics[3]);
  EXPECT_FLOAT_EQ(reference_extrinsics[4], new_extrinsics[4]);
  EXPECT_FLOAT_EQ(reference_extrinsics[5], new_extrinsics[5]);
}

TEST(ImageBuffers_Tests, UnitVectorAccess)
{
  //
  // NOTE: Beyond being a unit test, this function serves as a cookbook-like
  // example for getting access to the Unit Vector data.
  //

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  // set the mask to only get unit vector data
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_UVEC);
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  cv::Mat uvec = img->UnitVectors(); // this is a reference
  cv::Mat uvec_clone = uvec.clone(); // now we have a copy

  // re-init the framegrabber with the default mask and grab a frame
  fg.reset(new o3d3xx::FrameGrabber(cam));
  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  uvec = img->UnitVectors();

  // now, pixel-by-pixel, make sure the unit vectors are the same. This tests
  // that we are not stomping on the unit vector data after switching to image
  // streaming.
  //
  // NOTE: I'm providing a predicate function in case `==` proves too naive for
  // comparing the floats (but I think it should work for this particular case
  // ... i.e., the unit vectors should always be *exactly* the same). If we
  // want to compare for equality in some different way (e.g., n-ULPs) just
  // fill in the implementation.
  //
  EXPECT_TRUE(std::equal(uvec_clone.begin<float>(), uvec_clone.end<float>(),
                         uvec.begin<float>(),
                         [](float a, float b) -> bool { return a == b; }));

}

TEST(ImageBuffers_Tests, ComputeCartesian)
{
  //
  // NOTE: The code in this function is not "optimized" for performance. In
  // addition to this serving as a unit test, it will also serve as a
  // cookbook-like example for peforming off-board Cartesian data
  // compution (hence the comments). It is meant to be unambiguously clear to
  // the reader as to what is going on w/o skipping obfusticating any
  // intermediate steps that could be easily misunderstood in an optimized
  // version. Optimize as you will for your application.
  //

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  //
  // 1. Initialize the frame grabber with a schema that only streams in the
  // unit vectors. Hold a reference to the unit vector data
  //
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_UVEC);
  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  cv::Mat uvec = img->UnitVectors();

  //
  // 2. Re-init the framegrabber to only stream in:
  //    - radial distance image (we use this to compute the cartesian data)
  //    - cartesian (we use this to compare our computation vs. ground truth)
  //
  // NOTE: If you were doing this "for real", you would not need to bring in
  // this cartesian data ... the whole point is that you are computing it
  // instead of the camera ... which will make Christian and Christian very
  // happy :-)
  //
  fg.reset(new o3d3xx::FrameGrabber(cam, o3d3xx::IMG_RDIS|o3d3xx::IMG_CART));
  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  cv::Mat rdis = img->DepthImage(); // <-- XXX: going to be renamed soon
  cv::Mat xyz_cam = img->XYZImage(); // cartesian data computed by camera

  // for convenience later, we split out the xyz spatial planes
  std::vector<cv::Mat> xyz_channels(3);
  cv::split(xyz_cam, xyz_channels);
  cv::Mat x_cam = xyz_channels[0];
  cv::Mat y_cam = xyz_channels[1];
  cv::Mat z_cam = xyz_channels[2];

  // We need the translation vector from the extrinsics as well. We note the
  // unit vectors are already rotated per the extrinsic calibration.
  std::vector<float> extrinsics = img->Extrinsics();

  //
  // 3. Compute the cartesian data
  //

  // easy access to translation vector
  float tx = extrinsics[0];
  float ty = extrinsics[1];
  float tz = extrinsics[2];

  // easy access to the individual unit vectors
  cv::Mat ex, ey, ez;
  std::vector<cv::Mat> uvec_channels(3);
  cv::split(uvec, uvec_channels);
  ex = uvec_channels[0];
  ey = uvec_channels[1];
  ez = uvec_channels[2];

  // so we can do a vectorized operation, we need to unify the types
  cv::Mat rdis_f;
  rdis.convertTo(rdis_f, CV_32FC1);

  // compute
  cv::Mat x_ = ex.mul(rdis_f) + tx;
  cv::Mat y_ = ey.mul(rdis_f) + ty;
  cv::Mat z_ = ez.mul(rdis_f) + tz;

  // cast to int16_t
  cv::Mat x_i, y_i, z_i;
  x_.convertTo(x_i, CV_16SC1);
  y_.convertTo(y_i, CV_16SC1);
  z_.convertTo(z_i, CV_16SC1);

  //
  // Explicitly set to zero any "bad pixels". Bad pixels are those that are set
  // to zero in the radial distance image.
  //
  cv::Mat mask;
  cv::Mat mask_ = rdis != 0;
  mask_.convertTo(mask, CV_16SC1);
  mask /= 255;
  x_i = x_i.mul(mask);
  y_i = y_i.mul(mask);
  z_i = z_i.mul(mask);

  //
  // 4. Transform the cartesian data to the libo3d3xx coord frame
  //
  cv::Mat x_computed = z_i;
  cv::Mat y_computed = -x_i;
  cv::Mat z_computed = -y_i;

  //
  // 5. Compare for correctness.
  //
  // Recall, the XYZImage from the ImageBuffer is in mm, so, we will consider
  // these values equal if they are within 1 cm of eachother (rounding
  // differences in OpenCV (in the cast from float to int16_t above) vs. the
  // camera is a likely source of such discrepancies).
  //
  auto cmp = [](std::int16_t a, std::int16_t b) -> bool
    {
      if (std::abs(a - b) <= 10) // testing to w/in 1 cm tolerance
        {
          return true;
        }

      std::int16_t bad_pixel = std::numeric_limits<std::int16_t>::quiet_NaN();
      if ((a == bad_pixel) || (b == bad_pixel))
        {
          return true; // ignore bad pixels
        }

      return false;
    };

  EXPECT_TRUE(std::equal(x_cam.begin<std::int16_t>(), x_cam.end<std::int16_t>(),
                         x_computed.begin<std::int16_t>(), cmp));

  EXPECT_TRUE(std::equal(y_cam.begin<std::int16_t>(), y_cam.end<std::int16_t>(),
                         y_computed.begin<std::int16_t>(), cmp));

  EXPECT_TRUE(std::equal(z_cam.begin<std::int16_t>(), z_cam.end<std::int16_t>(),
                         z_computed.begin<std::int16_t>(), cmp));
}

TEST(ImageBuffers_Tests, DISABLED_ExposureTimes)
{
  std::string json =
    R"(
        {
          "o3d3xx":
          {
            "Device":
            {
              "ActiveApplication": "1"
            },
            "Apps":
            [
              {
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "ExposureTimeList": "125;5000",
                    "ExposureTimeRatio": "40",
                    "Type":"under5m_moderate"
                }
              }
           ]
          }
        }
      )";

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  cam->FromJSON(json);

  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(
      cam, o3d3xx::DEFAULT_SCHEMA_MASK|o3d3xx::EXP_TIME);

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  std::vector<std::uint32_t> exposure_times = img->ExposureTimes();

  EXPECT_EQ(exposure_times[0], 125);
  EXPECT_EQ(exposure_times[1], 5000);
  EXPECT_EQ(exposure_times[2], 0);

  std::string json2 =
    R"(
        {
          "o3d3xx":
          {
            "Device":
            {
              "ActiveApplication": "1"
            },
            "Apps":
            [
              {
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "ExposureTimeList": "5000",
                    "Type":"under5m_low"
                }
              }
           ]
          }
        }
      )";

  fg->Stop();
  cam->FromJSON(json2);

  img.reset(new o3d3xx::ImageBuffer());
  fg.reset(new o3d3xx::FrameGrabber(
                 cam, o3d3xx::DEFAULT_SCHEMA_MASK|o3d3xx::EXP_TIME));

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  exposure_times = img->ExposureTimes();

  EXPECT_EQ(exposure_times[0], 5000);
  EXPECT_EQ(exposure_times[1], 0);
  EXPECT_EQ(exposure_times[2], 0);

  std::string json3 =
    R"(
        {
          "o3d3xx":
          {
            "Device":
            {
              "ActiveApplication": "1"
            },
            "Apps":
            [
              {
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTimeList": "100;1000;5000",
                    "Type":"under5m_high"
                }
              }
           ]
          }
        }
      )";

  fg->Stop();
  cam->FromJSON(json3);

  img.reset(new o3d3xx::ImageBuffer());
  fg.reset(new o3d3xx::FrameGrabber(
                 cam, o3d3xx::DEFAULT_SCHEMA_MASK|o3d3xx::EXP_TIME));

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  exposure_times = img->ExposureTimes();

  EXPECT_EQ(exposure_times[0], 100);
  EXPECT_EQ(exposure_times[1], 1000);
  EXPECT_EQ(exposure_times[2], 5000);

  fg->Stop();
  cam->FromJSON(json);

  img.reset(new o3d3xx::ImageBuffer());
  fg.reset(new o3d3xx::FrameGrabber(cam)); // <-- default schema, no exptimes

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  exposure_times = img->ExposureTimes();

  EXPECT_EQ(exposure_times[0], 0);
  EXPECT_EQ(exposure_times[1], 0);
  EXPECT_EQ(exposure_times[2], 0);

}

TEST(ImageBuffers_Tests, DISABLED_IlluTemp)
{
  std::string json =
    R"(
        {
          "o3d3xx":
          {
            "Device":
            {
              "ActiveApplication": "1"
            },
            "Apps":
            [
              {
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "ExposureTimeList": "125;5000",
                    "ExposureTimeRatio": "40",
                    "Type":"under5m_moderate"
                }
              }
           ]
          }
        }
      )";

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  cam->FromJSON(json);

  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(
      cam, o3d3xx::DEFAULT_SCHEMA_MASK|o3d3xx::ILLU_TEMP);

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  float illu_temp = img->IlluTemp();

  EXPECT_GT(illu_temp, 10);
  EXPECT_LT(illu_temp, 90);
}
