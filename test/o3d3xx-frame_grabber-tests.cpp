#include "o3d3xx.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "gtest/gtest.h"

class FrameGrabberTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    cam_ = std::make_shared<o3d3xx::Camera>();
    cam_->RequestSession();
    cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

    dev_ = cam_->GetDeviceConfig();
    old_active_idx_ = dev_->ActiveApplication();

    idx_ = cam_->CopyApplication(old_active_idx_);
    dev_->SetActiveApplication(idx_);
    cam_->SetDeviceConfig(dev_.get());
    cam_->SaveDevice();
    cam_->CancelSession();
  }

  virtual void TearDown()
  {
    cam_->RequestSession();
    cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
    dev_->SetActiveApplication(old_active_idx_);
    cam_->SetDeviceConfig(dev_.get());
    cam_->SaveDevice();
    cam_->DeleteApplication(idx_);
  }

  o3d3xx::Camera::Ptr cam_;
  o3d3xx::DeviceConfig::Ptr dev_;
  int idx_ = -1;
  int old_active_idx_ = -1;
};

TEST_F(FrameGrabberTest, WaitForFrame)
{
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

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

TEST_F(FrameGrabberTest, WaitForFrameFast)
{
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  int i = 0;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000, false, false));
      i++;
    }

  EXPECT_EQ(i, 10);
  fg->Stop();
  EXPECT_FALSE(fg->WaitForFrame(img.get(), 500));
}

TEST_F(FrameGrabberTest, ResultSchema)
{
  //------------------------------------------
  // Set a non-default schema on to the camera
  //------------------------------------------

  std::string schema =
  "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"ascii\" }, \
  \"elements\": [ { \"type\": \"string\", \"value\": \"star\", \"id\": \
  \"start_string\" }, { \"type\": \"blob\", \"id\": \
  \"normalized_amplitude_image\" }, { \
  \"type\": \"blob\", \"id\": \"diagnostic_data\" }, { \"type\": \"string\", \
  \"value\": \"stop\", \"id\": \"end_string\" } ] }";

  cam_->RequestSession();
  cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  cam_->EditApplication(idx_);
  o3d3xx::AppConfig::Ptr app = cam_->GetAppConfig();
  app->SetPcicTcpResultSchema(schema);
  cam_->SetAppConfig(app.get());
  cam_->SaveApp();
  cam_->StopEditingApplication();
  cam_->CancelSession();

  //------------------------------------------
  // Run the FrameGrabber and make sure we are
  // getting image data
  //------------------------------------------
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  pcl::PointCloud<o3d3xx::PointT>::Ptr cloud;
  cv::Mat dimg;
  cv::Mat aimg;
  cv::Mat cimg;

  EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
  cloud = img->Cloud();
  dimg = img->DepthImage();
  aimg = img->AmplitudeImage();
  cimg = img->ConfidenceImage();

  int width = cloud->width;
  int height = cloud->height;

  ASSERT_GT(cloud->width, 0);
  ASSERT_GT(cloud->height, 0);

  ASSERT_EQ(dimg.size().width, width);
  ASSERT_EQ(dimg.size().height, height);

  ASSERT_EQ(aimg.size().width, width);
  ASSERT_EQ(aimg.size().height, height);

  ASSERT_EQ(cimg.size().width, width);
  ASSERT_EQ(cimg.size().height, height);

  //------------------------------------------
  // Stop the FrameGrabber and inspect that the
  // active application's schema is actually the
  // default schema
  //------------------------------------------
  fg->Stop();
  ASSERT_FALSE(fg->WaitForFrame(img.get(), 500));

  cam_->RequestSession();
  cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  cam_->EditApplication(idx_);
  app = cam_->GetAppConfig();
  schema = app->PcicTcpResultSchema();
  cam_->StopEditingApplication();
  cam_->CancelSession();

  boost::property_tree::ptree pt;
  std::istringstream is(schema);
  is.seekg(0, is.beg);
  boost::property_tree::read_json(is, pt);
  boost::property_tree::ptree elements_pt = pt.get_child("elements");
  int elem_idx = 1;
  for (auto& kv : elements_pt)
    {
      boost::property_tree::ptree elem_pt = kv.second;
      std::string elem_name = elem_pt.get<std::string>("id");

    switch (elem_idx)
      {
      case 1:
        ASSERT_EQ(elem_name, "start_string");
        break;

      case 2:
        ASSERT_EQ(elem_name, "normalized_amplitude_image");
        break;

      case 3:
        ASSERT_EQ(elem_name, "distance_image");
        break;

      case 4:
        ASSERT_EQ(elem_name, "x_image");
        break;

      case 5:
        ASSERT_EQ(elem_name, "y_image");
        break;

      case 6:
        ASSERT_EQ(elem_name, "z_image");
        break;

      case 7:
        ASSERT_EQ(elem_name, "confidence_image");
        break;

      case 8:
        ASSERT_EQ(elem_name, "diagnostic_data");
        break;

      case 9:
        ASSERT_EQ(elem_name, "end_string");
        break;

      default:
        // force an exception
        ASSERT_EQ(1, 0);
        break;
      }

      elem_idx++;
    }

  //------------------------------------------
  // Now, run the FrameGrabber's dtor. Check that
  // the active application's schema is the one
  // we had set above.
  //------------------------------------------
  fg.reset();

  cam_->RequestSession();
  cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  cam_->EditApplication(idx_);
  app = cam_->GetAppConfig();
  schema = app->PcicTcpResultSchema();
  cam_->StopEditingApplication();
  cam_->CancelSession();

  is.str(schema);
  is.seekg(0, is.beg);
  boost::property_tree::read_json(is, pt);
  elements_pt = pt.get_child("elements");
  elem_idx = 1;
  for (auto& kv : elements_pt)
    {
      boost::property_tree::ptree elem_pt = kv.second;
      std::string elem_name = elem_pt.get<std::string>("id");

    switch (elem_idx)
      {
      case 1:
        ASSERT_EQ(elem_name, "start_string");
        break;

      case 2:
        ASSERT_EQ(elem_name, "normalized_amplitude_image");
        break;

      case 3:
        ASSERT_EQ(elem_name, "diagnostic_data");
        break;

      case 4:
        ASSERT_EQ(elem_name, "end_string");
        break;

      default:
        // force an exception
        ASSERT_EQ(1, 0);
        break;
      }

      elem_idx++;
    }
}
