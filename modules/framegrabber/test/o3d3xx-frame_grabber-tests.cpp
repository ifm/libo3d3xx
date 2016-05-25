#include <cstdint>
#include <memory>
#include "gtest/gtest.h"
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

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
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  int i = 0;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
      i++;
    }

  EXPECT_EQ(i, 10);
  fg->Stop();
  EXPECT_FALSE(fg->WaitForFrame(buff.get(), 500));
}

TEST_F(FrameGrabberTest, WaitForFrameFast)
{
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  int i = 0;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000, false, false));
      i++;
    }

  EXPECT_EQ(i, 10);
  fg->Stop();
  EXPECT_FALSE(fg->WaitForFrame(buff.get(), 500));
}

TEST_F(FrameGrabberTest, CustomSchemas)
{
  std::uint16_t mask = o3d3xx::DEFAULT_SCHEMA_MASK;

  mask &= ~o3d3xx::IMG_RDIS;

  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam_, mask);

  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
}

TEST_F(FrameGrabberTest, ByteBufferCopyCtor)
{
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));

  o3d3xx::ByteBuffer::Ptr buff2 =
    std::make_shared<o3d3xx::ByteBuffer>(*(buff.get()));

  EXPECT_TRUE(buff->Dirty() != buff2->Dirty());
  EXPECT_TRUE(buff->Bytes() == buff2->Bytes());
}

TEST_F(FrameGrabberTest, ByteBufferCopyAssignmentOperator)
{
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();
  o3d3xx::ByteBuffer::Ptr buff2 = std::make_shared<o3d3xx::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));

  *(buff2.get()) = *(buff.get());

  EXPECT_TRUE(buff->Dirty() != buff2->Dirty());
  EXPECT_TRUE(buff->Bytes() == buff2->Bytes());
}

TEST_F(FrameGrabberTest, FrameGrabberRecycling)
{
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  for (int i = 0; i < 5; ++i)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
    }

  fg.reset(new o3d3xx::FrameGrabber(cam_));
  for (int i = 0; i < 5; ++i)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
    }
}

TEST_F(FrameGrabberTest, SoftwareTrigger)
{
  //
  // Modify the active application to operate in S/W trigger mode
  //
  this->cam_->RequestSession();
  this->cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  this->cam_->EditApplication(this->idx_);
  o3d3xx::AppConfig::Ptr app = this->cam_->GetAppConfig();
  app->SetTriggerMode((int) o3d3xx::Camera::trigger_mode::PROCESS_INTERFACE);
  this->cam_->SetAppConfig(app.get());
  this->cam_->SaveApp();
  this->cam_->StopEditingApplication();
  this->cam_->CancelSession();

  //
  // Data structures we use to grab image data from the camera
  //
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam_);
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  //
  // Now, waiting for image data should timeout
  //
  EXPECT_FALSE(fg->WaitForFrame(buff.get(), 1000));

  //
  // Now, do a s/w trigger and fetch the data
  //
  fg->SWTrigger();
  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
}
