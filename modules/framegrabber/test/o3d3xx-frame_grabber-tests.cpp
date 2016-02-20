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
