#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include "o3d3xx_camera.h"
#include "o3d3xx_pcicclient.h"

class PCICClientTest : public ::testing::Test
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

TEST_F(PCICClientTest, Connect)
{
  o3d3xx::PCICClient::Ptr pc = std::make_shared<o3d3xx::PCICClient>(cam_);

  int i = 0;
  while (i < 10)
    {
      //      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
      bool response = false;

      pc->Call(std::vector<std::uint8_t>(
        {'C','?'}), [=](std::vector<std::uint8_t> content)
        {
	  std::cout << "ready " << i << " " << std::endl;
          std::string s;
          s.assign(content.begin(), content.end());
          std::cout << s.size() << " " << s << std::endl;
        }
        );

      std::this_thread::sleep_for(std::chrono::seconds(1));
      i++;
    }

  EXPECT_EQ(i, 10);
  pc->Stop();
}

