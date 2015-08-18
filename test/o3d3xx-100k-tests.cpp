#include "o3d3xx.h"
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "gtest/gtest.h"

class DenseImageTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    cam_ = std::make_shared<o3d3xx::Camera>();
    cam_->RequestSession();
    cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

    dev_ = cam_->GetDeviceConfig();
    old_active_idx_ = dev_->ActiveApplication();

    std::string test_dir(__FILE__);
    test_dir.erase(test_dir.find_last_of("/") + 1);
    std::string in = test_dir + infile_;

    std::ifstream ifs(in, std::ios::in|std::ios::binary);
    ifs.unsetf(std::ios::skipws);
    std::streampos file_size;
    ifs.seekg(0, std::ios::end);
    file_size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<std::uint8_t> bytes;
    bytes.reserve(file_size);

    bytes.insert(bytes.begin(),
                 std::istream_iterator<std::uint8_t>(ifs),
                 std::istream_iterator<std::uint8_t>());

    idx_ = cam_->ImportIFMApp(bytes);
    dev_->SetActiveApplication(idx_);
    cam_->SetDeviceConfig(dev_.get());
    cam_->SaveDevice();
  }

  virtual void TearDown()
  {
    dev_->SetActiveApplication(old_active_idx_);
    cam_->SetDeviceConfig(dev_.get());
    cam_->SaveDevice();
    cam_->DeleteApplication(idx_);
  }

  o3d3xx::Camera::Ptr cam_;
  o3d3xx::DeviceConfig::Ptr dev_;
  std::string infile_ = "data/100k.o3d3xxapp";
  int idx_ = -1;
  int old_active_idx_ = -1;
};

TEST_F(DenseImageTest, ActiveApplication)
{
  ASSERT_EQ(idx_, dev_->ActiveApplication());
}

//
// Many more tests to come...
//
