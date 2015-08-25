#include "o3d3xx.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "gtest/gtest.h"

class TemporalFilterTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    cam_ = std::make_shared<o3d3xx::Camera>();
    cam_->RequestSession();
    cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

    o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
    int active_idx = dev->ActiveApplication();

    for (auto& app : cam_->GetApplicationList())
      {
        if (app.index != active_idx)
          {
            cam_->DeleteApplication(app.index);
          }
      }

    cam_->SaveDevice();
  }

  virtual void TearDown()
  {

  }

  o3d3xx::Camera::Ptr cam_;
};

TEST_F(TemporalFilterTest, TemporalFilterConfig_General)
{
  int n_imgs = 5;

  o3d3xx::TemporalFilterConfig::Ptr filt =
    std::make_shared<o3d3xx::TemporalFilterConfig>();

  ASSERT_EQ(filt->Type(),
            static_cast<int>(o3d3xx::Camera::temporal_filter::OFF));

  ASSERT_THROW(filt->SetNumberOfImages(n_imgs), o3d3xx::error_t);

  //
  // Mean filter
  //
  filt = std::make_shared<o3d3xx::TemporalMeanFilterConfig>();

  ASSERT_EQ(filt->Type(),
    static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER));

  ASSERT_NO_THROW(filt->SetNumberOfImages(n_imgs));
  ASSERT_EQ(n_imgs, filt->NumberOfImages());

  //
  // Adaptive exponential filter
  //
  filt =
    std::make_shared<o3d3xx::TemporalAdaptiveExponentialFilterConfig>();

  ASSERT_EQ(filt->Type(),
    static_cast<int>(o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER));

  ASSERT_THROW(filt->SetNumberOfImages(n_imgs), o3d3xx::error_t);
}

TEST_F(TemporalFilterTest, GetTemporalFilterParameters)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();

  std::vector<o3d3xx::Camera::temporal_filter> filts =
    {o3d3xx::Camera::temporal_filter::OFF,
     o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER,
     o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER};

  for (auto& filt : filts)
    {
      im->SetTemporalFilterType(static_cast<int>(filt));
      cam_->SetImagerConfig(im.get());

      std::unordered_map<std::string, std::string> params =
        cam_->GetTemporalFilterParameters();

      switch (filt)
        {
        case o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER:
          ASSERT_EQ(params.size(), 1);
          ASSERT_NO_THROW(params.at("NumberOfImages"));
          break;

        case o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER:
          // Despite what the docs sya, the sensor currently does not truly
          // support this filter type ... you can set it, but you cannot
          // configure it.
          //ASSERT_EQ(params.size(), 4);
          //ASSERT_NO_THROW(params.at("MinSmoothDiff"));
          //ASSERT_NO_THROW(params.at("MinSDAlpha"));
          //ASSERT_NO_THROW(params.at("MaxSmoothDiff"));
          //ASSERT_NO_THROW(params.at("MaxSDAlpha"));
          ASSERT_EQ(params.size(), 0);
          break;

        default:
          ASSERT_EQ(params.size(), 0);
          break;
        }
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(TemporalFilterTest, GetTemporalFilterParameterLimits)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();

  std::vector<o3d3xx::Camera::temporal_filter> filts =
    {o3d3xx::Camera::temporal_filter::OFF,
     o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER,
     o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER};

  for (auto& filt : filts)
    {
      im->SetTemporalFilterType(static_cast<int>(filt));
      cam_->SetImagerConfig(im.get());

      std::unordered_map<std::string, std::string> params =
        cam_->GetTemporalFilterParameters();

      std::unordered_map<std::string,
                         std::unordered_map<std::string,
                                            std::string> > limits =
        cam_->GetTemporalFilterParameterLimits();

      for (auto& param : params)
        {
          std::unordered_map<std::string, std::string>
            param_limits = limits.at(param.first);

          ASSERT_EQ(param_limits.size(), 2);
          ASSERT_LE(std::stoi(param_limits.at("min")),
                    std::stoi(param_limits.at("max")));
        }
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(TemporalFilterTest, TemporalFilterConfig_JSON)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::TemporalFilterConfig::Ptr filt = cam_->GetTemporalFilterConfig();
  std::string json = filt->ToJSON();

  o3d3xx::TemporalFilterConfig::Ptr filt2 =
    o3d3xx::TemporalFilterConfig::FromJSON(json);

  ASSERT_EQ(filt->Type(), filt2->Type());

  //
  // Apply a mean filter, and run a similar test
  //

  o3d3xx::TemporalFilterConfig::Ptr mean_filt =
    std::make_shared<o3d3xx::TemporalMeanFilterConfig>();
  mean_filt->SetNumberOfImages(5);

  cam_->SetTemporalFilterConfig(mean_filt.get());

  filt = cam_->GetTemporalFilterConfig();
  json = filt->ToJSON();
  filt2 = o3d3xx::TemporalFilterConfig::FromJSON(json);

  // make sure it is a mean filter
  ASSERT_EQ(mean_filt->Type(), filt->Type());
  ASSERT_EQ(mean_filt->NumberOfImages(), filt->NumberOfImages());

  // make sure the json looks good
  ASSERT_EQ(filt->Type(), filt2->Type());
  ASSERT_EQ(filt->NumberOfImages(), filt2->NumberOfImages());

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}
