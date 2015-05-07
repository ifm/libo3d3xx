#include "o3d3xx.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "gtest/gtest.h"

class SpatialFilterTest : public ::testing::Test
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

TEST_F(SpatialFilterTest, SpatialFilterConfig_General)
{
  o3d3xx::SpatialFilterConfig::Ptr filt =
    std::make_shared<o3d3xx::SpatialFilterConfig>();

  ASSERT_EQ(filt->Type(),
	    static_cast<int>(o3d3xx::Camera::spatial_filter::OFF));

  ASSERT_THROW(filt->SetMaskSize(0), o3d3xx::error_t);


  //
  // Mean filter
  //

  filt = std::make_shared<o3d3xx::SpatialMeanFilterConfig>();

  ASSERT_EQ(filt->Type(),
	    static_cast<int>(o3d3xx::Camera::spatial_filter::MEAN_FILTER));

  int mask_size =
    static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_5x5);
  ASSERT_NO_THROW(filt->SetMaskSize(mask_size));
  ASSERT_EQ(mask_size, filt->MaskSize());


  //
  // Median filter
  //

  filt = std::make_shared<o3d3xx::SpatialMedianFilterConfig>();

  ASSERT_EQ(filt->Type(),
	    static_cast<int>(o3d3xx::Camera::spatial_filter::MEDIAN_FILTER));

  ASSERT_NO_THROW(filt->SetMaskSize(mask_size));
  ASSERT_EQ(mask_size, filt->MaskSize());


  //
  // Bilateral filter
  //

  filt = std::make_shared<o3d3xx::SpatialBilateralFilterConfig>();

  ASSERT_EQ(filt->Type(),
	    static_cast<int>(o3d3xx::Camera::spatial_filter::BILATERAL_FILTER));

  ASSERT_NO_THROW(filt->SetMaskSize(mask_size));
  ASSERT_EQ(mask_size, filt->MaskSize());
}


TEST_F(SpatialFilterTest, GetSpatialFilterParameters)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();

  std::vector<o3d3xx::Camera::spatial_filter> filts =
    {o3d3xx::Camera::spatial_filter::OFF,
     o3d3xx::Camera::spatial_filter::MEDIAN_FILTER,
     o3d3xx::Camera::spatial_filter::MEAN_FILTER,
     o3d3xx::Camera::spatial_filter::BILATERAL_FILTER};

  for (auto& filt : filts)
    {
      im->SetSpatialFilterType(static_cast<int>(filt));
      cam_->SetImagerConfig(im.get());

      std::unordered_map<std::string, std::string> params =
	cam_->GetSpatialFilterParameters();

      switch (filt)
	{
	case o3d3xx::Camera::spatial_filter::MEDIAN_FILTER:
	  ASSERT_EQ(params.size(), 1);
	  ASSERT_NO_THROW(params.at("MaskSize"));
	  break;

	case o3d3xx::Camera::spatial_filter::MEAN_FILTER:
	  ASSERT_EQ(params.size(), 1);
	  ASSERT_NO_THROW(params.at("MaskSize"));
	  break;

	case o3d3xx::Camera::spatial_filter::BILATERAL_FILTER:
	  // Despite with the docs say, the sensor currently only
	  // supports the `MaskSize` parameter.
	  // ASSERT_EQ(params.size(), 3);
	  ASSERT_EQ(params.size(), 1);
	  ASSERT_NO_THROW(params.at("MaskSize"));
	  break;

	default:
	  ASSERT_EQ(params.size(), 0);
	  break;
	}
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(SpatialFilterTest, GetSpatialFilterParameterLimits)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();

  std::vector<o3d3xx::Camera::spatial_filter> filts =
    {o3d3xx::Camera::spatial_filter::OFF,
     o3d3xx::Camera::spatial_filter::MEDIAN_FILTER,
     o3d3xx::Camera::spatial_filter::MEAN_FILTER,
     o3d3xx::Camera::spatial_filter::BILATERAL_FILTER};

  for (auto& filt : filts)
    {
      im->SetSpatialFilterType(static_cast<int>(filt));
      cam_->SetImagerConfig(im.get());

      std::unordered_map<std::string, std::string> params =
	cam_->GetSpatialFilterParameters();

      std::unordered_map<std::string,
			 std::unordered_map<std::string,
					    std::string> > limits =
	cam_->GetSpatialFilterParameterLimits();

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

TEST_F(SpatialFilterTest, SpatialFilterConfig_JSON)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::SpatialFilterConfig::Ptr filt = cam_->GetSpatialFilterConfig();
  std::string json = filt->ToJSON();

  o3d3xx::SpatialFilterConfig::Ptr filt2 =
    o3d3xx::SpatialFilterConfig::FromJSON(json);

  ASSERT_EQ(filt->Type(), filt2->Type());

  //
  // Apply a median filter, and run a similar test
  //

  o3d3xx::SpatialFilterConfig::Ptr median_filt =
    std::make_shared<o3d3xx::SpatialMedianFilterConfig>();
  median_filt->SetMaskSize(
    static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_5x5));

  cam_->SetSpatialFilterConfig(median_filt.get());


  filt = cam_->GetSpatialFilterConfig();
  json = filt->ToJSON();
  filt2 = o3d3xx::SpatialFilterConfig::FromJSON(json);

  // first, make sure it is a median filter
  ASSERT_EQ(median_filt->Type(), filt->Type());
  ASSERT_EQ(median_filt->MaskSize(), filt->MaskSize());
  ASSERT_EQ(median_filt->TypeStr(), filt->TypeStr());
  ASSERT_EQ(median_filt->MaskSizeStr(), filt->MaskSizeStr());

  // make sure the json looks good
  ASSERT_EQ(filt->Type(), filt2->Type());
  ASSERT_EQ(filt->MaskSize(), filt2->MaskSize());

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}
