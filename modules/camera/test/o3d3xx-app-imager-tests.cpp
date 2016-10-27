#include "o3d3xx_camera.h"
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include "gtest/gtest.h"

//
// The `SetUp' and `TearDown' functions on the test fixture here are
// leveraged to put the camera into a testable state.
//

class AppImagerTest : public ::testing::Test
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

TEST_F(AppImagerTest, CopyDeleteApplication)
{
  std::vector<o3d3xx::Camera::app_entry_t> apps = cam_->GetApplicationList();
  int napps = apps.size();
  ASSERT_EQ(napps, 1);

  int new_idx = cam_->CopyApplication(apps.at(0).index);
  apps = cam_->GetApplicationList();
  ASSERT_GT(apps.size(), napps);

  ASSERT_NO_THROW(cam_->DeleteApplication(new_idx));
  apps = cam_->GetApplicationList();
  ASSERT_EQ(napps, apps.size());

  // copy an invalid application
  ASSERT_THROW(cam_->CopyApplication(100), o3d3xx::error_t);
  try
    {
      cam_->CopyApplication(100);
    }
  catch (const o3d3xx::error_t& ex)
    {
      ASSERT_EQ(ex.code(), O3D3XX_XMLRPC_INVALID_APPLICATION);
    }
}

TEST_F(AppImagerTest, CreateApplication)
{
  std::vector<o3d3xx::Camera::app_entry_t> apps = cam_->GetApplicationList();
  int napps = apps.size();
  ASSERT_GE(napps, 1);

  int new_idx = cam_->CreateApplication();
  apps = cam_->GetApplicationList();
  ASSERT_EQ(napps+1, apps.size());

  cam_->DeleteApplication(new_idx);
  apps = cam_->GetApplicationList();
  ASSERT_EQ(napps, apps.size());
}

TEST_F(AppImagerTest, ChangeAppNameAndDescription)
{
  int new_idx = cam_->CreateApplication();
  std::string name("Foo");
  std::string descr("Bar");
  cam_->ChangeAppNameAndDescription(new_idx, name, descr);

  std::vector<o3d3xx::Camera::app_entry_t> apps = cam_->GetApplicationList();
  for (auto& a : apps)
    {
      if (a.index == new_idx)
        {
          ASSERT_EQ(a.name, name);
          ASSERT_EQ(a.description, descr);
        }
    }

  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, EditApplication)
{
  int new_idx = cam_->CreateApplication();
  ASSERT_NO_THROW(cam_->EditApplication(new_idx));
  ASSERT_NO_THROW(cam_->StopEditingApplication());

  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, GetAppParameters)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::unordered_map<std::string, std::string> params =
    cam_->GetAppParameters();

  // for (auto& kv : params)
  //   {
  //     std::cout << kv.first << "=" << kv.second << std::endl;
  //   }

  ASSERT_EQ(params.size(), 9);

  ASSERT_NO_THROW(params.at("Name"));
  ASSERT_NO_THROW(params.at("Description"));
  ASSERT_NO_THROW(params.at("TriggerMode"));
  ASSERT_NO_THROW(params.at("PcicTcpResultSchema"));
  ASSERT_NO_THROW(params.at("PcicEipResultSchema"));
  ASSERT_NO_THROW(params.at("PcicPnioResultSchema"));
  ASSERT_NO_THROW(params.at("TemplateInfo"));
  ASSERT_NO_THROW(params.at("Type"));
  ASSERT_NO_THROW(params.at("LogicGraph"));

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, AppConfig)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::string new_name("Foo");
  std::string new_description("Bar");
  o3d3xx::AppConfig::Ptr app = cam_->GetAppConfig();
  app->SetName(new_name);
  app->SetDescription(new_description);
  app->SetTriggerMode(
    static_cast<int>(o3d3xx::Camera::trigger_mode::PROCESS_INTERFACE));

  ASSERT_NO_THROW(cam_->SetAppConfig(app.get()));
  ASSERT_NO_THROW(cam_->SaveApp());

  o3d3xx::AppConfig::Ptr app_new = cam_->GetAppConfig();
  ASSERT_EQ(app->Name(), app_new->Name());
  ASSERT_EQ(app->Description(), app_new->Description());
  ASSERT_EQ(app->TriggerMode(), app_new->TriggerMode());

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, AppConfig_JSON)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::AppConfig::Ptr app = cam_->GetAppConfig();
  std::string json = app->ToJSON();

  o3d3xx::AppConfig::Ptr app2 = o3d3xx::AppConfig::FromJSON(json);

  ASSERT_EQ(app->Name(), app2->Name());
  ASSERT_EQ(app->Description(), app2->Description());
  ASSERT_EQ(app->TriggerMode(), app2->TriggerMode());
  ASSERT_EQ(app->PcicTcpResultSchema(),
            app2->PcicTcpResultSchema());
  ASSERT_EQ(app->PcicEipResultSchema(),
            app2->PcicEipResultSchema());
  ASSERT_EQ(app->PcicPnioResultSchema(),
            app2->PcicPnioResultSchema());
  ASSERT_EQ(app->LogicGraph(), app2->LogicGraph());

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, GetAvailableImagerTypes)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types;
  ASSERT_NO_THROW(imager_types = cam_->GetAvailableImagerTypes());
  //ASSERT_EQ(imager_types.size(), 8);

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, ChangeImagerType)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types =
    cam_->GetAvailableImagerTypes();

  for (auto& type : imager_types)
    {
      ASSERT_NO_THROW(cam_->ChangeImagerType(type));

      std::unordered_map<std::string, std::string> params =
        cam_->GetImagerParameters();

      ASSERT_EQ(params.at("Type"), type);
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, GetImagerParameters)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types =
    cam_->GetAvailableImagerTypes();

  for (auto& type : imager_types)
    {
      cam_->ChangeImagerType(type);

      std::unordered_map<std::string, std::string> params =
        cam_->GetImagerParameters();

      // std::cout << type << ":" << std::endl;
      // for (auto& kv : params)
      //   {
      //     std::cout << kv.first << " --> " << kv.second << std::endl;
      //   }

      ASSERT_NO_THROW(params.at("Channel"));
      ASSERT_NO_THROW(params.at("ClippingBottom"));
      ASSERT_NO_THROW(params.at("ClippingLeft"));
      ASSERT_NO_THROW(params.at("ClippingRight"));
      ASSERT_NO_THROW(params.at("ClippingTop"));
      ASSERT_NO_THROW(params.at("ContinuousAutoExposure"));
      ASSERT_NO_THROW(params.at("EnableAmplitudeCorrection"));
      ASSERT_NO_THROW(params.at("EnableFastFrequency"));
      ASSERT_NO_THROW(params.at("EnableFilterAmplitudeImage"));
      ASSERT_NO_THROW(params.at("EnableFilterDistanceImage"));
      ASSERT_NO_THROW(params.at("EnableRectificationAmplitudeImage"));
      ASSERT_NO_THROW(params.at("EnableRectificationDistanceImage"));
      ASSERT_NO_THROW(params.at("UseSimpleBinning"));
      ASSERT_NO_THROW(params.at("ExposureTimeList"));
      ASSERT_NO_THROW(params.at("FrameRate"));
      ASSERT_NO_THROW(params.at("MinimumAmplitude"));
      ASSERT_NO_THROW(params.at("Resolution"));
      ASSERT_NO_THROW(params.at("ClippingCuboid"));
      ASSERT_NO_THROW(params.at("SpatialFilterType"));
      ASSERT_NO_THROW(params.at("SymmetryThreshold"));
      ASSERT_NO_THROW(params.at("TemporalFilterType"));
      ASSERT_NO_THROW(params.at("ThreeFreqMax2FLineDistPercentage"));
      ASSERT_NO_THROW(params.at("ThreeFreqMax3FLineDistPercentage"));
      ASSERT_NO_THROW(params.at("TwoFreqMaxLineDistPercentage"));
      ASSERT_NO_THROW(params.at("Type"));
      ASSERT_NO_THROW(params.at("MaxAllowedLEDFrameRate"));

      if (boost::algorithm::ends_with(type, "high"))
        {
          ASSERT_THROW(params.at("ExposureTime"), std::out_of_range);
          ASSERT_THROW(params.at("ExposureTimeRatio"), std::out_of_range);

          ASSERT_EQ(params.size(), 26);
        }
      else if (boost::algorithm::ends_with(type, "low"))
        {
          ASSERT_NO_THROW(params.at("ExposureTime"));
          ASSERT_THROW(params.at("ExposureTimeRatio"), std::out_of_range);

          ASSERT_EQ(params.size(), 27);
        }
      else
        {
          ASSERT_NO_THROW(params.at("ExposureTime"));
          ASSERT_NO_THROW(params.at("ExposureTimeRatio"));

          ASSERT_EQ(params.size(), 28);
        }
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, GetImagerParameterLimits)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types =
    cam_->GetAvailableImagerTypes();

  for (auto& type : imager_types)
    {
      cam_->ChangeImagerType(type);

      std::unordered_map<std::string, std::string> params =
        cam_->GetImagerParameters();

      //
      // tests that the call to fetch the parameter limits does not throw an
      // exception.
      //
      std::unordered_map<std::string,
                         std::unordered_map<std::string,
                                            std::string> > limits =
        cam_->GetImagerParameterLimits();

      for (auto& param : params)
        {
          try
            {
              //
              // Now we simply want to make sure that
              // the only limits, if the parameter has
              // limits, are "min" and "max" ... and
              // that the values make some sort of sense.
              // Otherwise, we need to investigate further.
              //

              std::unordered_map<std::string, std::string>
                param_limits = limits.at(param.first);

              ASSERT_EQ(param_limits.size(), 2);
              ASSERT_LE(std::stod(param_limits.at("min")),
                        std::stod(param_limits.at("max")));

            }
          catch (const std::out_of_range& ex)
            {
              // no limits defined for this parameter.
            }
        }
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}


TEST_F(AppImagerTest, ImagerConfig)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types =
    cam_->GetAvailableImagerTypes();

  for (auto& type : imager_types)
    {
      ASSERT_NO_THROW(cam_->ChangeImagerType(type));

      o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();
      ASSERT_EQ(im->Type(), type);

      ASSERT_NO_THROW(im->SetContinuousAutoExposure(true));
      ASSERT_NO_THROW(im->SetEnableAmplitudeCorrection(false));
      ASSERT_NO_THROW(im->SetEnableFastFrequency(true));
      ASSERT_NO_THROW(im->SetEnableFilterAmplitudeImage(false));
      ASSERT_NO_THROW(im->SetEnableFilterDistanceImage(false));
      ASSERT_NO_THROW(im->SetEnableRectificationAmplitudeImage(true));
      ASSERT_NO_THROW(im->SetEnableRectificationDistanceImage(true));
      ASSERT_NO_THROW(im->SetUseSimpleBinning(true));

      // mutate the config
      if (boost::algorithm::ends_with(type, "high"))
        {

        }
      else if (boost::algorithm::ends_with(type, "low"))
        {
          ASSERT_NO_THROW(im->SetExposureTime(2000));
        }
      else
        {
          ASSERT_NO_THROW(im->SetExposureTime(2000));
          ASSERT_NO_THROW(im->SetExposureTimeRatio(5));
        }

      ASSERT_NO_THROW(im->SetFrameRate(10));
      ASSERT_NO_THROW(im->SetMinimumAmplitude(5));
      ASSERT_NO_THROW(im->SetResolution(o3d3xx::RES_23K));
      ASSERT_NO_THROW(im->SetSpatialFilterType(
        static_cast<int>(o3d3xx::Camera::spatial_filter::MEDIAN_FILTER)));
      //ASSERT_NO_THROW(im->SetSymmetryThreshold(1));
      ASSERT_NO_THROW(im->SetTemporalFilterType(
        static_cast<int>(
          o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER)));

      // send new config parameters to the sensor
      ASSERT_NO_THROW(cam_->SetImagerConfig(im.get()));

      // check if they are correct when queried again
      o3d3xx::ImagerConfig::Ptr im2 = cam_->GetImagerConfig();
      ASSERT_EQ(im2->Type(), im->Type());

      if (boost::algorithm::ends_with(type, "high"))
        {

        }
      else if (boost::algorithm::ends_with(type, "low"))
        {
          ASSERT_EQ(im2->ExposureTime(), im->ExposureTime());
        }
      else
        {
          ASSERT_EQ(im2->ExposureTime(), im->ExposureTime());
          ASSERT_EQ(im2->ExposureTimeRatio(),
                    im->ExposureTimeRatio());
        }

      ASSERT_EQ(im2->FrameRate() , im->FrameRate());
      ASSERT_EQ(im2->MinimumAmplitude(), im->MinimumAmplitude());
      ASSERT_EQ(im2->SpatialFilterType(), im->SpatialFilterType());
      //ASSERT_EQ(im2->SymmetryThreshold(), im->SymmetryThreshold());
      ASSERT_EQ(im2->TemporalFilterType(), im->TemporalFilterType());
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, ImagerConfigValueOutOfRange)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types =
    cam_->GetAvailableImagerTypes();

  for (auto& type : imager_types)
    {
      ASSERT_NO_THROW(cam_->ChangeImagerType(type));

      o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();
      ASSERT_EQ(im->Type(), type);

      // mutate the config
      ASSERT_NO_THROW(im->SetFrameRate(1000.0));

      // send new config parameters to the sensor
      bool ex_thrown = false;
      try
        {
          cam_->SetImagerConfig(im.get());
        }
      catch (const o3d3xx::error_t& ex)
        {
          ASSERT_EQ(ex.code(), O3D3XX_XMLRPC_VALUE_OUT_OF_RANGE);
          ex_thrown = true;
        }

      ASSERT_TRUE(ex_thrown);
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, ImagerConfig_JSON)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();
  std::string json = im->ToJSON();

  o3d3xx::ImagerConfig::Ptr im2 = o3d3xx::ImagerConfig::FromJSON(json);

  ASSERT_EQ(im->Type(), im2->Type());
  ASSERT_EQ(im->Channel(), im2->Channel());
  ASSERT_EQ(im->ClippingBottom(), im2->ClippingBottom());
  ASSERT_EQ(im->ClippingLeft(), im2->ClippingLeft());
  ASSERT_EQ(im->ClippingRight(), im2->ClippingRight());
  ASSERT_EQ(im->ClippingTop(), im2->ClippingTop());
  ASSERT_EQ(im->ContinuousAutoExposure() , im2->ContinuousAutoExposure());
  ASSERT_EQ(im->EnableAmplitudeCorrection(), im2->EnableAmplitudeCorrection());
  ASSERT_EQ(im->EnableFastFrequency(), im2->EnableFastFrequency());
  ASSERT_EQ(im->EnableFilterAmplitudeImage(),
            im2->EnableFilterAmplitudeImage());
  ASSERT_EQ(im->EnableFilterDistanceImage(), im2->EnableFilterDistanceImage());
  ASSERT_EQ(im->EnableRectificationAmplitudeImage(),
            im2->EnableRectificationAmplitudeImage());
  ASSERT_EQ(im->EnableRectificationDistanceImage(),
            im2->EnableRectificationDistanceImage());
  ASSERT_EQ(im->UseSimpleBinning(), im2->UseSimpleBinning());

  if (boost::algorithm::ends_with(im->Type(), "high"))
    {

    }
  else if (boost::algorithm::ends_with(im->Type(), "low"))
    {
      ASSERT_EQ(im->ExposureTime(), im2->ExposureTime());
    }
  else
    {
      ASSERT_EQ(im->ExposureTime(), im2->ExposureTime());
      ASSERT_EQ(im->ExposureTimeRatio(), im2->ExposureTimeRatio());
    }

  ASSERT_EQ(im->ExposureTimeList(), im2->ExposureTimeList());

  ASSERT_EQ(im->FrameRate(), im2->FrameRate());
  ASSERT_EQ(im->MinimumAmplitude(), im2->MinimumAmplitude());
  ASSERT_EQ(im->Resolution(), im2->Resolution());
  ASSERT_EQ(im->ClippingCuboid(), im2->ClippingCuboid());
  ASSERT_EQ(im->SpatialFilterType(), im2->SpatialFilterType());
  ASSERT_EQ(im->SymmetryThreshold(), im2->SymmetryThreshold());
  ASSERT_EQ(im->TemporalFilterType(), im2->TemporalFilterType());
  ASSERT_EQ(im->ThreeFreqMax2FLineDistPercentage(),
            im2->ThreeFreqMax2FLineDistPercentage());
  ASSERT_EQ(im->ThreeFreqMax3FLineDistPercentage(),
            im2->ThreeFreqMax3FLineDistPercentage());
  ASSERT_EQ(im->TwoFreqMaxLineDistPercentage(),
            im2->TwoFreqMaxLineDistPercentage());

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}

TEST_F(AppImagerTest, Exposure)
{
  o3d3xx::DeviceConfig::Ptr dev = cam_->GetDeviceConfig();
  int new_idx = cam_->CopyApplication(dev->ActiveApplication());
  cam_->EditApplication(new_idx);

  std::vector<std::string> imager_types =
    cam_->GetAvailableImagerTypes();

  for (auto& type : imager_types)
    {
      ASSERT_NO_THROW(cam_->ChangeImagerType(type));

      o3d3xx::ImagerConfig::Ptr im = cam_->GetImagerConfig();
      ASSERT_EQ(im->Type(), type);

      // mutate the config
      if (boost::algorithm::ends_with(type, "high"))
        {

        }
      else if (boost::algorithm::ends_with(type, "low"))
        {
          ASSERT_NO_THROW(im->SetExposureTime(2000));
        }
      else
        {
          ASSERT_NO_THROW(im->SetExposureTime(2000));
          ASSERT_NO_THROW(im->SetExposureTimeRatio(5));
        }

      // send new config parameters to the sensor
      ASSERT_NO_THROW(cam_->SetImagerConfig(im.get()));

      // verify the absolute exposure times
      im = cam_->GetImagerConfig();
      std::vector<std::string> exposure_strings;
      std::string exposure_time_list = im->ExposureTimeList();

      boost::split(exposure_strings,
                   exposure_time_list,
                   boost::is_any_of(";"));

      if (boost::algorithm::ends_with(type, "high"))
        {
          ASSERT_EQ(exposure_strings.size(), 3);
        }
      else if (boost::algorithm::ends_with(type, "low"))
        {
          ASSERT_EQ(exposure_strings.size(), 1);
          ASSERT_EQ(std::atoi(exposure_strings.at(0).c_str()), 2000);
        }
      else
        {
          ASSERT_EQ(exposure_strings.size(), 2);
          ASSERT_EQ(std::atoi(exposure_strings.at(0).c_str()), 2000/5);
          ASSERT_EQ(std::atoi(exposure_strings.at(1).c_str()), 2000);
        }
    }

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}
