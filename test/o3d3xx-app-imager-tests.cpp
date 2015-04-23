#include "o3d3xx.h"
#include <memory>
#include "gtest/gtest.h"

//
// Due to a current bug in the sensor, we are separating out the `application'
// and `imager' unit tests from the other `camera' tests. The `SetUp' and
// `TearDown' functions on the test fixture here are leveraged to put the
// camera into a testable state.
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

  ASSERT_NO_THROW(params.at("Name"));
  ASSERT_NO_THROW(params.at("Description"));
  ASSERT_NO_THROW(params.at("TriggerMode"));
  ASSERT_NO_THROW(params.at("PcicTcpResultOutputEnabled"));
  ASSERT_NO_THROW(params.at("PcicTcpResultSchema"));

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
  ASSERT_EQ(app->PcicTcpResultOutputEnabled(),
	    app2->PcicTcpResultOutputEnabled());
  ASSERT_EQ(app->PcicTcpResultSchema(),
	    app2->PcicTcpResultSchema());

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
  ASSERT_EQ(imager_types.size(), 8);

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

      ASSERT_NO_THROW(params.at("Type"));
      ASSERT_NO_THROW(params.at("TypeHash"));
      ASSERT_NO_THROW(params.at("ReduceMotionArtifacts"));
      ASSERT_NO_THROW(params.at("Channel"));
      ASSERT_NO_THROW(params.at("ClippingLeft"));
      ASSERT_NO_THROW(params.at("ClippingRight"));
      ASSERT_NO_THROW(params.at("ClippingBottom"));
      ASSERT_NO_THROW(params.at("ClippingTop"));
      ASSERT_NO_THROW(params.at("FrameRate"));
      ASSERT_NO_THROW(params.at("SpatialFilterType"));
      //ASSERT_NO_THROW(params.at("AverageFilterNumPictures"));

      if ((type == "under5m_high") ||
	  (type == "upto30m_high"))
	{
	  ASSERT_THROW(params.at("ExposureTime"), std::out_of_range);
	}
      else
	{
	  ASSERT_NO_THROW(params.at("ExposureTime"));
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

      // need to do more investigation on this imager type
      // ... for now skipping tests
      if (type == "upto30m_high")
	{
	  continue;
	}

      // mutate the config
      ASSERT_NO_THROW(im->SetFrameRate(10));
      ASSERT_NO_THROW(im->SetReduceMotionArtifacts(true));
      ASSERT_NO_THROW(im->SetSpatialFilterType(1));

      // XXX: For prototype camera, max here is 1
      //ASSERT_NO_THROW(im->SetAverageFilterNumPictures(2));

      // send new config parameters to the sensor
      ASSERT_NO_THROW(cam_->SetImagerConfig(im.get()));

      // check if they are correct when queried again
      o3d3xx::ImagerConfig::Ptr im2 = cam_->GetImagerConfig();
      ASSERT_EQ(im2->Type(), type);
      ASSERT_EQ(im2->FrameRate() , 10);
      ASSERT_EQ(im2->ReduceMotionArtifacts(), true);
      ASSERT_EQ(im2->SpatialFilterType(), 1);
      //ASSERT_EQ(im2->AverageFilterNumPictures(), 2);
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

      // need to do more investigation on this imager type
      // ... for now skipping tests
      if (type == "upto30m_high")
	{
	  continue;
	}

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

  ASSERT_EQ(im->FrameRate(), im2->FrameRate());
  ASSERT_EQ(im->ClippingLeft(), im2->ClippingLeft());
  ASSERT_EQ(im->ClippingTop(), im2->ClippingTop());
  ASSERT_EQ(im->ClippingRight(), im2->ClippingRight());
  ASSERT_EQ(im->ClippingBottom(), im2->ClippingBottom());
  ASSERT_EQ(im->ReduceMotionArtifacts(), im2->ReduceMotionArtifacts());
  ASSERT_EQ(im->SpatialFilterType(), im2->SpatialFilterType());
  ASSERT_EQ(im->AverageFilterNumPictures(), im2->AverageFilterNumPictures());

  cam_->StopEditingApplication();
  cam_->DeleteApplication(new_idx);
}
