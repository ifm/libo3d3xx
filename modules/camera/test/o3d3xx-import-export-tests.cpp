#include "o3d3xx_camera.h"
#include <cstdint>
#include <memory>
#include <vector>
#include "gtest/gtest.h"

TEST(ImportExport_Tests, ImportExportApp)
{
  o3d3xx::Camera::Ptr cam =
    std::make_shared<o3d3xx::Camera>();

  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

  std::vector<o3d3xx::Camera::app_entry_t> apps =
    cam->GetApplicationList();

  int orig_num_apps = apps.size();
  int active_idx = cam->GetDeviceConfig()->ActiveApplication();

  std::vector<std::uint8_t> bytes;
  ASSERT_NO_THROW(bytes = cam->ExportIFMApp(active_idx));

  int new_idx = -1;
  ASSERT_NO_THROW(new_idx = cam->ImportIFMApp(bytes));

  apps = cam->GetApplicationList();
  ASSERT_EQ(apps.size(), orig_num_apps+1);

  ASSERT_NO_THROW(cam->DeleteApplication(new_idx));

  apps = cam->GetApplicationList();
  ASSERT_EQ(apps.size(), orig_num_apps);
}
