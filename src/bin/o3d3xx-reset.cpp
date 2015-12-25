/*
 * Copyright (C) 2014 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <boost/program_options.hpp>
#include "o3d3xx.h"

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  uint32_t xmlrpc_port;
  std::string password;

  try
    {
      //---------------------------------------------------
      // Handle command-line arguments
      //---------------------------------------------------
      o3d3xx::CmdLineOpts opts("o3d3xx Factory Reset");

      po::options_description reset_opts("Reset Information");
      reset_opts.add_options()
        ("reboot", "Reboot the device after reset (ATTENTION: The network configuration may change)");
      opts.visible.add(reset_opts);

      po::options_description sensor_opts("Sensor Information");
      sensor_opts.add_options()
        ("sensor", "Create a new default \"camera\" application for a \"sensor\" device");
      opts.visible.add(sensor_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      //---------------------------------------------------
      // Reset the camera
      //---------------------------------------------------
      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      cam->RequestSession();
      cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
      if (opts.vm.count("sensor"))
        {
          // create a new application
          int app_idx = cam->CreateApplication();
          cam->EditApplication(app_idx);
          o3d3xx::ImagerConfig::Ptr imager = cam->GetImagerConfig();

          // Set values which comply with the default "camera" application
          imager->SetType("upto30m_low");
          imager->SetEnableRectificationAmplitudeImage(false);
          imager->SetEnableRectificationDistanceImage(false);
          imager->SetExposureTime(5000);
          imager->SetExposureTimeList("5000");
          imager->SetFrameRate(10.0);
          cam->SetImagerConfig(imager.get());
          cam->SaveApp();
          cam->StopEditingApplication();

          // activate the created application
          o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
          dev->SetActiveApplication(app_idx);
          cam->SetDeviceConfig(dev.get());
          cam->SaveDevice();
          cam->CancelSession();
        }
      else
        {
          cam->FactoryReset();
        }

      cam->CancelSession();

      if (opts.vm.count("reboot"))
        {
          cam->Reboot();
        }
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to reset camera: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}
