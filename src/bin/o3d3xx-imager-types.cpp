/*
 * Copyright (C) 2015 Love Park Robotics, LLC
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

#include <algorithm>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/program_options.hpp>
#include "o3d3xx.h"

//
// Lists available imagers and (optionally) their configurable parameters.
//

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  bool verbose;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx Imager Types");

      po::options_description im_opts("Imager Types");
      im_opts.add_options()
        ("params,p",
         po::value<bool>()->default_value(false),
         "Include tunable parameters for each imager type");

      opts.visible.add(im_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      verbose = opts.vm["params"].as<bool>();

      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      cam->RequestSession();
      cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

      o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
      cam->EditApplication(dev->ActiveApplication());

      std::vector<std::string> imager_types = cam->GetAvailableImagerTypes();
      std::sort(imager_types.begin(), imager_types.end());

      for (auto& type : imager_types)
        {
          std::cout << type << std::endl;

          if (verbose)
            {
              cam->ChangeImagerType(type);

              std::unordered_map<std::string, std::string> params =
                cam->GetImagerParameters();

              std::unordered_map<std::string,
                                 std::unordered_map<std::string,
                                                    std::string> > limits =
                cam->GetImagerParameterLimits();

              std::map<std::string, std::string>
                params_sorted(params.begin(), params.end());

              for (auto it = params_sorted.begin();
                   it != params_sorted.end(); ++it)
                {
                  std::cout << "\t" << it->first << ": ";

                  try
                    {
                      std::unordered_map<std::string, std::string>
                        param_limits = limits.at(it->first);

                      std::cout << "[min=" << param_limits.at("min")
                                << ", max=" << param_limits.at("max")
                                << "]";
                    }
                  catch (const std::out_of_range& ex)
                    {
                      // no limits defined for this parameter
                      std::cout << "no limits defined";
                    }

                  std::cout << std::endl;
                }

              std::cout << std::endl;
            }
        }
      cam->StopEditingApplication();
    }
  catch (const std::exception& ex)
    {
      std::cerr << "Failed to list available imagers: "
                << ex.what() << std::endl;
      return 1;
    }

  return 0;
}
