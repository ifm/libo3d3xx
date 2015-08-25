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
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <boost/program_options.hpp>
#include "o3d3xx.h"

//
// Removes an application at the specified index
//

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  int index;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx Remove Application");

      po::options_description rm_opts("Remove");
      rm_opts.add_options()
        ("index,i",
         po::value<int>()->default_value(-1),
         "Index of application to remove");

      opts.visible.add(rm_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      index = opts.vm["index"].as<int>();
      if (index > 0)
        {
          o3d3xx::Camera::Ptr cam =
            std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

          cam->RequestSession();
          cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
          cam->DeleteApplication(index);
        }
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to remove application at index='"
                << index << "':"
                << std::endl << e.what() << std::endl;
      return 1;
    }

  return 0;
}
