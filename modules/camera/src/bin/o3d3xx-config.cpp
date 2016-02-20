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
#include "o3d3xx_camera.h"

//
// Configures the camera based upon a JSON configuration file.
//

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  std::string infile;
  std::string json;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx Config");

      po::options_description config_opts("Config Information");
      config_opts.add_options()
        ("file,f",
         po::value<std::string>()->default_value("-"),
         "Input JSON configuration file");

      opts.visible.add(config_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      infile.assign(opts.vm["file"].as<std::string>());

      if (infile == "-")
        {
          std::string line;
          std::ostringstream buff;
          while (std::getline(std::cin, line))
            {
              buff << line << std::endl;
            }

          json.assign(buff.str());
        }
      else
        {
          std::ifstream ifs(infile, std::ios::in);
          if (! ifs)
            {
              throw o3d3xx::error_t(O3D3XX_IO_ERROR);
            }

          json.assign((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));
        }

      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      cam->FromJSON(json);
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to configure camera from JSON config '"
                << infile << "':"
                << std::endl << e.what() << std::endl;
      return 1;
    }

  return 0;
}
