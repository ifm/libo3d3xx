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

#include "o3d3xx_camera/cmdline_opts.h"
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "o3d3xx_camera/camera.hpp"
#include "o3d3xx_camera/version.h"

namespace po = boost::program_options;

o3d3xx::CmdLineOpts::CmdLineOpts(const std::string& description)
  : visible(description)
{
  // generic options
  po::options_description general("General");
  general.add_options()
    ("version,v", "Print version string and exit")
    ("help,h", "Produce help message");

  // options for connecting to the camera
  po::options_description connection_opts("Connection Information");
  connection_opts.add_options()
    ("ip",
     po::value<std::string>()->default_value(o3d3xx::DEFAULT_IP),
     "IP address of the camera")

    ("xmlrpc-port",
     po::value<std::uint32_t>()->default_value(o3d3xx::DEFAULT_XMLRPC_PORT),
     "XMLRPC port of the camera")

    ("password",
     po::value<std::string>()->default_value(o3d3xx::DEFAULT_PASSWORD),
     "Password for configuring the camera");

  this->visible.add(general).add(connection_opts);
}

int
o3d3xx::CmdLineOpts::Parse(int argc, const char **argv,
                           std::string *ip,
                           std::uint32_t *xmlrpc_port,
                           std::string *password,
                           std::function<void()> fn,
                           std::ostream& out)
{
  int retval = 1;
  int major, minor, patch;

  po::store(po::command_line_parser(argc, argv).
            options(this->visible).run(), this->vm);

  po::notify(this->vm);

  if (this->vm.count("help"))
    {
      out << this->visible << std::endl;
      retval = 0;
    }
  else if (this->vm.count("version"))
    {
      o3d3xx::version(&major, &minor, &patch);
      out << O3D3XX_LIBRARY_NAME
          << ": version=" << major << "."
          << minor << "." << patch << std::endl;
      retval = 0;
    }

  if (retval != 0)
    {
      if (ip != nullptr)
        {
          ip->assign(this->vm["ip"].as<std::string>());
        }

      if (xmlrpc_port != nullptr)
        {
          *xmlrpc_port = this->vm["xmlrpc-port"].as<std::uint32_t>();
        }

      if (password != nullptr)
        {
          password->assign(this->vm["password"].as<std::string>());
        }

      fn();
    }

  return retval;
}
