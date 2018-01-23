/*
 * Copyright (C) 2018 ifm syntron gmgh
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
#include "o3d3xx_camera.h"

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  auto limit = 0;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx trace logs");
      po::options_description trace_opts("Limit the amount of trace messages");
      trace_opts.add_options()
        ("limit,l",
         po::value<int>(),
         "Limit the amount of trace log messages printed. If not provided all are printed");
       
      opts.visible.add(trace_opts);
      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }
      if (opts.vm.count("limit"))
        {
          limit = std::max(1,opts.vm["limit"].as<int>()); 
        }

      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      std::vector<std::string> logs = cam->GetTraceLogs(limit);

      for (auto& log : logs )
        {
          std::cout << log 
                    << std::endl
                    << std::flush;
        }

    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to get trace logs: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}
