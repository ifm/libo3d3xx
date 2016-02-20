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

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include "o3d3xx_camera.h"

//
// Exports a camera configuration or just an application in the IFM-supported
// export format.
//

namespace po = boost::program_options;

int main(int argc, const char* argv[])
{
  int idx = -1;
  std::vector<std::uint8_t> bytes;

  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  std::string outfile;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx IFM Export");

      po::options_description export_opts("Export Information");
      export_opts.add_options()
        ("file,f", po::value<std::string>(), "Output file")
        ("app,a",
         "Flag indicating that the file to export is an application")
        ("config,c",
         "Flag indicating that the file to export is a camera configuration")
        ("index,i", po::value<int>(),
         "Index of the app to export (defaults to current active)");

      opts.visible.add(export_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      if (opts.vm.count("app") && opts.vm.count("config"))
        {
          std::cerr << "Only one of `app' or `config' may be supplied!"
                    << std::endl << std::flush;

          throw(o3d3xx::error_t(O3D3XX_INVALID_ARGUMENT));
        }
      else if (! (opts.vm.count("app") || opts.vm.count("config")))
        {
          std::cerr << "Exactly one of `app' or `config' must be supplied!"
                    << std::endl << std::flush;

          throw(o3d3xx::error_t(O3D3XX_INVALID_ARGUMENT));
        }

      outfile.assign(opts.vm["file"].as<std::string>());

      // get the byte buffer from the camera
      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      if (opts.vm.count("app"))
        {
          // export an app
          if (opts.vm.count("index"))
            {
              idx = opts.vm["index"].as<int>();
            }

          if (idx == -1)
            {
              idx = cam->GetDeviceConfig()->ActiveApplication();
            }

          bytes = cam->ExportIFMApp(idx);
        }
      else
        {
          // export a configuration
          std::cerr << "Config export: not yet implemented!"
                    << std::endl;
        }

      // write the bytes out to the supplied file
      std::ofstream(outfile, std::ios::binary).
        write((char *)bytes.data(), bytes.size());
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to export to file '"
                << outfile << "':"
                << std::endl << e.what() << std::endl;

      return 1;
    }

  return 0;
}
