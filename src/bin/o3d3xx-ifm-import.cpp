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
#include "o3d3xx.h"

//
// Imports a camera configuration or just an application based upon the IFM
// zip-compressed JSON file(s)
//

namespace po = boost::program_options;

int main(int argc, const char* argv[])
{
  int idx = -1;

  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  std::string infile;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx IFM Import");

      po::options_description import_opts("Import Information");
      import_opts.add_options()
        ("file,f", po::value<std::string>(), "Input IFM file")
        ("app,a",
         "Flag indicating that the file to import is an application")
        ("config,c",
         "Flag indicating that the file to import is a camera configuration");

      opts.visible.add(import_opts);

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

      infile.assign(opts.vm["file"].as<std::string>());
      std::ifstream ifs(infile, std::ios::in|std::ios::binary);
      if (! ifs)
        {
          if (opts.vm.count("file") == 0)
            {
              std::cerr << "The `file' option is required!"
                        << std::endl << std::flush;
            }

          throw o3d3xx::error_t(O3D3XX_IO_ERROR);
        }

      // read the file into a byte buffer
      ifs.unsetf(std::ios::skipws);
      std::streampos file_size;
      ifs.seekg(0, std::ios::end);
      file_size = ifs.tellg();
      ifs.seekg(0, std::ios::beg);

      std::vector<std::uint8_t> bytes;
      bytes.reserve(file_size);

      bytes.insert(bytes.begin(),
                   std::istream_iterator<std::uint8_t>(ifs),
                   std::istream_iterator<std::uint8_t>());

      // send the byte buffer to the camera for import
      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      if (opts.vm.count("app"))
        {
          // import an app
          idx = cam->ImportIFMApp(bytes);
          std::cout << "New application created at index: "
                    << idx << std::endl << std::flush;
        }
      else
        {
          // import a "configuration"
          std::cerr << "Config import: not yet implemented!"
                    << std::endl;
        }
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to import file '"
                << infile << "':"
                << std::endl << e.what() << std::endl;
      return 1;
    }

  return 0;
}
