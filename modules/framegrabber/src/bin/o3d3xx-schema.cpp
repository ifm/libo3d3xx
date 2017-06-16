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
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

//
// Generates PCIC schemas based ona mask and prints them to the screen for
// visual inspection
//

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  uint32_t xmlrpc_port;
  std::string password;
  std::uint16_t mask = o3d3xx::DEFAULT_SCHEMA_MASK;
  std::string mask_str;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx Schema");

      po::options_description schema_opts("Schema Information");
      schema_opts.add_options()
        ("mask,m",
         po::value<std::uint16_t>()->default_value(mask),
         "`mask' used to generate schema")

        ("str,s",
         po::value<std::string>()->default_value("-"),
         "Mask string: e.g., 'IMG_AMP|IMG_CART'")

        ("dump,d",
         "Dump masking options and exit");

      opts.visible.add(schema_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      if (opts.vm.count("dump"))
        {
          std::cout << "Masking options:" << std::endl
                    << '\t' << "IMG_RDIS: "
                    << (int) o3d3xx::IMG_RDIS << std::endl
                    << '\t' << "IMG_AMP:  "
                    << (int) o3d3xx::IMG_AMP << std::endl
                    << '\t' << "IMG_RAMP: "
                    << (int) o3d3xx::IMG_RAMP << std::endl
                    << '\t' << "IMG_CART: "
                    << (int) o3d3xx::IMG_CART << std::endl
                    << '\t' << "IMG_UVEC: "
                    << (int) o3d3xx::IMG_UVEC << std::endl
                    << '\t' << "EXP_TIME: "
                    << (int) o3d3xx::EXP_TIME << std::endl
	            << '\t' << "ILLU_TEMP: "
		    << (int) o3d3xx::ILLU_TEMP << std::endl;
          return 0;
        }

      mask_str.assign(opts.vm["str"].as<std::string>());
      mask = opts.vm["mask"].as<std::uint16_t>();

      if (mask_str != "-")
        {
          mask = o3d3xx::schema_mask_from_string(mask_str);
        }

      std::cout << "mask=" << (int) mask
                << ", str=" << mask_str
                << std::endl << "---" << std::endl
                << o3d3xx::make_pcic_schema(mask)
                << std::endl;
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to generate schema: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}
