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

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

//
// Measures how fast the frame grabber is running
//

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  int nframes;
  int nruns;
  bool sw_trigger;
  bool organize;
  double mean = 0.0;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx Frame Grabber Speed");

      po::options_description hz_opts("Hz");
      hz_opts.add_options()
        ("nframes,f",
         po::value<int>()->default_value(10),
         "Number of frames to capture")

        ("nruns,r",
         po::value<int>()->default_value(1),
         "Number of runs to average")

        ("sw,s",
         po::value<bool>()->default_value(false),
         "Software Trigger the FrameGrabber")

        ("organize,o",
         po::value<bool>()->default_value(false),
         "Calls the Organize() hook on the ByteBuffer interface (DEPRECATED)");

      opts.visible.add(hz_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      nframes = opts.vm["nframes"].as<int>();
      nruns = opts.vm["nruns"].as<int>();
      sw_trigger = opts.vm["sw"].as<bool>();
      organize = opts.vm["organize"].as<bool>();

      if (nframes <= 0)
        {
          nframes = 10;
        }

      std::vector<double> stats;

      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      o3d3xx::FrameGrabber::Ptr fg =
        std::make_shared<o3d3xx::FrameGrabber>(cam);

      o3d3xx::ByteBuffer::Ptr buff =
        std::make_shared<o3d3xx::ByteBuffer>();

      for (int i = 0; i < nruns; i++)
        {
          auto start = std::chrono::steady_clock::now();
          for (int j = 0; j < nframes; j++)
            {
              if (sw_trigger)
                {
                  fg->SWTrigger();
                }

              if (! fg->WaitForFrame(buff.get(), 1000, false, organize))
                {
                  std::cerr << "Timeout waiting for camera!"
                            << std::endl;
                  return -1;
                }
            }
          auto stop = std::chrono::steady_clock::now();
          auto diff = stop - start;
          stats.push_back(std::chrono::duration<double>(diff).count());
        }

      if (nruns >= 1)
        {
          mean =
            std::accumulate<std::vector<double>::iterator,double>(
              stats.begin(), stats.end(), 0.0) / (double) nruns;

          std::cout << "FrameGrabber running at: "
                    << nframes / mean << " Hz"
                    << std::endl
                    << nframes << " frames captured; averaged over "
                    << nruns << " runs"
                    << std::endl;
        }
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to compute frame grabber speed: "
                << e.what() << std::endl;
      return 1;
    }

  return 0;
}
