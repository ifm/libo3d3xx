/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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
#include <fstream>
#include <iostream>
#include <memory>
#include <ratio>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <o3d3xx_camera.h>
#include <o3d3xx_oem.h>
#include <opencv2/opencv.hpp>

//
// Quantify framegrabber jitter
//

namespace po = boost::program_options;

template<typename T>
T median2d(const cv::Mat& arr)
{
  T median = 0;
  std::size_t sz = arr.rows*arr.cols;

  std::vector<T> arr_cp(sz);
  std::copy(arr.begin<T>(), arr.end<T>(), arr_cp.begin());
  std::sort(arr_cp.begin(), arr_cp.end());

  if (sz > 0)
    {
      if (sz % 2 == 0)
        {
          median =
            (arr_cp.at(sz/2-1)+arr_cp.at(sz/2))/2.;
        }
      else
        {
          median = arr_cp.at(sz/2);
        }
    }

  return median;
}

template<typename T>
T mad(const cv::Mat& arr, T center)
{
  cv::Mat arr_d;
  cv::absdiff(arr, center, arr_d);
  return median2d<T>(arr_d);
}

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;
  std::string password;
  int nframes;
  std::string outfile;
  cv::Mat arr;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx OEM Frame Grabber Jitter");

      po::options_description jitter_opts("Jitter");
      jitter_opts.add_options()
        ("nframes,f",
         po::value<int>()->default_value(100),
         "Number of frames to capture")

        ("outfile,o",
         po::value<std::string>()->default_value("-"),
         "Raw data output file, if not specified, nothing is written");

      opts.visible.add(jitter_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      outfile = opts.vm["outfile"].as<std::string>();
      nframes = opts.vm["nframes"].as<int>();
      nframes = nframes <= 0 ? 100 : nframes;

      auto cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);
      auto fg = std::make_shared<o3d3xx::oem::FrameGrabber>(cam);
      auto im = std::make_shared<o3d3xx::oem::ImageBuffer>();
      // get the one-time allocations out of the way, and,
      // make sure it doesn't get optimized away by the compiler
      if (! fg->WaitForFrame(im.get(), 1000))
        {
          std::cerr << "Timeout waiting for first image acquisition!"
                    << std::endl;
          return 1;
        }

      // holds our timing results (millis)
      arr.create(nframes, 1, CV_32FC1);

      for (int i = 0; i < nframes; ++i)
        {
          auto t1 = std::chrono::high_resolution_clock::now();
          if (! fg->WaitForFrame(im.get(), 1000))
            {
              std::cerr << "Timeout waiting for image acquisition!"
                        << std::endl;
              return 1;
            }
          auto t2 = std::chrono::high_resolution_clock::now();

          std::chrono::duration<float, std::milli> fp_ms = t2 - t1;
          arr.at<float>(i, 0) = fp_ms.count();
        }

      cv::Scalar mean, stddev;
      float median = median2d<float>(arr);
      cv::meanStdDev(arr, mean, stddev);
      std::cout << "Mean:   " << mean[0] << " ms" << std::endl;
      std::cout << "Median: " <<  median << " ms" << std::endl;
      std::cout << "Stddev: " << stddev[0] << " ms" << std::endl;
      std::cout << "MAD:    " << mad(arr, median) << " ms" << std::endl;

      if (outfile != "-")
        {
          std::ofstream out;
          out.open(outfile);
          out << cv::format(arr, cv::Formatter::FMT_CSV);
          out.close();
          std::cout << "Raw data has been written to: " << outfile << std::endl;
        }
    }
  catch (const std::exception& ex)
    {
      std::cerr << "Failed to compute jitter statistics: "
                << ex.what() << std::endl;
      return 1;
    }

  return 0;
}
