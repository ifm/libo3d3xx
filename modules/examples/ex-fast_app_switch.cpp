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

//
// ex-fast_app_switch.cpp
//
// Shows how to switch between two applications on the camera using PCIC, which
// should (theoretically) be fast. It also prints out some high-level latency
// metrics. Please NOTE: this is not a micro-benchmarking test suite, just a
// first order approximation of the expected latency.
//

#include <algorithm>
#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <o3d3xx_camera.h>
#include <o3d3xx_framegrabber.h>
#include <o3d3xx_image.h>
#include <o3d3xx_pcicclient.h>

template<std::size_t N, typename T>
double timeit(T func)
{
  std::vector<double> times;
  times.reserve(N);

  for (std::size_t i = 0; i < N; ++i)
    {
      auto start = std::chrono::steady_clock::now();
      func();
      auto stop = std::chrono::steady_clock::now();

      auto diff = std::chrono::duration<double>(stop - start).count();
      times.push_back(diff);
    }

  double median = 0.;
  std::sort(times.begin(), times.end());

  if (N > 0)
    {
      if (N % 2 == 0)
        {
          median = (times.at(N/2-1)+times.at(N/2))/2;
        }
      else
        {
          median = times.at(N/2);
        }
    }

  return median;
}


int main(int argc, const char **argv)
{
  o3d3xx::Logging::Init();
  constexpr std::size_t N = 20;

  std::string json_streaming =
    R"(
        {
          "o3d3xx":
          {
            "Device":
              {
                "ActiveApplication": "1"
              },
            "Apps":
            [
              {
                "Name": "23k",
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "FrameRate": "5",
                    "MinimumAmplitude": "42",
                    "Resolution": "0",
                    "SpatialFilterType": "0",
                    "SymmetryThreshold": "0",
                    "TemporalFilterType": "0",
                    "Type": "upto30m_moderate"
                }
              },
              {
                "Name": "100k",
                "TriggerMode": "1",
                "Index": "2",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "FrameRate": "5",
                    "MinimumAmplitude": "42",
                    "Resolution": "1",
                    "SpatialFilterType": "0",
                    "SymmetryThreshold": "0",
                    "TemporalFilterType": "0",
                    "Type": "upto30m_moderate"
                }
              }
            ]
          }
        }
      )";

  std::string json_swtrigger =
    R"(
        {
          "o3d3xx":
          {
            "Device":
              {
                "ActiveApplication": "1"
              },
            "Apps":
            [
              {
                "TriggerMode": "2",
                "Index": "1"
              },
              {
                "TriggerMode": "2",
                "Index": "2"
              }
            ]
          }
        }
      )";

  // instantiate the camera and set the configuration
  auto cam = std::make_shared<o3d3xx::Camera>();
  std::cout << "Setting camera configuration: " << std::endl
            << json_streaming << std::endl;
  cam->FromJSON(json_streaming);

  // create our image buffer to hold frame data from the camera
  auto img = std::make_shared<o3d3xx::ImageBuffer>();

  // instantiate our framegrabber
  auto fg = std::make_shared<o3d3xx::FrameGrabber>(cam);

  // instantiate our pcic interface
  auto pcic = std::make_shared<o3d3xx::PCICClient>(cam);

  //-----------------------------------------------------
  // OK, let's run some tests....
  //-----------------------------------------------------

  auto acquire_frame =
    [fg, img](cv::Mat& cloud, int resolution, bool sw = false) -> void
    {
      for (int i = 0; i < 5; ++i)
        {
          if (sw)
            {
              fg->SWTrigger();
            }

          if (! fg->WaitForFrame(img.get(), 1000))
            {
              std::cerr << "Timeout waiting for camera!" << std::endl;
              std::abort();
            }

          cloud = img->XYZImage();

          if (resolution == 23)
            {
              // 23k
              if ((cloud.rows == 132) && (cloud.cols == 176))
                {
                  return;
                }
            }
          else
            {
              // 100k
              if ((cloud.rows == 264) && (cloud.cols == 352))
                {
                  return;
                }
            }
        }

      std::cerr << "Acquired frame is of incorrect resolution!" << std::endl;
      std::abort();
    };

  auto assert_pcic_ok = [](std::string const& res) -> void
    {
      if (res != "*")
        {
          std::cerr << "PCIC switch failed!" << std::endl;
          std::abort();
        }
    };

  cv::Mat cloud;
  std::string res;

  //
  // Toggle back and forth N times (NOTE: 2xN switches per loop)
  // ... in streaming mode
  //
  std::cout << "OK, running streaming benchmarks..." << std::endl;
  std::cout << "Test 0: Streaming mode, toggling 23K <-> 100k "
            <<  N << "x (" << 2*N << " switches)" << std::endl;
  double switch0 =
    timeit<N>([&res, pcic, assert_pcic_ok, acquire_frame, &cloud]()
              {
                res = pcic->Call("a02");
                assert_pcic_ok(res);
                acquire_frame(cloud, 100);

                res = pcic->Call("a01");
                assert_pcic_ok(res);
                acquire_frame(cloud, 23);
              });

  //
  // Run same test as above but in s/w trigger mode
  //
  std::cout << "Setting camera configuration: " << std::endl
            << json_swtrigger << std::endl;
  cam->FromJSON(json_swtrigger);
  std::cout << "OK, running s/w trigger benchmarks..." << std::endl;

  std::cout << "Test 1: S/W trigger mode, toggling 23K <-> 100k "
            <<  N << "x (" << 2*N << " switches)" << std::endl;
  double switch1 =
    timeit<N>([&res, pcic, assert_pcic_ok, acquire_frame, &cloud]()
              {
                res = pcic->Call("a02");
                assert_pcic_ok(res);
                acquire_frame(cloud, 100, true);

                res = pcic->Call("a01");
                assert_pcic_ok(res);
                acquire_frame(cloud, 23, true);
              });

  //
  // Show our results
  //

  std::cout << std::endl
            << "*********************************************************"
            << std::endl << std::endl;

  std::cout << "Streaming mode, median exec time = " << switch0 << " secs ("
            << switch0 / 2. << " secs per switch + image acquisition)"
            << std::endl;

  std::cout << "S/W trigger mode, median exec time = " << switch1 << " secs ("
            << switch1 / 2. << " secs per switch + image acquisition)"
            << std::endl;

  std::cout << std::endl
            << "*********************************************************"
            << std::endl;

  return 0;
}
