// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm robotics
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
// ex-pcic_dos_attack
//
// Demonstrates behavior of the framegrabber when all allowable PCIC
// connections have be exhausted.
//

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/core/core.hpp>
#include <o3d3xx_camera.h>
#include <o3d3xx_framegrabber.h>
#include <o3d3xx_image.h>
#include <o3d3xx_pcicclient.h>

static int N = 10;
static bool RUN = true;

void worker_func()
{
  auto id = std::this_thread::get_id();
  std::cout << "New worker thread: " << id << std::endl;

  auto cam = std::make_shared<o3d3xx::Camera>();
  auto fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
  auto buff = std::make_shared<o3d3xx::ByteBuffer>();

  while (RUN)
    {
      if (! fg->WaitForFrame(buff.get(), 1000))
        {
          std::cout << id << " timeout!" << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
  o3d3xx::Logging::Init();

  // Ctrl-C to exit the program
  std::signal(SIGINT,
              [](int sig)
              {
                std::cout << "Bye" << std::endl;
                std::exit(0);
              });

  auto id = std::this_thread::get_id();
  std::cout << "Main thread id: " << id << std::endl;

  // create a frame grabber ... implicitly connects to PCIC
  auto cam = std::make_shared<o3d3xx::Camera>();
  auto fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
  auto buff = std::make_shared<o3d3xx::ByteBuffer>();

  // create a bunch of workers that try to fetch frames from the camera
  // ... here we try to create N more pcic connections whereby we assume
  // N is a DOS attack on pcic (i.e., more than the allowed number of
  // connections).
  std::vector<std::thread> workers;
  for (int i = 0; i < N; ++i)
    {
      workers.push_back(std::thread{worker_func});
    }

  // let the workers run for a while
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // tell all the workers to stop
  RUN = false;
  std::cout << "Waiting for all workers..." << std::endl;
  for (auto& t : workers)
    {
      t.join();
    }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Now, try to get data from PCIC with our original fg
  std::cout << "Fetching 10 more frames..." << std::endl;
  for (int i = 0; i < 10; ++i)
    {
      if (! fg->WaitForFrame(buff.get(), 1000))
        {
          std::cout << id << " timeout in main thread!" << std::endl;
        }
    }
  std::cout << "OK" << std::endl;

  return 0;
}
