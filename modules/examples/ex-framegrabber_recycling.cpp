/*
 * Copyright (C) 2016 Love Park Robotics, LLC
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
// ex-framegrabber_recycling.cpp
//
// This example is in response to:
//
// https://github.com/lovepark/libo3d3xx/issues/45
//
// It demonstrates how to make your framegrabbing robust to the camera
// "disappearing" e.g., due to power cycling, lost network, etc.
//

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"

int main(int argc, const char **argv)
{
  o3d3xx::Logging::Init();

  // Ctrl-C to exit the program
  std::signal(SIGINT,
              [](int sig)
              {
                std::cout << "Bye" << std::endl;
                std::exit(0);
              });

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
  o3d3xx::ByteBuffer::Ptr buff = std::make_shared<o3d3xx::ByteBuffer>();

  double thresh = 3.0;
  auto last_frame = std::chrono::steady_clock::now();

  while (true)
    {
      if (! fg->WaitForFrame(buff.get(), 1000))
        {
          std::cout << "Timeout waiting for camera!" << std::endl;

          auto now = std::chrono::steady_clock::now();
          auto diff = now - last_frame;
          if (std::chrono::duration_cast<std::chrono::seconds>(diff).count() >
              thresh)
            {
              std::cout << "Restarting FrameGrabber..." << std::endl;
              fg.reset(new o3d3xx::FrameGrabber(cam));
            }

          continue;
        }

      last_frame = std::chrono::steady_clock::now();
      std::cout << "Got frame!" << std::endl;
    }

  return 0;
}
