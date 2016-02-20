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

#include <iostream>
#include "o3d3xx_camera.h"

int main(int argc, const char **argv)
{
  int major;
  int minor;
  int patch;

  o3d3xx::Logging::Init();

  o3d3xx::version(&major, &minor, &patch);
  std::cout << O3D3XX_LIBRARY_NAME
            << ": version=" << major << "."
            << minor << "." << patch << std::endl;

  return 0;
}
