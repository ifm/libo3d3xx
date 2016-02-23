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
// ex-file_io.cpp
//
// Capture a frame from the camera, and write the data out to files. For
// exemplary purposes, we will write the amplitdue and radial distance images
// to PNG files and the point cloud to a PCD file.
//

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

int main(int argc, const char **argv)
{
  o3d3xx::Logging::Init();

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(
      cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);

  if (! fg->WaitForFrame(img.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

  pcl::io::savePCDFileASCII("point_cloud.pcd", *(img->Cloud()));
  imwrite("amplitude.png", img->AmplitudeImage());
  imwrite("radial_distance.png", img->DepthImage());

  return 0;
}
