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
// ex-cartesian.cpp
//
// Computes the cartesian data from unit vectors, extrinsics, and
// radial depth image.
//

#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

int main(int argc, const char **argv)
{
  o3d3xx::Logging::Init();

  // Get access to the camera and an image buffer to hold and organize the data
  // from the camera
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  // Initialize the framegrabber, with a schema that only streams in the unit
  // vectors and get the unit vectors. We only need to get them once.
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_UVEC);
  if (! fg->WaitForFrame(img.get(), 1000))
    {
      std::cerr << "Failed to get unit vectors." << std::endl;
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

  // rotated unit vectors
  cv::Mat ex, ey, ez;
  std::vector<cv::Mat> uvec_channels(3);
  cv::split(img->UnitVectors(), uvec_channels);
  ex = uvec_channels[0];
  ey = uvec_channels[1];
  ez = uvec_channels[2];

  // Reinitialize the framegrabber with a schema that streams only the radial
  // distance image. NOTE: The extrinsics will be available as part of this
  // schema as they are an invariant.
  fg.reset(new o3d3xx::FrameGrabber(cam, o3d3xx::IMG_RDIS));

  // Typically, the rest of this would be in a loop, but we are only going to
  // compute the cartesian data once.
  if (! fg->WaitForFrame(img.get(), 1000))
    {
      std::cerr << "Failed to get radial distance image." << std::endl;
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

  // hold a reference to the radial distance image and convert to float
  cv::Mat rdis = img->DepthImage();
  cv::Mat rdis_f;
  rdis.convertTo(rdis_f, CV_32FC1);

  // get a copy of the extrinsics
  std::vector<float> extrinsics = img->Extrinsics();

  // NOTE: The unit vectors are already rotated, so, we only need to extract
  // out the translation vector. The units are in mm.
  float tx = extrinsics[0];
  float ty = extrinsics[1];
  float tz = extrinsics[2];

  // Compute the cartesian data
  cv::Mat x_f = ex.mul(rdis_f) + tx;
  cv::Mat y_f = ey.mul(rdis_f) + ty;
  cv::Mat z_f = ez.mul(rdis_f) + tz;

  // cast to uint16_t
  cv::Mat x_i, y_i, z_i;
  x_f.convertTo(x_i, CV_16SC1);
  y_f.convertTo(y_i, CV_16SC1);
  z_f.convertTo(z_i, CV_16SC1);

  // explicitly set to zero any bad pixels
  cv::Mat mask;
  cv::Mat mask_ = rdis != 0;
  mask_.convertTo(mask, CV_16SC1);
  mask /= 255;
  x_i = x_i.mul(mask);
  y_i = y_i.mul(mask);
  z_i = z_i.mul(mask);

  // Transform from the IFM/O3D camera frame to the libo3d3xx/ROS camera frame
  // NOTE: x, y, and z are now in mm.
  cv::Mat x = z_i;
  cv::Mat y = -x_i;
  cv::Mat z = -y_i;

  // spew the numbers out to the screen
  std::cout << x << std::endl
            << y << std::endl
            << z << std::endl;

  return 0;
}
