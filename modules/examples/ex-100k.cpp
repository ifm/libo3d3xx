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
// ex-100k.cpp
//
// Shows how to use the API to create a new application, set its resolution to
// 100K, and stream data from the camera in that mode. We note that you really
// should use `o3d3xx-dump` and `o3d3xx-config` to create an application that
// employs the 100k imager, set it as active, and simply stream data from it
// rather than using the API to manipulate the applications on the
// camera. However, we have gotten enough requests for this, that we are going
// to show how to do it.
//

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

int main(int argc, const char **argv)
{
  o3d3xx::Logging::Init();

  //
  // setup
  //
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

  o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
  int old_active_idx = dev->ActiveApplication();
  int idx = cam->CopyApplication(old_active_idx);
  cam->EditApplication(idx);
  o3d3xx::ImagerConfig::Ptr im = cam->GetImagerConfig();
  im->SetResolution(o3d3xx::RES_100K);
  cam->SetImagerConfig(im.get());
  cam->SaveApp();

  dev->SetActiveApplication(idx);
  cam->SetDeviceConfig(dev.get());
  cam->SaveDevice();
  cam->CancelSession();

  //
  // stream data -- we will grab 5 images then exit
  //
  o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();

  pcl::PointCloud<o3d3xx::PointT>::Ptr cloud;
  cv::Mat amplitude;

  for (int i = 0; i < 5; ++i)
    {
      if (! fg->WaitForFrame(img.get(), 1000))
        {
          std::cerr << "Timeout waiting for camera!" << std::endl;
          continue;
        }

      cloud = img->Cloud();
      amplitude = img->AmplitudeImage();

      // print out dims as rows x cols
      std::cout << "Cloud dimensions: "
                << cloud->height << "x" << cloud->width << std::endl;
      std::cout << "Amplitude dimensions: "
                << amplitude.size().height << "x"
                << amplitude.size().width << std::endl << std::endl;
    }

  //
  // clean up
  //
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  dev->SetActiveApplication(old_active_idx);
  cam->SetDeviceConfig(dev.get());
  cam->SaveDevice();
  cam->DeleteApplication(idx);

  return 0;
}
