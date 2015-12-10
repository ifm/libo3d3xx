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

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "o3d3xx.h"

//-------------------------------------------------------------
// Quick and dirty viewer application to visualize the various libo3d3xx images
// -- leverages the built-in PCL and OpenCV visualization infrastructure.
//-------------------------------------------------------------

class O3DViewer
{
public:
  O3DViewer(o3d3xx::Camera::Ptr cam, const std::string& descr)
    : cam_(cam),
      description_(descr)
  {}

  void Toggle100kMode()
  {
	o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
    cam->RequestSession();
    cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

    o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
    int idx = dev->ActiveApplication();

    // toggle the 100K output support
    cam->EditApplication(idx);
    o3d3xx::ImagerConfig::Ptr im = cam->GetImagerConfig();
    im->SetOutput100K(!im->Output100K());
    cam->SetImagerConfig(im.get());
    cam->SaveApp();
    cam->CancelSession();
  }

  void SaveDepthImage(o3d3xx::ImageBuffer::Ptr buff)
  {
	  cv::imwrite("./Image_Depth.png", buff->DepthImage());
  }

  void SaveRawAmplitudeImage(o3d3xx::ImageBuffer::Ptr buff)
  {
	  cv::imwrite("./Image_RawAmplitude.png", buff->RawAmplitudeImage());
  }

  void SaveNormalizedAmplitudeImage(o3d3xx::ImageBuffer::Ptr buff)
  {
	  cv::imwrite("./Image_NormalizedAmplitude.png", buff->AmplitudeImage());
  }

  void SaveConfidenceImage(o3d3xx::ImageBuffer::Ptr buff)
  {
	  cv::imwrite("./Image_Confidence.png", buff->ConfidenceImage());
  }

  void SavePointCloud(o3d3xx::ImageBuffer::Ptr buff)
  {
	  pcl::io::savePCDFile("./Image_PointCloud.pcd", *buff->Cloud());
  }

  void Run()
  {
    int win_w = 4*352;
    int win_h = 4*264;

    o3d3xx::FrameGrabber::Ptr fg =
      std::make_shared<o3d3xx::FrameGrabber>(this->cam_);

    o3d3xx::ImageBuffer::Ptr buff =
      std::make_shared<o3d3xx::ImageBuffer>();

    //
    // Setup for point cloud
    //
    std::shared_ptr<pcl::visualization::PCLVisualizer> pclvis_ =
      std::make_shared<pcl::visualization::PCLVisualizer>(this->description_);
    pclvis_->setBackgroundColor(0, 0, 0);
    pclvis_->setSize(win_w, win_h);
    pclvis_->setCameraPosition(-3.0, // x-position
                               0,    // y-position
                               0,    // z-position
                               0,    // x-axis "up" (0 = false)
                               0,    // y-axis "up" (0 = false)
                               1);   // z-axis "up" (1 = true)

    // use "A" and "a" to toggle axes indicators
    pclvis_->registerKeyboardCallback(
     [&](const pcl::visualization::KeyboardEvent& ev)
      {
        if (ev.getKeySym() == "A" && ev.keyDown())
          {
            pclvis_->addCoordinateSystem();
          }
        else if (ev.getKeySym() == "a" && ev.keyDown())
          {
            pclvis_->removeCoordinateSystem();
          }
      });


    //
    // Setup for amplitude, depth, and confidence images
    //

    // used for all 2d images
    cv::namedWindow(this->description_, cv::WINDOW_NORMAL|cv::WINDOW_OPENGL);
    cv::resizeWindow(this->description_, win_w, win_h);
    int retval;
    double min, max;
    cv::Rect roi;

    // depth
    cv::Mat display_img;
    cv::Mat depth_colormap_img;

    // confidence
    cv::Mat conf_img;
    cv::Mat conf_colormap_img;

    // amplitude
    cv::Mat amp_colormap_img;
    cv::Mat raw_amp_colormap_img;

    bool is_first = true;
    while (! pclvis_->wasStopped())
      {
        pclvis_->spinOnce(100);
        if (! fg->WaitForFrame(buff.get(), 500))
          {
            continue;
          }

        //------------
        // Point cloud
        //------------
        pcl::visualization::PointCloudColorHandlerGenericField<o3d3xx::PointT>
          color_handler(buff->Cloud(), "intensity");

        if (is_first)
          {
            is_first = false;
            pclvis_->addPointCloud(buff->Cloud(), color_handler, "cloud");
          }
        else
          {
            pclvis_->updatePointCloud(buff->Cloud(), color_handler, "cloud");
          }

        //------------
        // 2D images
        //------------

        // depth image
        cv::minMaxIdx(buff->DepthImage(), &min, &max);
        cv::convertScaleAbs(buff->DepthImage(),
                            depth_colormap_img, 255 / max);
        cv::applyColorMap(depth_colormap_img, depth_colormap_img,
                          cv::COLORMAP_JET);

        // confidence image: show as binary image of good pixel vs. bad pixel.
        conf_img = buff->ConfidenceImage();
        conf_colormap_img = cv::Mat::ones(conf_img.rows,
                                          conf_img.cols,
                                          CV_8UC1);
        cv::bitwise_and(conf_img, conf_colormap_img,
                        conf_colormap_img);
        cv::convertScaleAbs(conf_colormap_img,
                            conf_colormap_img, 255);
        cv::applyColorMap(conf_colormap_img, conf_colormap_img,
                          cv::COLORMAP_SUMMER);

        // amplitude
        cv::minMaxIdx(buff->AmplitudeImage(), &min, &max);
        cv::convertScaleAbs(buff->AmplitudeImage(),
                            amp_colormap_img, 255 / max);
        cv::applyColorMap(amp_colormap_img, amp_colormap_img,
                          cv::COLORMAP_BONE);

        cv::minMaxIdx(buff->RawAmplitudeImage(), &min, &max);
        cv::convertScaleAbs(buff->RawAmplitudeImage(),
                            raw_amp_colormap_img, 255 / max);
        cv::applyColorMap(raw_amp_colormap_img, raw_amp_colormap_img,
                          cv::COLORMAP_BONE);

        // stitch 2d images together and display
        display_img.create(depth_colormap_img.rows*2,
                           depth_colormap_img.cols*2,
                           depth_colormap_img.type());

        roi = cv::Rect(0, 0,
                       depth_colormap_img.cols,
                       depth_colormap_img.rows);
        depth_colormap_img.copyTo(display_img(roi));

        roi = cv::Rect(depth_colormap_img.cols, 0,
                       conf_colormap_img.cols,
                       conf_colormap_img.rows);
        conf_colormap_img.copyTo(display_img(roi));

        roi = cv::Rect(0, depth_colormap_img.rows,
                       amp_colormap_img.cols,
                       amp_colormap_img.rows);
        amp_colormap_img.copyTo(display_img(roi));

        roi = cv::Rect(depth_colormap_img.cols,
                       depth_colormap_img.rows,
                       raw_amp_colormap_img.cols,
                       raw_amp_colormap_img.rows);
        raw_amp_colormap_img.copyTo(display_img(roi));

        cv::imshow(this->description_, display_img);

        // `ESC', `q', or `Q' to exit
        retval = cv::waitKey(33) & 0xff;
        if ((retval == 'q') || (retval == 'Q') || (retval == 27))
          {
        	printf( "Quit\n" );
            break;
          }
        else if (retval == '1')
          {
        	Toggle100kMode();
          }
        else if (retval == 'd')
          {
        	SaveDepthImage(buff);
          }
        else if (retval == 'r')
          {
        	SaveRawAmplitudeImage(buff);
          }
        else if (retval == 'n')
          {
        	SaveNormalizedAmplitudeImage(buff);
          }
        else if (retval == 'c')
          {
        	SaveConfidenceImage(buff);
          }
        else if (retval == 'p')
          {
        	SavePointCloud(buff);
          }
        else if (retval == 's')
          {
        	cout << "Save all image types" << endl;
        	SaveDepthImage(buff);
        	SaveRawAmplitudeImage(buff);
        	SaveNormalizedAmplitudeImage(buff);
        	SaveConfidenceImage(buff);
        	SavePointCloud(buff);
          }

      } // end: while (...)
  }

private:
  o3d3xx::Camera::Ptr cam_;
  std::string description_;

}; // end: class O3DViewer

//-------------------------------------------------------------
// Go...
//-------------------------------------------------------------

int main(int argc, const char **argv)
{
  int retval = 0;

  std::string camera_ip;
  uint32_t xmlrpc_port;
  std::string password;
  std::string descr("o3d3xx Viewer");

  try
    {
      //---------------------------------------------------
      // Handle command-line arguments
      //---------------------------------------------------
      o3d3xx::CmdLineOpts opts(descr);
      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port, &password))
        {
          return 0;
        }

      //---------------------------------------------------
      // Initialize the camera
      //---------------------------------------------------
      o3d3xx::Camera::Ptr cam =
        std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

      //---------------------------------------------------
      // Run the viewer
      //---------------------------------------------------
      O3DViewer viewer(cam, descr);
      viewer.Run();
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return 1;
    }

  return retval;
}
