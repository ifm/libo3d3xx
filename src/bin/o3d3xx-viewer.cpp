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

//-------------------------------------------------------------
// Quick and dirty viewer application to visualize the various libo3d3xx images
//-------------------------------------------------------------

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "o3d3xx.h"

//-----------------------------------------------------------
// Render point clouds on a separate thread
//-----------------------------------------------------------
class O3DCloudViewer
{
public:
  O3DCloudViewer(o3d3xx::FrameGrabber::Ptr fg, const std::string& descr)
    : fg_(fg),
      description_(descr),
      thread_(new std::thread(std::bind(&O3DCloudViewer::Run, this)))
  {}

  void Join()
  {
    return this->thread_->join();
  }

  void Run()
  {
    pcl::visualization::CloudViewer viewer(this->description_);
    o3d3xx::ImageBuffer::Ptr buff(new o3d3xx::ImageBuffer());

    while (! viewer.wasStopped())
      {
	if (! this->fg_->WaitForFrame(buff.get(), 1000, true))
	  {
	    continue;
	  }

	viewer.showCloud(buff->Cloud());
      }
  }

private:
  o3d3xx::FrameGrabber::Ptr fg_;
  std::string description_;
  std::unique_ptr<std::thread> thread_;

}; // end: O3DCloudViewer

//-----------------------------------------------------------
// Render opencv images on a separate thread
//-----------------------------------------------------------
class O3DMatViewer
{
public:
  static const int DEPTH = 1;
  static const int AMP = 2;
  static const int CONF = 3;

  O3DMatViewer(o3d3xx::FrameGrabber::Ptr fg,
	       const std::string& descr,
	       int image_type)
    : fg_(fg),
      description_(descr),
      image_type_(image_type),
      thread_(new std::thread(std::bind(&O3DMatViewer::Run, this)))
  {}

  void Join()
  {
    return this->thread_->join();
  }

  void Run()
  {
    cv::Mat colormap_img;
    cv::namedWindow(this->description_, cv::WINDOW_NORMAL);
    double min, max;

    int retval = 0;
    while (true)
      {
	o3d3xx::ImageBuffer::Ptr buff(new o3d3xx::ImageBuffer());

	if (this->fg_->WaitForFrame(buff.get(), 1000, true))
	  {
	    switch (this->image_type_)
	      {
	      case O3DMatViewer::DEPTH:
		cv::minMaxIdx(buff->DepthImage(), &min, &max);
		cv::convertScaleAbs(buff->DepthImage(),
				    colormap_img, 255.0 / max);
		cv::applyColorMap(colormap_img, colormap_img,
				  cv::COLORMAP_JET);
		cv::imshow(this->description_, colormap_img);
		break;

	      case O3DMatViewer::AMP:
		cv::minMaxIdx(buff->AmplitudeImage(), &min, &max);
		cv::convertScaleAbs(buff->AmplitudeImage(),
				    colormap_img, 255.0 / max);
		cv::applyColorMap(colormap_img, colormap_img,
				  cv::COLORMAP_JET);
		cv::imshow(this->description_, colormap_img);
		break;

	      case O3DMatViewer::CONF:
		// view confidence image as a binary
		// image showing good pixel vs. bad pixel
		cv::Mat conf_img = buff->ConfidenceImage();
		colormap_img = cv::Mat::ones(conf_img.rows,
					     conf_img.cols,
					     CV_8UC1);
		cv::bitwise_and(conf_img, colormap_img,
				colormap_img);
		cv::convertScaleAbs(colormap_img,
		 		    colormap_img, 255);
		cv::applyColorMap(colormap_img, colormap_img,
				  cv::COLORMAP_JET);
		cv::imshow(this->description_, colormap_img);
		break;
	      }
	  }

	// `ESC', `q', or `Q' to exit
	retval = cv::waitKey(33);
	if ((retval == 27) || (retval == 113) || (retval == 81))
	  {
	    break;
	  }
      }
  }

private:
  o3d3xx::FrameGrabber::Ptr fg_;
  std::string description_;
  int image_type_;
  std::unique_ptr<std::thread> thread_;

}; // end: O3DMatViewer

namespace po = boost::program_options;

int main(int argc, const char **argv)
{
  int major, minor, patch;

  std::string camera_ip(o3d3xx::DEFAULT_IP);
  uint32_t xmlrpc_port = o3d3xx::DEFAULT_XMLRPC_PORT;

  try
    {
      //---------------------------------------------------
      // Handle command-line arguments
      //---------------------------------------------------
      o3d3xx::CmdLineOpts opts("o3d3xx Viewer");

      po::options_description viewer_opts("Viewer Information");
      viewer_opts.add_options()
	("cloud", "Visualize Point Cloud")
	("depth", "Visualize Depth Image")
	("amp", "Visualize Amplitude Image")
	("conf", "Visualize Confidence Image");

      opts.visible.add(viewer_opts);
      opts.Parse(argc, argv);

      if (opts.vm.count("help"))
	{
	  std::cout << opts.visible
		    << std::endl;
	  return 0;
	}

      if (opts.vm.count("version"))
	{
	  o3d3xx::version(&major, &minor, &patch);
	  std::cout << "Version=" << major << "."
		    << minor << "." << patch << std::endl;
	  return 0;
	}

      //---------------------------------------------------
      // Initialize o3d3xx logging facilities
      //---------------------------------------------------
      FLAGS_logbuflevel = -1;
      o3d3xx::Logging::Init();
      google::SetStderrLogging(google::FATAL);

      //---------------------------------------------------
      // Initialize the camera and frame grabber
      //---------------------------------------------------
      camera_ip.assign(opts.vm["ip"].as<std::string>());
      xmlrpc_port = opts.vm["xmlrpc-port"].as<std::uint32_t>();
      o3d3xx::Camera::Ptr cam(new o3d3xx::Camera(camera_ip, xmlrpc_port));
      o3d3xx::FrameGrabber::Ptr fg(new o3d3xx::FrameGrabber(cam));

      //---------------------------------------------------
      // Run the requested image renderer
      //---------------------------------------------------
      if (opts.vm.count("depth"))
	{
	  O3DMatViewer depth_viewer(fg, "o3d3xx Depth Viewer",
				    O3DMatViewer::DEPTH);
	  depth_viewer.Join();
	}
      else if (opts.vm.count("amp"))
	{
	  O3DMatViewer amp_viewer(fg, "o3d3xx Amplitude Viewer",
				  O3DMatViewer::AMP);
	  amp_viewer.Join();
	}
      else if (opts.vm.count("conf"))
	{
	  O3DMatViewer conf_viewer(fg, "o3d3xx Confidence Viewer",
				  O3DMatViewer::CONF);
	  conf_viewer.Join();
	}
      else
	{
	  O3DCloudViewer cloud_viewer(fg, "o3d3xx Cloud Viewer");
	  cloud_viewer.Join();
	}
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return 1;
    }

  return 0;
}
