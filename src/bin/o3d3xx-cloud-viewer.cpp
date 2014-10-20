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
#include <string>
#include <glog/logging.h>
#include <pcl/visualization/cloud_viewer.h>
#include "o3d3xx.h"

class O3DViewer
{
public:
  O3DViewer() : viewer("o3d3xx Cloud Viewer") {}

  void run(o3d3xx::Camera::Ptr cam)
  {
    o3d3xx::FrameGrabber::Ptr fg(new o3d3xx::FrameGrabber(cam));
    o3d3xx::ImageBuffer::Ptr buff(new o3d3xx::ImageBuffer());

    while (! viewer.wasStopped())
      {
	if (! fg->WaitForFrame(buff.get(), 1000))
	  {
	    continue;
	  }

	viewer.showCloud(buff->Cloud());
      }
  }

  pcl::visualization::CloudViewer viewer;
};

int main(int argc, const char **argv)
{
  int major, minor, patch;

  std::string camera_ip(o3d3xx::DEFAULT_IP);
  uint32_t xmlrpc_port = o3d3xx::DEFAULT_XMLRPC_PORT;

  try
    {
      o3d3xx::CmdLineOpts opts("Cloud Viewer");
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

      // init logging
      FLAGS_logbuflevel = -1;
      o3d3xx::Logging::Init();
      google::SetStderrLogging(google::FATAL);

      // instantiate camera
      camera_ip.assign(opts.vm["ip"].as<std::string>());
      xmlrpc_port = opts.vm["xmlrpc-port"].as<std::uint32_t>();
      o3d3xx::Camera::Ptr cam(new o3d3xx::Camera(camera_ip, xmlrpc_port));

      // run viewer
      O3DViewer v;
      v.run(cam);
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return 1;
    }

  return 0;
}
