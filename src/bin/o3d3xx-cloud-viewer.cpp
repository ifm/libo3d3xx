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
#include <boost/program_options.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "o3d3xx.h"

namespace po = boost::program_options;

class CmdLineOpts
{
public:
  CmdLineOpts(int argc, const char **argv)
    : visible("Visualize Point Clouds from O3D3XX Camera")
  {
    // generic options
    po::options_description general("General");
    general.add_options()
      ("version,v", "Print version string and exit")
      ("help,h", "Produce help message");

    // options for connecting to the camera
    po::options_description connection_opts("Connection Information");
    connection_opts.add_options()
      ("ip",
       po::value<std::string>()->default_value(o3d3xx::DEFAULT_IP),
       "IP address of camera")

      ("xmlrpc-port",
       po::value<uint32_t>()->default_value(o3d3xx::DEFAULT_XMLRPC_PORT),
       "XMLRPC port of camera");

    po::options_description cmdline_options;
    cmdline_options.add(general).add(connection_opts);
    this->visible.add(general).add(connection_opts);
    po::store(po::command_line_parser(argc, argv).
	      options(cmdline_options).run(), this->vm);
    po::notify(this->vm);
  }

  po::variables_map vm;
  po::options_description visible;
};

class O3DViewer
{
public:
  O3DViewer() : viewer("o3d3xx-toolbox Cloud Viewer") {}

  void run(o3d3xx::Camera::Ptr cam)
  {
    o3d3xx::FrameGrabber::Ptr fg(new o3d3xx::FrameGrabber(cam));

    pcl::PointCloud<o3d3xx::PointT>::Ptr
      cloud(new pcl::PointCloud<o3d3xx::PointT>());

    while (! viewer.wasStopped())
      {
	if (! fg->WaitForCloud(cloud, 1000))
	  {
	    continue;
	  }

	viewer.showCloud(cloud);
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
      CmdLineOpts opts(argc, argv);

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

      //
      // pull out values required to connect to the camera
      //
      if (opts.vm.count("ip"))
	{
	  camera_ip.assign(opts.vm["ip"].as<std::string>());
	}

      if (opts.vm.count("xmlrpc-port"))
	{
	  xmlrpc_port = opts.vm["xmlrpc-port"].as<std::uint32_t>();
	}

      //
      // instantiate the camera
      //
      o3d3xx::Camera::Ptr cam(new o3d3xx::Camera(camera_ip, xmlrpc_port));

      //
      // run the viewer
      //
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
