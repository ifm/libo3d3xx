#include <iostream>
#include <memory>
#include <glog/logging.h>
#include "o3d3xx.h"

int main(int argc, const char** argv)
{
  int major, minor, patch;

  std::string camera_ip(o3d3xx::DEFAULT_IP);
  uint32_t xmlrpc_port = o3d3xx::DEFAULT_XMLRPC_PORT;

  try
    {
      //---------------------------------------------------
      // Handle command-line arguments
      //---------------------------------------------------
      o3d3xx::CmdLineOpts opts("o3d3xx Factory Reset");
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

      FLAGS_logbuflevel = -1;
      o3d3xx::Logging::Init();
      google::SetStderrLogging(google::FATAL);

      camera_ip.assign(opts.vm["ip"].as<std::string>());
      xmlrpc_port = opts.vm["xmlrpc-port"].as<std::uint32_t>();

      o3d3xx::Camera::Ptr cam =
	std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port);
      cam->RequestSession();
      cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
      cam->FactoryReset();
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to reset camera: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}
