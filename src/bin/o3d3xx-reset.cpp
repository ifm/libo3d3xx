#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <boost/program_options.hpp>
#include "o3d3xx.h"

namespace po = boost::program_options;

int main(int argc, const char** argv)
{
  std::string camera_ip;
  uint32_t xmlrpc_port;

  try
    {
      //---------------------------------------------------
      // Handle command-line arguments
      //---------------------------------------------------
      o3d3xx::CmdLineOpts opts("o3d3xx Factory Reset");

      po::options_description reset_opts("Reset Information");
      reset_opts.add_options()
	("reboot", "Reboot the sensor after reset");
      opts.visible.add(reset_opts);

      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port))
	{
	  return 0;
	}

      //---------------------------------------------------
      // Reset the camera
      //---------------------------------------------------
      o3d3xx::Camera::Ptr cam =
	std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port);

      cam->RequestSession();
      cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
      cam->FactoryReset();

      if (opts.vm.count("reboot"))
	{
	  cam->Reboot();
	}
    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to reset camera: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}
