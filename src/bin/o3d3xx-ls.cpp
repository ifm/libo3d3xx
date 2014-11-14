#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include "o3d3xx.h"

int main(int argc, const char** argv)
{
  std::string camera_ip;
  std::uint32_t xmlrpc_port;

  try
    {
      o3d3xx::CmdLineOpts opts("o3d3xx Application Listing");
      if (! opts.Parse(argc, argv, &camera_ip, &xmlrpc_port))
	{
	  return 0;
	}

      o3d3xx::Camera::Ptr cam =
      	std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port);

      std::unordered_map<std::string, std::string> params =
	cam->GetAllParameters();

      int active_app = std::stoi(params["ActiveApplication"]);

      std::vector<o3d3xx::Camera::app_entry_t> apps = cam->GetApplicationList();
      for (auto& app : apps)
	{
	  if (app.index == active_app)
	    {
	      std::cout << "* ";
	    }
	  else
	    {
	      std::cout << "  ";
	    }

	  std::cout << "[" << app.index
		    << "] id=" << app.id << ", name="
		    << app.name << ", description=" << app.description
		    << std::endl;
	}

    }
  catch (const std::exception& e)
    {
      std::cerr << "Failed to list applications: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}
