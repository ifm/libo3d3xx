#include <iostream>
#include <memory>
#include <glog/logging.h>
#include "o3d3xx.h"

int main(int argc, const char** argv)
{
  FLAGS_logbuflevel = -1;
  o3d3xx::Logging::Init();
  google::SetStderrLogging(google::FATAL);

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  cam->FactoryReset();

  return 0;
}
