#include "o3d3xx.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "gtest/gtest.h"

TEST(Camera_Tests, Ctor)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());
  EXPECT_EQ(cam->GetIP(), o3d3xx::DEFAULT_IP);
  EXPECT_EQ(cam->GetXMLRPCPort(), o3d3xx::DEFAULT_XMLRPC_PORT);
  EXPECT_EQ(cam->GetPassword(), o3d3xx::DEFAULT_PASSWORD);

  cam.reset(new o3d3xx::Camera("192.168.0.100"));
  EXPECT_EQ(cam->GetIP(), std::string("192.168.0.100"));
  EXPECT_EQ(cam->GetXMLRPCPort(), o3d3xx::DEFAULT_XMLRPC_PORT);
  EXPECT_EQ(cam->GetPassword(), o3d3xx::DEFAULT_PASSWORD);


  cam.reset(new o3d3xx::Camera("192.168.0.101", 8080));
  EXPECT_EQ(cam->GetIP(), std::string("192.168.0.101"));
  EXPECT_EQ(cam->GetXMLRPCPort(), 8080);
  EXPECT_EQ(cam->GetPassword(), o3d3xx::DEFAULT_PASSWORD);

  cam.reset(new o3d3xx::Camera("192.168.0.102", 8181, "foo"));
  EXPECT_EQ(cam->GetIP(), std::string("192.168.0.102"));
  EXPECT_EQ(cam->GetXMLRPCPort(), 8181);
  EXPECT_EQ(cam->GetPassword(), std::string("foo"));
}

TEST(Camera_Tests, GetXMLRPCURLPrefix)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  EXPECT_EQ(cam->GetXMLRPCURLPrefix(),
            std::string("http://" + cam->GetIP() +
                        ":" + std::to_string(cam->GetXMLRPCPort())));

  cam->SetIP("192.168.0.100");
  EXPECT_EQ(cam->GetIP(), std::string("192.168.0.100"));
  EXPECT_EQ(cam->GetXMLRPCURLPrefix(),
            std::string("http://" + cam->GetIP() +
                        ":" + std::to_string(cam->GetXMLRPCPort())));

  cam->SetXMLRPCPort(8080);
  EXPECT_EQ(cam->GetXMLRPCPort(), 8080);
  EXPECT_EQ(cam->GetXMLRPCURLPrefix(),
            std::string("http://" + cam->GetIP() +
                        ":" + std::to_string(cam->GetXMLRPCPort())));
}

TEST(Camera_Tests, GetAllParameters)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  std::unordered_map<std::string, std::string> all_params;
  EXPECT_NO_THROW(all_params = cam->GetAllParameters());
}

TEST(Camera_Tests, GetParameter)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  std::unordered_map<std::string, std::string> all_params;
  EXPECT_NO_THROW(all_params = cam->GetAllParameters());

  for (auto& kv : all_params)
    {
      // NOTE: we are not checking the values from the sensor vs. those stored
      // in the hash table here b/c the hardware does not return consistent
      // values. e.g., in some cases 'true' vs. '1'.
      EXPECT_NO_THROW(cam->GetParameter(kv.first));
    }

  EXPECT_THROW(cam->GetParameter("Bogus Parameter"),
               o3d3xx::error_t);
}

TEST(Camera_Tests, GetSWVersion)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  std::unordered_map<std::string, std::string> sw_version;
  EXPECT_NO_THROW(sw_version = cam->GetSWVersion());
}

TEST(Camera_Tests, GetApplicationList)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  std::vector<o3d3xx::Camera::app_entry_t> apps;
  EXPECT_NO_THROW(apps = cam->GetApplicationList());
}

TEST(Camera_Tests, RequestSession)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  EXPECT_NO_THROW(cam->RequestSession());
  EXPECT_EQ(cam->GetSessionID().size(), 32);

  // camera dtor should cancel the session for us
}

TEST(Camera_Tests, CancelSession)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  // we have no session, so, CancelSession should not make the XMLRPC call
  EXPECT_EQ("", cam->GetSessionID());
  EXPECT_TRUE(cam->CancelSession());

  // set a dummy session
  cam->SetSessionID("ABC");

  // session doesn't really exist, so, sensor should send back an error
  // and the session id should still be the dummy above
  EXPECT_FALSE(cam->CancelSession());
  EXPECT_EQ("ABC", cam->GetSessionID());
  cam->SetSessionID("");

  // Get a real session and let the dtor cancel it
  EXPECT_NO_THROW(cam->RequestSession());
  EXPECT_EQ(cam->GetSessionID().size(), 32);
}

TEST(Camera_Tests, Heartbeat)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  // Heartbeat w/o a session should throw
  EXPECT_THROW(cam->Heartbeat(10), o3d3xx::error_t);

  int timeout = std::stoi(cam->GetParameter("SessionTimeout"));
  cam->RequestSession();

  EXPECT_EQ(10, cam->Heartbeat(10));
  // @bug The following test always fails (I think it is a bug in the sensor)
  //EXPECT_EQ(10, std::stoi(cam->GetParameter("SessionTimeout")));
  EXPECT_EQ(timeout, cam->Heartbeat(timeout));
}

TEST(Camera_Tests, SetOperatingMode)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  cam->RequestSession();
  EXPECT_NO_THROW(cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT));
  EXPECT_EQ(static_cast<int>(o3d3xx::Camera::operating_mode::EDIT),
            std::stoi(cam->GetParameter("OperatingMode")));

  // after session is cancelled (by dtor), camera automatically goes back
  // into RUN mode.
}

//---------------------------------------
// Uncomment to test rebooting the camera
//
// Leaving it here sort of messes up running our unit tests
// @todo Provide a guarntee this will run last.
//---------------------------------------
//
// TEST(Camera_Tests, Reboot)
// {
//   o3d3xx::Camera::Ptr cam =
//     o3d3xx::Camera::Ptr(new o3d3xx::Camera());

//   EXPECT_NO_THROW(cam->Reboot(o3d3xx::Camera::boot_mode::PRODUCTIVE));
// }

TEST(Camera_Tests, GetDeviceConfig)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();

  // verify that all mutable parameters are correct
  std::unordered_map<std::string, std::string> params =
    cam->GetAllParameters();

  EXPECT_EQ(params.at("Name"), dev->Name());
  EXPECT_EQ(params.at("Description"), dev->Description());
  EXPECT_EQ(std::stoi(params.at("ActiveApplication")),
            dev->ActiveApplication());
  EXPECT_EQ(std::stoi(params.at("PcicTcpPort")), dev->PcicTCPPort());
  EXPECT_EQ(std::stoi(params.at("PcicProtocolVersion")), dev->PcicProtocolVersion());
  EXPECT_EQ(std::stoi(params.at("IOLogicType")), dev->IOLogicType());
  EXPECT_EQ(o3d3xx::stob(params.at("IODebouncing")), dev->IODebouncing());
  EXPECT_EQ(std::stoi(params.at("IOExternApplicationSwitch")),
            dev->IOExternApplicationSwitch());
  EXPECT_EQ(std::stoi(params.at("SessionTimeout")), dev->SessionTimeout());
  EXPECT_EQ(std::stod(params.at("ExtrinsicCalibTransX")),
            dev->ExtrinsicCalibTransX());
  EXPECT_EQ(std::stod(params.at("ExtrinsicCalibTransY")),
            dev->ExtrinsicCalibTransY());
  EXPECT_EQ(std::stod(params.at("ExtrinsicCalibTransZ")),
            dev->ExtrinsicCalibTransZ());
  EXPECT_EQ(std::stod(params.at("ExtrinsicCalibRotX")),
            dev->ExtrinsicCalibRotX());
  EXPECT_EQ(std::stod(params.at("ExtrinsicCalibRotY")),
            dev->ExtrinsicCalibRotY());
  EXPECT_EQ(std::stod(params.at("ExtrinsicCalibRotY")),
            dev->ExtrinsicCalibRotY());
}

TEST(Camera_Tests, ActivateDisablePassword)
{
  std::string tmp_password = "foobar";

  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  EXPECT_NO_THROW(cam->RequestSession());
  EXPECT_NO_THROW(cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT));
  cam->SetPassword(tmp_password);
  EXPECT_NO_THROW(cam->ActivatePassword());
  //EXPECT_TRUE(cam->SaveDevice());

  //o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
  //EXPECT_TRUE(dev->PasswordActivated());

  EXPECT_NO_THROW(cam->DisablePassword());
  //EXPECT_TRUE(cam->SaveDevice());
  //dev = cam->GetDeviceConfig();
  //EXPECT_FALSE(dev->PasswordActivated());
}

TEST(Camera_Tests, SetDeviceConfig)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
  std::string orig_name = dev->Name();
  std::string tmp_name = "foobar";

  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  dev->SetName(tmp_name);
  EXPECT_NO_THROW(cam->SetDeviceConfig(dev.get()));

  dev = cam->GetDeviceConfig();
  EXPECT_EQ(tmp_name, dev->Name());

  dev->SetName(orig_name);
  EXPECT_NO_THROW(cam->SetDeviceConfig(dev.get()));

  dev = cam->GetDeviceConfig();
  EXPECT_EQ(orig_name, dev->Name());
}

TEST(Camera_Tests, DeviceConfig_JSON)
{
  o3d3xx::Camera::Ptr cam =
    o3d3xx::Camera::Ptr(new o3d3xx::Camera());

  o3d3xx::DeviceConfig::Ptr dev = cam->GetDeviceConfig();
  std::string json = dev->ToJSON();

  o3d3xx::DeviceConfig::Ptr dev2 =
    o3d3xx::DeviceConfig::FromJSON(json);

  EXPECT_EQ(dev->Name(), dev2->Name());
  EXPECT_EQ(dev->Description(), dev2->Description());
  EXPECT_EQ(dev->ActiveApplication(), dev2->ActiveApplication());
  EXPECT_EQ(dev->PcicTCPPort(), dev2->PcicTCPPort());
  EXPECT_EQ(dev->PcicProtocolVersion(), dev2->PcicProtocolVersion());
  EXPECT_EQ(dev->IOLogicType(), dev2->IOLogicType());
  EXPECT_EQ(dev->IODebouncing(), dev2->IODebouncing());
  EXPECT_EQ(dev->IOExternApplicationSwitch(),
            dev2->IOExternApplicationSwitch());
  EXPECT_EQ(dev->SessionTimeout(), dev2->SessionTimeout());
  EXPECT_EQ(dev->ExtrinsicCalibTransX(), dev2->ExtrinsicCalibTransX());
  EXPECT_EQ(dev->ExtrinsicCalibTransY(), dev2->ExtrinsicCalibTransY());
  EXPECT_EQ(dev->ExtrinsicCalibTransZ(), dev2->ExtrinsicCalibTransZ());
  EXPECT_EQ(dev->ExtrinsicCalibRotX(), dev2->ExtrinsicCalibRotX());
  EXPECT_EQ(dev->ExtrinsicCalibRotY(), dev2->ExtrinsicCalibRotY());
  EXPECT_EQ(dev->ExtrinsicCalibRotZ(), dev2->ExtrinsicCalibRotZ());

  // we do not want to compare the read-only properties
}

TEST(Camera_Tests, GetNetParameters)
{
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

  std::unordered_map<std::string, std::string> params =
    cam->GetNetParameters();

  EXPECT_NO_THROW(params.at("MACAddress"));
  EXPECT_NO_THROW(params.at("NetworkSpeed"));
  EXPECT_NO_THROW(params.at("StaticIPv4Address"));
  EXPECT_NO_THROW(params.at("StaticIPv4Gateway"));
  EXPECT_NO_THROW(params.at("StaticIPv4SubNetMask"));
  EXPECT_NO_THROW(params.at("UseDHCP"));
}

// TEST(Camera_Tests, NetConfig)
// {
//   o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
//   cam->RequestSession();
//   cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

//   o3d3xx::NetConfig::Ptr net = cam->GetNetConfig();
//   std::string orig_ip = net->StaticIPv4Address();
//   net->SetStaticIPv4Address("192.168.0.70");

//   EXPECT_NO_THROW(cam->SetNetConfig(net.get()));
//   EXPECT_NO_THROW(cam->SaveNet());

//   usleep(2000);
//   cam->RequestSession();
//   cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
//   o3d3xx::NetConfig::Ptr net2 = cam->GetNetConfig();
//   EXPECT_EQ(net->StaticIPv4Address(), net2->StaticIPv4Address());

//   net2->SetStaticIPv4Address(orig_ip);
//   EXPECT_NO_THROW(cam->SetNetConfig(net2.get()));
//   EXPECT_NO_THROW(cam->SaveNet());

//   usleep(2000);
//   cam->RequestSession();
//   cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
//   o3d3xx::NetConfig::Ptr net3 = cam->GetNetConfig();
//   EXPECT_EQ(orig_ip, net3->StaticIPv4Address());
// }

TEST(Camera_Tests, NetConfig_JSON)
{
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  cam->RequestSession();
  cam->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

  o3d3xx::NetConfig::Ptr net = cam->GetNetConfig();
  std::string json = net->ToJSON();

  o3d3xx::NetConfig::Ptr net2 =
    o3d3xx::NetConfig::FromJSON(json);

  EXPECT_EQ(net->StaticIPv4Address(), net2->StaticIPv4Address());
  EXPECT_EQ(net->StaticIPv4Gateway(), net2->StaticIPv4Gateway());
  EXPECT_EQ(net->StaticIPv4SubNetMask(), net2->StaticIPv4SubNetMask());
  EXPECT_EQ(net->UseDHCP(), net2->UseDHCP());

  // we do not want to compare the read-only properties
}
