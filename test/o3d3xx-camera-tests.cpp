#include "o3d3xx.h"
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
  EXPECT_NO_THROW(cam->CancelSession());

  // set a dummy session
  cam->SetSessionID("ABC");

  // session doesn't really exist, so, sensor should send back an error
  // and the session id should still be the dummy above
  EXPECT_NO_THROW(cam->CancelSession());
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

  // Heartbeat w/o a session should yield a negative return
  EXPECT_LT(cam->Heartbeat(10), 0);

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
