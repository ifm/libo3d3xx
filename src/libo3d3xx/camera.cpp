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

#include "o3d3xx/camera.hpp"
#include <map>
#include <mutex>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <glog/logging.h>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/client_simple.hpp>
#include "o3d3xx/util.hpp"

const std::string o3d3xx::DEFAULT_PASSWORD = "";
const std::string o3d3xx::DEFAULT_IP = "192.168.0.69";
const std::uint32_t o3d3xx::DEFAULT_XMLRPC_PORT = 80;

const std::string o3d3xx::XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";
const std::string o3d3xx::XMLRPC_SESSION = "session_XXX/";
const std::string o3d3xx::XMLRPC_EDIT = "edit/";
const std::string o3d3xx::XMLRPC_DEVICE = "device/";
const std::string o3d3xx::XMLRPC_NET = "network/";
const std::string o3d3xx::XMLRPC_APP = "application/";
const std::string o3d3xx::XMLRPC_IMAGER = "imager_001";

o3d3xx::Camera::Camera(const std::string& ip,
		       const std::uint32_t xmlrpc_port,
		       const std::string& password)
  : password_(password),
    ip_(ip),
    xmlrpc_port_(xmlrpc_port),
    xmlrpc_url_prefix_("http://" + ip + ":" + std::to_string(xmlrpc_port_)),
    xmlrpc_client_(xmlrpc_c::clientSimple()),
    session_("")
{
  DLOG(INFO) << "Initializing Camera: ip="
	     << this->ip_
	     << ", xmlrpc_port=" << this->xmlrpc_port_
	     << ", password=" << this->password_
	     << ", xmlrpc_url_prefix=" << this->xmlrpc_url_prefix_;
}

o3d3xx::Camera::~Camera()
{
  DLOG(INFO) << "Camera being destroyed";
  this->CancelSession();
}

std::string o3d3xx::Camera::GetIP()
{
  std::lock_guard<std::mutex> lock(this->ip_mutex_);
  return this->ip_;
}

void o3d3xx::Camera::SetIP(const std::string& ip)
{
  std::lock_guard<std::mutex> lock(this->ip_mutex_);
  this->ip_ = ip;
  this->SetXMLRPCURLPrefix(this->ip_, this->GetXMLRPCPort());
}

std::uint32_t o3d3xx::Camera::GetXMLRPCPort()
{
  std::lock_guard<std::mutex> lock(this->xmlrpc_port_mutex_);
  return this->xmlrpc_port_;
}

void o3d3xx::Camera::SetXMLRPCPort(const std::uint32_t& port)
{
  std::lock_guard<std::mutex> lock(this->xmlrpc_port_mutex_);
  this->xmlrpc_port_ = port;
  this->SetXMLRPCURLPrefix(this->GetIP(), this->xmlrpc_port_);
}

std::string o3d3xx::Camera::GetPassword()
{
  std::lock_guard<std::mutex> lock(this->password_mutex_);
  return this->password_;
}

void o3d3xx::Camera::SetPassword(const std::string& password)
{
  std::lock_guard<std::mutex> lock(this->password_mutex_);
  this->password_ = password;
}

std::string o3d3xx::Camera::GetSessionID()
{
  std::lock_guard<std::mutex> lock(this->session_mutex_);
  return this->session_;
}

void o3d3xx::Camera::SetSessionID(const std::string& id)
{
  std::lock_guard<std::mutex> lock(this->session_mutex_);
  this->session_ = id;
}

std::string o3d3xx::Camera::GetXMLRPCURLPrefix()
{
  std::lock_guard<std::mutex> lock(this->xmlrpc_url_prefix_mutex_);
  return this->xmlrpc_url_prefix_;
}

void o3d3xx::Camera::SetXMLRPCURLPrefix(const std::string& ip,
					const std::uint32_t& port)
{
  std::lock_guard<std::mutex> lock(this->xmlrpc_url_prefix_mutex_);
  this->xmlrpc_url_prefix_ =
    std::string("http://" + ip + ":" + std::to_string(port));
}

std::string
o3d3xx::Camera::GetParameter(const std::string& param)
{
  xmlrpc_c::value_string result(this->_XCallMain("getParameter",
						 "s", param.c_str()));
  return std::string(result);
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetAllParameters()
{
  return o3d3xx::value_struct_to_map(this->_XCallMain("getAllParameters"));
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetSWVersion()
{
  return o3d3xx::value_struct_to_map(this->_XCallMain("getSWVersion"));
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetHWInfo()
{
  return o3d3xx::value_struct_to_map(this->_XCallMain("getHWInfo"));
}

std::vector<o3d3xx::Camera::app_entry_t>
o3d3xx::Camera::GetApplicationList()
{
  xmlrpc_c::value_array result(this->_XCallMain("getApplicationList"));
  std::vector<xmlrpc_c::value> const res_vec(result.vectorValueValue());

  std::vector<o3d3xx::Camera::app_entry_t> retval;
  for (auto& entry : res_vec)
    {
      xmlrpc_c::value_struct const entry_st(entry);
      std::map<std::string, xmlrpc_c::value>
	entry_map(static_cast<std::map<std::string, xmlrpc_c::value> >
          (entry_st));

      o3d3xx::Camera::app_entry_t app;
      app.index = xmlrpc_c::value_int(entry_map["Index"]).cvalue();
      app.id = xmlrpc_c::value_int(entry_map["Id"]).cvalue();
      app.name = xmlrpc_c::value_string(entry_map["Name"]).cvalue();
      app.description =
	xmlrpc_c::value_string(entry_map["Description"]).cvalue();

      retval.push_back(app);
    }
  return retval;
}

void
o3d3xx::Camera::Reboot(const boot_mode& mode)
{
  this->_XCallMain("reboot", "i", mode);
}

std::string
o3d3xx::Camera::RequestSession()
{
  try
    {
      xmlrpc_c::value_string val_str(
        this->_XCallMain("requestSession", "s", this->GetPassword().c_str()));

      this->SetSessionID(static_cast<std::string>(val_str));
    }
  catch (const o3d3xx::error_t& ex)
    {
      // for now do nothing
    }

  return this->GetSessionID();
}

void
o3d3xx::Camera::CancelSession()
{
  if (this->GetSessionID() != "")
    {
      try
	{
	  this->_XCallSession("cancelSession");
	  this->SetSessionID("");
	}
      catch (const o3d3xx::error_t& ex)
	{
	  LOG(ERROR) << "Failed to cancel session: "
		     << this->GetSessionID();
	}
    }
}

int
o3d3xx::Camera::Heartbeat(int hb)
{
  int retval = -1;

  try
    {
      xmlrpc_c::value_int v_int(this->_XCallSession("heartbeat", "i", hb));
      retval = v_int.cvalue();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Failed to set heartbeat value";
    }

  return retval;
}

void
o3d3xx::Camera::SetOperatingMode(const o3d3xx::Camera::operating_mode& mode)
{
  try
    {
      this->_XCallSession("setOperatingMode", "i", mode);
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Failed to set operating mode!";
      throw(ex);
    }
}
