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

#include "o3d3xx_camera/camera.hpp"
#include <map>
#include <memory>
#include <mutex>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include <xmlrpc-c/client.hpp>
#include "o3d3xx_camera/util.h"
#include "o3d3xx_camera/device_config.h"
#include "o3d3xx_camera/net_config.h"
#include "o3d3xx_camera/app_config.h"
#include "o3d3xx_camera/imager_config.h"
#include "o3d3xx_camera/version.h"

const std::string o3d3xx::DEFAULT_PASSWORD = "";
const std::string o3d3xx::DEFAULT_IP =
  std::getenv("O3D3XX_IP") == nullptr ?
  "192.168.0.69" : std::string(std::getenv("O3D3XX_IP"));

const std::string o3d3xx::DEFAULT_SUBNET = "255.255.255.0";
const std::string o3d3xx::DEFAULT_GW = "192.168.0.201";
const std::uint32_t o3d3xx::DEFAULT_XMLRPC_PORT = 80;
const int o3d3xx::MAX_HEARTBEAT = 300; // seconds
const int o3d3xx::NET_WAIT = 3000; // millis

const std::string o3d3xx::XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";
const std::string o3d3xx::XMLRPC_SESSION = "session_XXX/";
const std::string o3d3xx::XMLRPC_EDIT = "edit/";
const std::string o3d3xx::XMLRPC_DEVICE = "device/";
const std::string o3d3xx::XMLRPC_NET = "network/";
const std::string o3d3xx::XMLRPC_APP = "application/";
const std::string o3d3xx::XMLRPC_IMAGER = "imager_001/";
const std::string o3d3xx::XMLRPC_SPATIALFILTER = "spatialfilter";
const std::string o3d3xx::XMLRPC_TEMPORALFILTER = "temporalfilter";

const int o3d3xx::RES_23K = 0;
const int o3d3xx::RES_100K = 1;

o3d3xx::Camera::Camera(const std::string& ip,
                       const std::uint32_t xmlrpc_port,
                       const std::string& password)
  : password_(password),
    ip_(ip),
    xmlrpc_port_(xmlrpc_port),
    xmlrpc_url_prefix_("http://" + ip + ":" + std::to_string(xmlrpc_port_)),
    xclient_(new xmlrpc_c::client_xml(
               xmlrpc_c::clientXmlTransportPtr(
                 new xmlrpc_c::clientXmlTransport_curl(
                   xmlrpc_c::clientXmlTransport_curl::constrOpt().
                   timeout(o3d3xx::NET_WAIT))))),
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
                                                 param.c_str()));
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
  this->_XCallMain("reboot", static_cast<int>(mode));
}

std::string
o3d3xx::Camera::RequestSession()
{
  xmlrpc_c::value_string val_str(
    this->_XCallMain("requestSession",
                     this->GetPassword().c_str(),
                     std::string("")));

  this->SetSessionID(static_cast<std::string>(val_str));
  this->Heartbeat(o3d3xx::MAX_HEARTBEAT);
  return this->GetSessionID();
}

bool
o3d3xx::Camera::CancelSession()
{
  bool retval = true;

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
                     << this->GetSessionID() << " -> "
                     << ex.what();

          retval = false;
        }
    }

  return retval;
}

int
o3d3xx::Camera::Heartbeat(int hb)
{
  xmlrpc_c::value_int v_int(this->_XCallSession("heartbeat", hb));
  return v_int.cvalue();
}

void
o3d3xx::Camera::SetOperatingMode(const o3d3xx::Camera::operating_mode& mode)
{
  this->_XCallSession("setOperatingMode", static_cast<int>(mode));
}

o3d3xx::DeviceConfig::Ptr
o3d3xx::Camera::GetDeviceConfig()
{
  return std::make_shared<o3d3xx::DeviceConfig>(this->GetAllParameters());
}

void
o3d3xx::Camera::ActivatePassword()
{
  this->_XCallDevice("activatePassword",
                     this->GetPassword().c_str());
}

void
o3d3xx::Camera::DisablePassword()
{
  this->_XCallDevice("disablePassword");
}

void
o3d3xx::Camera::SaveDevice()
{
  this->_XCallDevice("save");
}

void
o3d3xx::Camera::SetDeviceConfig(const o3d3xx::DeviceConfig* config)
{
  o3d3xx::DeviceConfig::Ptr dev = this->GetDeviceConfig();

  // only check mutable parameters and only make the network call if they are
  // different
  if (dev->Name() != config->Name())
    {
      this->_XCallDevice("setParameter", "Name",
                         config->Name().c_str());
    }

  if (dev->Description() != config->Description())
    {
      this->_XCallDevice("setParameter", "Description",
                         config->Description().c_str());
    }

  if (dev->ActiveApplication() != config->ActiveApplication())
    {
      this->_XCallDevice("setParameter", "ActiveApplication",
                         config->ActiveApplication());
    }

  if (dev->PcicTCPPort() != config->PcicTCPPort())
    {
      this->_XCallDevice("setParameter", "PcicTcpPort",
                         config->PcicTCPPort());
    }

  if (dev->PcicProtocolVersion() != config->PcicProtocolVersion())
    {
      this->_XCallDevice("setParameter", "PcicProtocolVersion",
                         config->PcicProtocolVersion());
    }

  if (dev->IOLogicType() != config->IOLogicType())
    {
      this->_XCallDevice("setParameter", "IOLogicType",
                         config->IOLogicType());
    }

  if (dev->IOExternApplicationSwitch() !=
      config->IOExternApplicationSwitch())
    {
      this->_XCallDevice("setParameter", "IOExternApplicationSwitch",
                         config->IOExternApplicationSwitch());
    }

  if (dev->SessionTimeout() != config->SessionTimeout())
    {
      this->_XCallDevice("setParameter", "SessionTimeout",
                         config->SessionTimeout());
    }

  if (dev->ServiceReportPassedBuffer() != config->ServiceReportPassedBuffer())
    {
      this->_XCallDevice("setParameter", "ServiceReportPassedBuffer",
                         config->ServiceReportPassedBuffer());
    }

  if (dev->ServiceReportFailedBuffer() != config->ServiceReportFailedBuffer())
    {
      this->_XCallDevice("setParameter", "ServiceReportFailedBuffer",
                         config->ServiceReportFailedBuffer());
    }

  if (dev->ExtrinsicCalibTransX() != config->ExtrinsicCalibTransX())
    {
      this->_XCallDevice("setParameter", "ExtrinsicCalibTransX",
                         config->ExtrinsicCalibTransX());
    }

  if (dev->ExtrinsicCalibTransY() != config->ExtrinsicCalibTransY())
    {
      this->_XCallDevice("setParameter", "ExtrinsicCalibTransY",
                         config->ExtrinsicCalibTransY());
    }

  if (dev->ExtrinsicCalibTransZ() != config->ExtrinsicCalibTransZ())
    {
      this->_XCallDevice("setParameter", "ExtrinsicCalibTransZ",
                         config->ExtrinsicCalibTransZ());
    }

  if (dev->ExtrinsicCalibRotX() != config->ExtrinsicCalibRotX())
    {
      this->_XCallDevice("setParameter", "ExtrinsicCalibRotX",
                         config->ExtrinsicCalibRotX());
    }

  if (dev->ExtrinsicCalibRotY() != config->ExtrinsicCalibRotY())
    {
      this->_XCallDevice("setParameter", "ExtrinsicCalibRotY",
                         config->ExtrinsicCalibRotY());
    }

  if (dev->ExtrinsicCalibRotZ() != config->ExtrinsicCalibRotZ())
    {
      this->_XCallDevice("setParameter", "ExtrinsicCalibRotZ",
                         config->ExtrinsicCalibRotZ());
    }

  if (dev->EvaluationFinishedMinHoldTime() !=
      config->EvaluationFinishedMinHoldTime())
    {
      this->_XCallDevice("setParameter", "EvaluationFinishedMinHoldTime",
                         config->EvaluationFinishedMinHoldTime());
    }

  if (dev->SaveRestoreStatsOnApplSwitch() !=
      config->SaveRestoreStatsOnApplSwitch())
    {
      this->_XCallDevice("setParameter", "SaveRestoreStatsOnApplSwitch",
                         config->SaveRestoreStatsOnApplSwitch());
    }

  if (dev->PNIODeviceName() != config->PNIODeviceName())
    {
      this->_XCallDevice("setParameter", "PNIODeviceName",
                         config->PNIODeviceName());
    }

  if (dev->EthernetFieldBus() != config->EthernetFieldBus())
    {
      this->_XCallDevice("setParameter", "EthernetFieldBus",
                         config->EthernetFieldBus());
    }
}

o3d3xx::NetConfig::Ptr
o3d3xx::Camera::GetNetConfig()
{
  return std::make_shared<o3d3xx::NetConfig>(this->GetNetParameters());
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetNetParameters()
{
  return o3d3xx::value_struct_to_map(this->_XCallNet("getAllParameters"));
}

void
o3d3xx::Camera::SetNetConfig(const o3d3xx::NetConfig* config)
{
  bool has_changed = false;
  this->SetNetConfig(config, &has_changed);
}

void
o3d3xx::Camera::SetNetConfig(const o3d3xx::NetConfig* config,
                             bool* has_changed)
{
  *has_changed = false;

  // only set mutable parameters and only if they are different than current
  // settings.
  o3d3xx::NetConfig::Ptr net = this->GetNetConfig();

  if ((net->UseDHCP() == true) &&
      (config->UseDHCP() == true))
    {
      // We short circuit everything here b/c if the camera
      // is already set to DHCP and the request is to set it
      // to DHCP, regardless of what the other network params
      // are set to, the camera is going to come up as DHCP.
      //
      // This can be useful to avoid an unnecessary reboot of the camera,
      // for example, in `FromJSON(...)'
      //
      return;
    }

  if (net->StaticIPv4Address() != config->StaticIPv4Address())
    {
      this->_XCallNet("setParameter",
                      "StaticIPv4Address",
                      config->StaticIPv4Address().c_str());

      *has_changed = true;
    }

  if (net->StaticIPv4Gateway() != config->StaticIPv4Gateway())
    {
      this->_XCallNet("setParameter",
                      "StaticIPv4Gateway",
                      config->StaticIPv4Gateway().c_str());

      *has_changed = true;
    }

  if (net->StaticIPv4SubNetMask() != config->StaticIPv4SubNetMask())
    {
      this->_XCallNet("setParameter",
                      "StaticIPv4SubNetMask",
                      config->StaticIPv4SubNetMask().c_str());

      *has_changed = true;
    }

  if (net->UseDHCP() != config->UseDHCP())
    {
      this->_XCallNet("setParameter",
                      "UseDHCP", config->UseDHCP() ? "true" : "false");

      *has_changed = true;
    }
}

void
o3d3xx::Camera::SaveNet()
{
  o3d3xx::NetConfig::Ptr net = this->GetNetConfig();

  // When calling `saveAndActivateConfig' on the network object of the xmlrpc
  // interface, the sensor's network interface will have to be reset. As a
  // result, there will be no xmlrpc result returned -- we expect a timeout.
  try
    {
      this->_XCallNet("saveAndActivateConfig");
    }
  catch (const o3d3xx::error_t& ex)
    {
      if (ex.code() != O3D3XX_XMLRPC_TIMEOUT)
        {
          throw ex;
        }
    }

  this->SetSessionID("");
  this->SetIP(net->StaticIPv4Address());
}

int
o3d3xx::Camera::CopyApplication(int idx)
{
  try
    {
      xmlrpc_c::value_int v_int(this->_XCallEdit("copyApplication", idx));
      return v_int.cvalue();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Failed to copy application with index="
                 << idx << ": "
                 << ex.what();

      throw ex;
    }
}

void
o3d3xx::Camera::DeleteApplication(int idx)
{
  this->_XCallEdit("deleteApplication", idx);
}

int
o3d3xx::Camera::CreateApplication()
{
  try
    {
      xmlrpc_c::value_int v_int(this->_XCallEdit("createApplication"));
      return v_int.cvalue();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Failed to create application: "
                 << ex.what();

      throw ex;
    }
}

void
o3d3xx::Camera::ChangeAppNameAndDescription(int idx,
                                            const std::string& name,
                                            const std::string& descr)
{
  this->_XCallEdit("changeNameAndDescription",
                   idx, name.c_str(), descr.c_str());
}

void
o3d3xx::Camera::EditApplication(int idx)
{
  this->_XCallEdit("editApplication", idx);
}

void
o3d3xx::Camera::StopEditingApplication()
{
  this->_XCallEdit("stopEditingApplication");
}

void
o3d3xx::Camera::FactoryReset()
{
  this->_XCallEdit("factoryReset");
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetAppParameters()
{
  return o3d3xx::value_struct_to_map(this->_XCallApp("getAllParameters"));
}

void
o3d3xx::Camera::SaveApp()
{
  this->_XCallApp("save");
}

o3d3xx::AppConfig::Ptr
o3d3xx::Camera::GetAppConfig()
{
  return o3d3xx::AppConfig::Ptr(
           new o3d3xx::AppConfig(this->GetAppParameters()));
}

void
o3d3xx::Camera::SetAppConfig(const o3d3xx::AppConfig* config)
{
  o3d3xx::AppConfig::Ptr app = this->GetAppConfig();

  if (app->Name() != config->Name())
    {
      this->_XCallApp("setParameter",
                      "Name", config->Name().c_str());
    }

  if (app->Description() != config->Description())
    {
      this->_XCallApp("setParameter",
                      "Description", config->Description().c_str());
    }

  if (app->TriggerMode() != config->TriggerMode())
    {
      this->_XCallApp("setParameter",
                      "TriggerMode", config->TriggerMode());
    }

  //
  // To avoid making extraneous network calls, we compare the
  // parsed JSON and not the JSON strings for the result schema.
  //
  boost::property_tree::ptree app_schema_pt;
  boost::property_tree::ptree config_schema_pt;
  std::istringstream app_is;
  std::istringstream config_is;

  if (config->PcicTcpResultSchema() != "")
    {
      try
        {
          app_is.str(app->PcicTcpResultSchema());
          config_is.str(config->PcicTcpResultSchema());
          app_is.seekg(0, app_is.beg);
          config_is.seekg(0, config_is.beg);
          boost::property_tree::read_json(app_is, app_schema_pt);
          boost::property_tree::read_json(config_is, config_schema_pt);

          if (app_schema_pt != config_schema_pt)
            {
              LOG(WARNING) << "Setting PCIC TCP Result Schema to: "
                           << config->PcicTcpResultSchema();

              this->_XCallApp("setParameter",
                              "PcicTcpResultSchema",
                              config->PcicTcpResultSchema());
            }
        }
      catch (const std::exception& ex)
        {
          LOG(WARNING) << "Error parsing TCP result schema JSON: "
                       << ex.what();
          LOG(WARNING) << "On-camera JSON: "
                       << app->PcicTcpResultSchema();
          LOG(WARNING) << "Desired JSON: "
                       << config->PcicTcpResultSchema();
        }
    }

  //
  // re-use above objects for parsing the EIP JSON schema
  //
  if (config->PcicEipResultSchema() != "")
    {
      try
        {
          app_is.str(app->PcicEipResultSchema());
          config_is.str(config->PcicEipResultSchema());
          app_is.seekg(0, app_is.beg);
          config_is.seekg(0, config_is.beg);
          boost::property_tree::read_json(app_is, app_schema_pt);
          boost::property_tree::read_json(config_is, config_schema_pt);

          if (app_schema_pt != config_schema_pt)
            {
              LOG(WARNING) << "Setting PCIC EIP Result Schema to: "
                           << config->PcicEipResultSchema();

              this->_XCallApp("setParameter",
                              "PcicEipResultSchema",
                              config->PcicEipResultSchema());
            }
        }
      catch (const std::exception& ex)
        {
          LOG(WARNING) << "Error parsing EIP result schema JSON: "
                       << ex.what();
          LOG(WARNING) << "On-camera JSON: "
                       << app->PcicEipResultSchema();
          LOG(WARNING) << "Desired JSON: "
                       << config->PcicEipResultSchema();
        }
    }

  //
  // re-use above objects for parsing the PNIO JSON schema
  //
  if (config->PcicPnioResultSchema() != "")
    {
      try
        {
          app_is.str(app->PcicPnioResultSchema());
          config_is.str(config->PcicPnioResultSchema());
          app_is.seekg(0, app_is.beg);
          config_is.seekg(0, config_is.beg);
          boost::property_tree::read_json(app_is, app_schema_pt);
          boost::property_tree::read_json(config_is, config_schema_pt);

          if (app_schema_pt != config_schema_pt)
            {
              LOG(WARNING) << "Setting PCIC PNIO Result Schema to: "
                           << config->PcicPnioResultSchema();

              this->_XCallApp("setParameter",
                              "PcicPnioResultSchema",
                              config->PcicPnioResultSchema());
            }
        }
      catch (const std::exception& ex)
        {
          LOG(WARNING) << "Error parsing PNIO result schema JSON: "
                       << ex.what();
          LOG(WARNING) << "On-camera JSON: "
                       << app->PcicPnioResultSchema();
          LOG(WARNING) << "Desired JSON: "
                       << config->PcicPnioResultSchema();
        }
    }

  //
  // re-use the above objects for parsing the LogicGraph JSON
  //
  if (config->LogicGraph() != "")
    {
      try
        {
          app_is.str(app->LogicGraph());
          config_is.str(config->LogicGraph());
          app_is.seekg(0, app_is.beg);
          config_is.seekg(0, config_is.beg);
          boost::property_tree::read_json(app_is, app_schema_pt);
          boost::property_tree::read_json(config_is, config_schema_pt);

          if (app_schema_pt != config_schema_pt)
            {
              LOG(WARNING) << "Setting LogicGraph to: " << config->LogicGraph();
              this->_XCallApp("setParameter",
                              "LogicGraph", config->LogicGraph());
            }
        }
      catch (const std::exception& ex)
        {
          LOG(WARNING) << "Error parsing logic graph JSON: " << ex.what();
          LOG(WARNING) << "On-camera JSON: " << app->LogicGraph();
          LOG(WARNING) << "Desired JSON: " << config->LogicGraph();
        }
    }
}

std::vector<std::string>
o3d3xx::Camera::GetAvailableImagerTypes()
{
  xmlrpc_c::value_array a = this->_XCallImager("availableTypes");

  std::vector<xmlrpc_c::value> v = a.vectorValueValue();
  std::vector<std::string> retval;
  for (auto& vs : v)
    {
      retval.push_back(static_cast<std::string>(xmlrpc_c::value_string(vs)));
    }

  return retval;
}

void
o3d3xx::Camera::ChangeImagerType(const std::string& type)
{
  this->_XCallImager("changeType", type.c_str());
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetImagerParameters()
{
  return o3d3xx::value_struct_to_map(this->_XCallImager("getAllParameters"));
}

std::unordered_map<std::string,
                   std::unordered_map<std::string, std::string> >
o3d3xx::Camera::GetImagerParameterLimits()
{
  return o3d3xx::value_struct_to_map_of_maps(
    this->_XCallImager("getAllParameterLimits"));
}

o3d3xx::ImagerConfig::Ptr
o3d3xx::Camera::GetImagerConfig()
{
  return std::make_shared<o3d3xx::ImagerConfig>(this->GetImagerParameters());
}

void
o3d3xx::Camera::SetImagerConfig(const o3d3xx::ImagerConfig* config)
{
  o3d3xx::ImagerConfig::Ptr im = this->GetImagerConfig();

  DLOG(INFO) << "Setting Imager Config for Type="
             << im->Type();

  if (im->Channel() != config->Channel())
    {
      DLOG(INFO) << "Setting Channel="
                 << config->Channel();

      this->_XCallImager("setParameter",
                         "Channel", config->Channel());
    }

  if (im->ClippingBottom() != config->ClippingBottom())
    {
      DLOG(INFO) << "Setting ClippingBottom="
                 << config->ClippingBottom();

      this->_XCallImager("setParameter",
                         "ClippingBottom", config->ClippingBottom());
    }

  if (im->ClippingLeft() != config->ClippingLeft())
    {
      DLOG(INFO) << "Setting ClippingLeft="
                 << config->ClippingLeft();

      this->_XCallImager("setParameter",
                         "ClippingLeft", config->ClippingLeft());
    }

  if (im->ClippingRight() != config->ClippingRight())
    {
      DLOG(INFO) << "Setting ClippingRight="
                 << config->ClippingRight();

      this->_XCallImager("setParameter",
                         "ClippingRight", config->ClippingRight());
    }

  if (im->ClippingTop() != config->ClippingTop())
    {
      DLOG(INFO) << "Setting ClippingTop="
                 << config->ClippingTop();

      this->_XCallImager("setParameter",
                         "ClippingTop", config->ClippingTop());
    }

  if (im->ContinuousAutoExposure() != config->ContinuousAutoExposure())
    {
      DLOG(INFO) << "Setting ContinuousAutoExposure="
                 << config->ContinuousAutoExposure();

      this->_XCallImager("setParameter",
                         "ContinuousAutoExposure",
                         config->ContinuousAutoExposure());
    }

  if (im->EnableAmplitudeCorrection() != config->EnableAmplitudeCorrection())
    {
      DLOG(INFO) << "Setting EnableAmplitudeCorrection="
                 << config->EnableAmplitudeCorrection();

      this->_XCallImager("setParameter",
                         "EnableAmplitudeCorrection",
                         config->EnableAmplitudeCorrection());
    }

  if (im->EnableFastFrequency() != config->EnableFastFrequency())
    {
      DLOG(INFO) << "Setting EnableFastFrequency="
                 << config->EnableFastFrequency();

      this->_XCallImager("setParameter",
                         "EnableFastFrequency",
                         config->EnableFastFrequency());
    }

  if (im->EnableFilterAmplitudeImage() != config->EnableFilterAmplitudeImage())
    {
      DLOG(INFO) << "Setting EnableFilterAmplitudeImage="
                 << config->EnableFilterAmplitudeImage();

      this->_XCallImager("setParameter",
                         "EnableFilterAmplitudeImage",
                         config->EnableFilterAmplitudeImage());
    }

  if (im->EnableFilterDistanceImage() != config->EnableFilterDistanceImage())
    {
      DLOG(INFO) << "Setting EnableFilterDistanceImage="
                 << config->EnableFilterDistanceImage();

      this->_XCallImager("setParameter",
                         "EnableFilterDistanceImage",
                         config->EnableFilterDistanceImage());
    }

  if (im->EnableRectificationAmplitudeImage() !=
      config->EnableRectificationAmplitudeImage())
    {
      DLOG(INFO) << "Setting EnableRectificationAmplitudeImage="
                 << config->EnableRectificationAmplitudeImage();

      this->_XCallImager("setParameter",
                         "EnableRectificationAmplitudeImage",
                         config->EnableRectificationAmplitudeImage());
    }

  if (im->EnableRectificationDistanceImage() !=
      config->EnableRectificationDistanceImage())
    {
      DLOG(INFO) << "Setting EnableRectificationDistanceImage="
                 << config->EnableRectificationDistanceImage();

      this->_XCallImager("setParameter",
                         "EnableRectificationDistanceImage",
                         config->EnableRectificationDistanceImage());
    }

  if (boost::algorithm::ends_with(im->Type(), "high"))
    {

    }
  else if (boost::algorithm::ends_with(im->Type(), "low"))
    {
      if (im->ExposureTime() != config->ExposureTime())
        {
          DLOG(INFO) << "Setting ExposureTime="
                     << config->ExposureTime();

          this->_XCallImager("setParameter",
                             "ExposureTime", config->ExposureTime());
        }
    }
  else
    {
      if (im->ExposureTime() != config->ExposureTime())
        {
          DLOG(INFO) << "Setting ExposureTime="
                     << config->ExposureTime();

          this->_XCallImager("setParameter",
                             "ExposureTime", config->ExposureTime());
        }

      if (im->ExposureTimeRatio() != config->ExposureTimeRatio())
        {
          DLOG(INFO) << "Setting ExposureTimeRatio="
                     << config->ExposureTimeRatio();

          this->_XCallImager("setParameter",
                             "ExposureTimeRatio",
                             config->ExposureTimeRatio());
        }
    }

  if (im->FrameRate() != config->FrameRate())
    {
      DLOG(INFO) << "Setting FrameRate="
                 << config->FrameRate();

      this->_XCallImager("setParameter",
                         "FrameRate", config->FrameRate());
    }

  if (im->MinimumAmplitude() != config->MinimumAmplitude())
    {
      DLOG(INFO) << "Setting MinimumAmplitude="
                 << config->MinimumAmplitude();

      this->_XCallImager("setParameter",
                         "MinimumAmplitude", config->MinimumAmplitude());
    }

  if (im->Resolution() != config->Resolution())
    {
      DLOG(INFO) << "Setting Resolution=" << config->Resolution();

      this->_XCallImager("setParameter",
                         "Resolution", config->Resolution());
    }

  if (im->SpatialFilterType() != config->SpatialFilterType())
    {
      DLOG(INFO) << "Setting SpatialFilterType="
                 << config->SpatialFilterType();

      this->_XCallImager("setParameter",
                         "SpatialFilterType",
                         config->SpatialFilterType());
    }

  //! @todo Is SymmetryThreshold implemented in camera?
  // if (im->SymmetryThreshold() != config->SymmetryThreshold())
  //   {
  //     DLOG(INFO) << "Setting SymmetryThreshold="
  //             << config->SymmetryThreshold();

  //     this->_XCallImager("setParameter",
  //                     "SymmetryTreshold",
  //                     config->SymmetryThreshold());
  //   }

  if (im->TemporalFilterType() != config->TemporalFilterType())
    {
      DLOG(INFO) << "Setting TemporalFilterType="
                 << config->TemporalFilterType();

      this->_XCallImager("setParameter",
                         "TemporalFilterType",
                         config->TemporalFilterType());
    }

  if (im->ThreeFreqMax2FLineDistPercentage() !=
      config->ThreeFreqMax2FLineDistPercentage())
    {
      DLOG(INFO) << "Setting ThreeFreqMax2FLineDistPercentage="
                 << config->ThreeFreqMax2FLineDistPercentage();

      this->_XCallImager("setParameter",
                         "ThreeFreqMax2FLineDistPercentage",
                         config->ThreeFreqMax2FLineDistPercentage());
    }

  if (im->ThreeFreqMax3FLineDistPercentage() !=
      config->ThreeFreqMax3FLineDistPercentage())
    {
      DLOG(INFO) << "Setting ThreeFreqMax3FLineDistPercentage="
                 << config->ThreeFreqMax3FLineDistPercentage();

      this->_XCallImager("setParameter",
                         "ThreeFreqMax3FLineDistPercentage",
                         config->ThreeFreqMax3FLineDistPercentage());
    }

  if (im->TwoFreqMaxLineDistPercentage() !=
      config->TwoFreqMaxLineDistPercentage())
    {
      DLOG(INFO) << "Setting TwoFreqMaxLineDistPercentage="
                 << config->TwoFreqMaxLineDistPercentage();

      this->_XCallImager("setParameter",
                         "TwoFreqMaxLineDistPercentage",
                         config->TwoFreqMaxLineDistPercentage());
    }

  //
  // To avoid making extraneous network calls, we compare the
  // parsed JSON and not the JSON strings for the cuboid string
  //
  boost::property_tree::ptree img_schema_pt;
  boost::property_tree::ptree config_schema_pt;
  std::istringstream img_is;
  std::istringstream config_is;

  if (config->ClippingCuboid() != "")
    {
      try
        {
          img_is.str(im->ClippingCuboid());
          config_is.str(config->ClippingCuboid());
          img_is.seekg(0, img_is.beg);
          config_is.seekg(0, config_is.beg);
          boost::property_tree::read_json(img_is, img_schema_pt);
          boost::property_tree::read_json(config_is, config_schema_pt);

          if (img_schema_pt != config_schema_pt)
            {
              LOG(WARNING) << "Setting Clipping Cuboid Schema to: "
                           << config->ClippingCuboid();

              this->_XCallImager("setParameter",
                                 "ClippingCuboid",
                                 config->ClippingCuboid());
            }
        }
      catch (const std::exception& ex)
        {
          LOG(WARNING) << "Error parsing Clipping Cuboid JSON: "
                       << ex.what();
          LOG(WARNING) << "On-camera JSON: "
                       << im->ClippingCuboid();
          LOG(WARNING) << "Desired JSON: "
                       << config->ClippingCuboid();
        }
    }
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetTemporalFilterParameters()
{
  return o3d3xx::value_struct_to_map(
    this->_XCallTemporalFilter("getAllParameters"));
}

std::unordered_map<std::string,
                   std::unordered_map<std::string, std::string> >
o3d3xx::Camera::GetTemporalFilterParameterLimits()
{
  return o3d3xx::value_struct_to_map_of_maps(
    this->_XCallTemporalFilter("getAllParameterLimits"));
}

o3d3xx::TemporalFilterConfig::Ptr
o3d3xx::Camera::GetTemporalFilterConfig()
{
  o3d3xx::ImagerConfig::Ptr im = this->GetImagerConfig();
  int filter_type = im->TemporalFilterType();

  std::unordered_map<std::string, std::string> params =
    this->GetTemporalFilterParameters();

  o3d3xx::TemporalFilterConfig::Ptr filt;

  switch (filter_type)
    {
    case static_cast<int>(
      o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER):
        filt = std::make_shared<o3d3xx::TemporalMeanFilterConfig>();
        filt->SetNumberOfImages(std::stoi(params.at("NumberOfImages")));
        break;

    case static_cast<int>(
      o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER):
        filt =
          std::make_shared<o3d3xx::TemporalAdaptiveExponentialFilterConfig>();
        break;

    default:
      filt = std::make_shared<o3d3xx::TemporalFilterConfig>();
      break;
    }

  return filt;
}

void
o3d3xx::Camera::SetTemporalFilterConfig(
  const o3d3xx::TemporalFilterConfig* config)
{
  DLOG(INFO) << "Setting Temporal Filter Config for Type="
             << config->Type();

  o3d3xx::ImagerConfig::Ptr im = this->GetImagerConfig();
  im->SetTemporalFilterType(config->Type());
  this->SetImagerConfig(im.get());

  if (config->Type() ==
      static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER))
    {
      this->_XCallTemporalFilter("setParameter",
                                 "NumberOfImages", config->NumberOfImages());
    }
}

std::unordered_map<std::string, std::string>
o3d3xx::Camera::GetSpatialFilterParameters()
{
  return o3d3xx::value_struct_to_map(
    this->_XCallSpatialFilter("getAllParameters"));
}

std::unordered_map<std::string,
                   std::unordered_map<std::string, std::string> >
o3d3xx::Camera::GetSpatialFilterParameterLimits()
{
  return o3d3xx::value_struct_to_map_of_maps(
    this->_XCallSpatialFilter("getAllParameterLimits"));
}

o3d3xx::SpatialFilterConfig::Ptr
o3d3xx::Camera::GetSpatialFilterConfig()
{
  o3d3xx::ImagerConfig::Ptr im = this->GetImagerConfig();
  int filter_type = im->SpatialFilterType();

  std::unordered_map<std::string, std::string> params =
    this->GetSpatialFilterParameters();

  o3d3xx::SpatialFilterConfig::Ptr filt;

  switch (filter_type)
    {
    case static_cast<int>(o3d3xx::Camera::spatial_filter::MEDIAN_FILTER):
      filt = std::make_shared<o3d3xx::SpatialMedianFilterConfig>();
      filt->SetMaskSize(std::stoi(params.at("MaskSize")));
      break;

    case static_cast<int>(o3d3xx::Camera::spatial_filter::MEAN_FILTER):
      filt = std::make_shared<o3d3xx::SpatialMeanFilterConfig>();
      filt->SetMaskSize(std::stoi(params.at("MaskSize")));
      break;

    case static_cast<int>(o3d3xx::Camera::spatial_filter::BILATERAL_FILTER):
      filt = std::make_shared<o3d3xx::SpatialBilateralFilterConfig>();
      filt->SetMaskSize(std::stoi(params.at("MaskSize")));
      break;

    default:
      filt = std::make_shared<o3d3xx::SpatialFilterConfig>();
      break;
    }

  return filt;
}

void
o3d3xx::Camera::SetSpatialFilterConfig(
  const o3d3xx::SpatialFilterConfig* config)
{
  DLOG(INFO) << "Setting Spatial Filter Config for Type="
             << config->Type();

  o3d3xx::ImagerConfig::Ptr im = this->GetImagerConfig();
  im->SetSpatialFilterType(config->Type());
  this->SetImagerConfig(im.get());

  if (config->Type() ==
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF))
    {
      return;
    }

  this->_XCallSpatialFilter("setParameter",
                            "MaskSize", config->MaskSize());
}

std::string
o3d3xx::Camera::ToJSON()
{
  bool do_cancel = false;
  std::string root = "o3d3xx";
  boost::property_tree::ptree pt;
  pt.put(root + "." + std::string(O3D3XX_LIBRARY_NAME),
         O3D3XX_VERSION);

  std::ostringstream time_buf;
  std::time_t t = std::time(nullptr);
  time_buf << std::asctime(std::localtime(&t));
  std::string time_str = time_buf.str();
  time_str.erase(std::remove(time_str.begin(), time_str.end(), '\n'),
                 time_str.end());
  pt.put(root + ".Date", time_str);

  try
    {
      if (this->GetSessionID() == "")
        {
          this->RequestSession();
          do_cancel = true;
        }

      // serialize the hardware info
      boost::property_tree::ptree hw_pt;
      std::unordered_map<std::string, std::string> hwinfo =
        this->GetHWInfo();

      for (auto& kv : hwinfo)
        {
          hw_pt.put(kv.first, kv.second);
        }

      pt.put_child(root + ".HWInfo", hw_pt);

      // serialize the software info
      boost::property_tree::ptree sw_pt;
      std::unordered_map<std::string, std::string> swinfo =
        this->GetSWVersion();

      for (auto& kv : swinfo)
        {
          sw_pt.put(kv.first, kv.second);
        }

      pt.put_child(root + ".SWVersion", sw_pt);

      // enter "edit mode" to get access to device, net, app, and imager
      // configurations
      this->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

      // device configuration
      o3d3xx::DeviceConfig::Ptr dev = this->GetDeviceConfig();
      std::istringstream dev_in(dev->ToJSON());
      boost::property_tree::ptree dev_pt;
      boost::property_tree::read_json(dev_in, dev_pt);
      pt.put_child(root + ".Device", dev_pt);

      // network configuration
      o3d3xx::NetConfig::Ptr net = this->GetNetConfig();
      std::istringstream net_in(net->ToJSON());
      boost::property_tree::ptree net_pt;
      boost::property_tree::read_json(net_in, net_pt);
      pt.put_child(root + ".Net", net_pt);

      // applications
      boost::property_tree::ptree apps_pt;

      for (auto& app : this->GetApplicationList())
        {
          this->EditApplication(app.index);
          o3d3xx::AppConfig::Ptr app_ptr = this->GetAppConfig();
          std::istringstream app_in(app_ptr->ToJSON());
          boost::property_tree::ptree app_pt;
          boost::property_tree::read_json(app_in, app_pt);
          app_pt.put("Index", app.index);
          app_pt.put("Id", app.id);

          // imager
          o3d3xx::ImagerConfig::Ptr im_ptr = this->GetImagerConfig();
          std::istringstream im_in(im_ptr->ToJSON());
          boost::property_tree::ptree im_pt;
          boost::property_tree::read_json(im_in, im_pt);

          // spatial filter
          o3d3xx::SpatialFilterConfig::Ptr sf_ptr =
            this->GetSpatialFilterConfig();
          std::istringstream sf_in(sf_ptr->ToJSON());
          boost::property_tree::ptree sf_pt;
          boost::property_tree::read_json(sf_in, sf_pt);
          im_pt.put_child("SpatialFilter", sf_pt);

          // temporal filter
          o3d3xx::TemporalFilterConfig::Ptr tf_ptr =
            this->GetTemporalFilterConfig();
          std::istringstream tf_in(tf_ptr->ToJSON());
          boost::property_tree::ptree tf_pt;
          boost::property_tree::read_json(tf_in, tf_pt);
          im_pt.put_child("TemporalFilter", tf_pt);

          // now add the imager and filters to the app
          app_pt.put_child("Imager", im_pt);
          apps_pt.push_back(std::make_pair("", app_pt));

          this->StopEditingApplication();
        }

      pt.put_child(root + ".Apps", apps_pt);
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << ex.what();
      this->CancelSession();
      throw ex;
    }


  if (do_cancel)
    {
      this->CancelSession();
    }

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

void
o3d3xx::Camera::FromJSON(const std::string& json)
{
  bool do_cancel = false;

  boost::property_tree::ptree pt;
  std::istringstream is(json);
  is.seekg(0, is.beg);
  boost::property_tree::read_json(is, pt);

  int desired_active_application = -1;

  try
    {
      if (this->GetSessionID() == "")
        {
          this->RequestSession();
          do_cancel = true;
        }

      this->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

      //
      // device config
      //
      try
        {
          boost::property_tree::ptree dev_pt = pt.get_child("o3d3xx.Device");
          std::ostringstream dev_buf;
          boost::property_tree::write_json(dev_buf, dev_pt);
          o3d3xx::DeviceConfig::Ptr dev =
            o3d3xx::DeviceConfig::FromJSON(dev_buf.str(),
                                           this->GetDeviceConfig());
          this->SetDeviceConfig(dev.get());
          this->SaveDevice();

          try
            {
              desired_active_application =
                dev_pt.get<int>("ActiveApplication");
            }
          catch (const std::exception& ex)
            {
              // no "ActiveApplication" specified
            }
        }
      catch (const boost::property_tree::ptree_bad_path& ex)
        {
          LOG(WARNING) << "In `FromJSON(...)', skipping `Device' section";
          LOG(WARNING) << "ptree_bad_path: " << ex.what();
        }

      //
      // app config
      //
      std::set<int> app_indices;
      for (auto& app : this->GetApplicationList())
        {
          app_indices.insert(app.index);
        }

      try
        {
          boost::property_tree::ptree apps_pt = pt.get_child("o3d3xx.Apps");
          for (auto& kv : apps_pt)
            {
              boost::property_tree::ptree app_pt = kv.second;
              std::ostringstream app_buff;
              boost::property_tree::write_json(app_buff, app_pt);

              int index = -1;

              try
                {
                  index = app_pt.get<int>("Index");
                }
              catch (const std::exception& ex)
                {
                  // no index, so, we will have to create the application
                }

              if (app_indices.find(index) == app_indices.end())
                {
                  index = this->CreateApplication();
                }

              this->EditApplication(index);
              o3d3xx::AppConfig::Ptr app =
                o3d3xx::AppConfig::FromJSON(app_buff.str(),
                                            this->GetAppConfig());

              this->SetAppConfig(app.get());
              this->SaveApp();

              try
                {
                  boost::property_tree::ptree im_pt =
                    app_pt.get_child("Imager");
                  std::ostringstream im_buff;
                  boost::property_tree::write_json(im_buff, im_pt);
                  o3d3xx::ImagerConfig::Ptr im =
                    o3d3xx::ImagerConfig::FromJSON(im_buff.str(),
                                                   this->GetImagerConfig());

                  o3d3xx::ImagerConfig::Ptr current_im =
                    this->GetImagerConfig();

                  if (current_im->Type() != im->Type())
                    {
                      this->ChangeImagerType(im->Type());
                    }

                  this->SetImagerConfig(im.get());

                  // add in the spatial filter
                  try
                    {
                      boost::property_tree::ptree sf_pt =
                        im_pt.get_child("SpatialFilter");
                      std::ostringstream sf_buff;
                      boost::property_tree::write_json(sf_buff, sf_pt);
                      o3d3xx::SpatialFilterConfig::Ptr sf_filt =
                        o3d3xx::SpatialFilterConfig::FromJSON(
                          sf_buff.str(), this->GetSpatialFilterConfig());

                      this->SetSpatialFilterConfig(sf_filt.get());
                    }
                  catch (const boost::property_tree::ptree_bad_path& path_ex)
                    {
                      LOG(WARNING) << "In `FromJSON(...)', "
                                   << "skipping `SpatialFilter' section.";
                      LOG(WARNING) << "ptree_bad_path: " << path_ex.what();
                    }

                  // add in the temporal filter
                  try
                    {
                      boost::property_tree::ptree tf_pt =
                        im_pt.get_child("TemporalFilter");
                      std::ostringstream tf_buff;
                      boost::property_tree::write_json(tf_buff, tf_pt);
                      o3d3xx::TemporalFilterConfig::Ptr tf_filt =
                        o3d3xx::TemporalFilterConfig::FromJSON(
                          tf_buff.str(), this->GetTemporalFilterConfig());

                      this->SetTemporalFilterConfig(tf_filt.get());
                    }
                  catch (const boost::property_tree::ptree_bad_path& path_ex)
                    {
                      LOG(WARNING) << "In `FromJSON(...)', "
                                   << "skipping `TemporalFilter' section.";
                      LOG(WARNING) << "ptree_bad_path: " << path_ex.what();
                    }

                  this->SaveApp();
                }
              catch (const boost::property_tree::ptree_bad_path& path_ex)
                {
                  LOG(WARNING) << "In `FromJSON(...)', "
                               << "skipping `Imager' section for app";
                  LOG(WARNING) << "ptree_bad_path: " << path_ex.what();
                }

              this->StopEditingApplication();
            }
        }
      catch (const boost::property_tree::ptree_bad_path& ex)
        {
          LOG(WARNING) << "In `FromJSON(...)', skipping `Apps' section";
          LOG(WARNING) << "ptree_bad_path: " << ex.what();
        }

      // set the active application
      if (desired_active_application > 0)
        {
          o3d3xx::DeviceConfig::Ptr newdev = this->GetDeviceConfig();
          newdev->SetActiveApplication(desired_active_application);
          this->SetDeviceConfig(newdev.get());
          this->SaveDevice();
        }

      //
      // net config
      //
      try
        {
          boost::property_tree::ptree net_pt = pt.get_child("o3d3xx.Net");
          std::ostringstream net_buf;
          boost::property_tree::write_json(net_buf, net_pt);
          o3d3xx::NetConfig::Ptr net =
            o3d3xx::NetConfig::FromJSON(net_buf.str(), this->GetNetConfig());
          bool has_changed = false;
          this->SetNetConfig(net.get(), &has_changed);
          if (has_changed)
            {
              this->SaveNet();
            }
        }
      catch (const boost::property_tree::ptree_bad_path& ex)
        {
          LOG(WARNING) << "In `FromJSON(...)', skipping `Net' section";
          LOG(WARNING) << "ptree_bad_path: " << ex.what();
        }
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << ex.what();
      this->CancelSession();
      throw ex;
    }

  if (do_cancel)
    {
      this->CancelSession();
    }
}

int
o3d3xx::Camera::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  bool do_cancel = false;
  int index = -1;

  try
    {
      if (this->GetSessionID() == "")
        {
          this->RequestSession();
          do_cancel = true;
        }

      this->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
      // this will base64 encode the data for us
      xmlrpc_c::value_int v_int(this->_XCallSession("importApplication",
                                                    bytes));
      index = v_int.cvalue();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << ex.what();
      this->CancelSession();
      throw ex;
    }

  if (do_cancel)
    {
      this->CancelSession();
    }

  return index;
}

std::vector<std::uint8_t>
o3d3xx::Camera::ExportIFMApp(int idx)
{
  std::vector<std::uint8_t> retval;
  bool do_cancel = false;

  try
    {
      if (this->GetSessionID() == "")
        {
          this->RequestSession();
          do_cancel = true;
        }

      this->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);

      const xmlrpc_c::value_bytestring v_bytes =
        this->_XCallSession("exportApplication", idx);

      retval = v_bytes.vectorUcharValue();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << ex.what();
      this->CancelSession();
      throw ex;
    }

  if (do_cancel)
    {
      this->CancelSession();
    }

  return retval;
}
