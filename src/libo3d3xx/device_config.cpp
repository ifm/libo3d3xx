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

#include "o3d3xx/device_config.h"
#include <exception>
#include <functional>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx/util.hpp"

o3d3xx::DeviceConfig::DeviceConfig()
  : name_("New sensor"),
    description_(""),
    active_application_(0),
    pcic_tcp_port_(50010),
    pcic_protocol_version_(3),
    io_logic_type_(1),
    io_debouncing_(true),
    io_extern_application_switch_(0),
    session_timeout_(30),
    extrinsic_calib_trans_x_(0.0),
    extrinsic_calib_trans_y_(0.0),
    extrinsic_calib_trans_z_(0.0),
    extrinsic_calib_rot_x_(0.0),
    extrinsic_calib_rot_y_(0.0),
    extrinsic_calib_rot_z_(0.0),
    ip_address_config_(0),
    password_activated_(false),
    operating_mode_(0)
{ }

o3d3xx::DeviceConfig::DeviceConfig(
  const std::unordered_map<std::string, std::string>& params)
  : DeviceConfig()
{
  for (auto& kv : params)
    {
      try
        {
          auto func = o3d3xx::DeviceConfig::mutator_map.at(kv.first);
          func(this, kv.second);
        }
      catch (const std::out_of_range& ex)
        {
          // we expect this for the read-only params
        }
      catch (const std::invalid_argument& ia)
        {
          LOG(ERROR) << "Invalid arg for: "
                     << kv.first << "=" << kv.second;
        }
    }
}

std::string
o3d3xx::DeviceConfig::Name() const noexcept
{
  return this->name_;
}

void
o3d3xx::DeviceConfig::SetName(const std::string& name) noexcept
{
  this->name_ = name;
}

std::string
o3d3xx::DeviceConfig::Description() const noexcept
{
  return this->description_;
}

void
o3d3xx::DeviceConfig::SetDescription(const std::string& description) noexcept
{
  this->description_ = description;
}

int
o3d3xx::DeviceConfig::ActiveApplication() const noexcept
{
  return this->active_application_;
}

void
o3d3xx::DeviceConfig::SetActiveApplication(int idx) noexcept
{
  this->active_application_ = idx;
}

int
o3d3xx::DeviceConfig::PcicTCPPort() const noexcept
{
  return this->pcic_tcp_port_;
}

void
o3d3xx::DeviceConfig::SetPcicTCPPort(int port) noexcept
{
  this->pcic_tcp_port_ = port;
}

int
o3d3xx::DeviceConfig::PcicProtocolVersion() const noexcept
{
  return this->pcic_protocol_version_;
}

void
o3d3xx::DeviceConfig::SetPcicProtocolVersion(int version) noexcept
{
  this->pcic_protocol_version_ = version;
}

int
o3d3xx::DeviceConfig::IOLogicType() const noexcept
{
  return this->io_logic_type_;
}

void
o3d3xx::DeviceConfig::SetIOLogicType(int type) noexcept
{
  this->io_logic_type_ = type;
}

bool
o3d3xx::DeviceConfig::IODebouncing() const noexcept
{
  return this->io_debouncing_;
}

void
o3d3xx::DeviceConfig::SetIODebouncing(bool on) noexcept
{
  this->io_debouncing_ = on;
}

int
o3d3xx::DeviceConfig::IOExternApplicationSwitch() const noexcept
{
  return this->io_extern_application_switch_;
}

void
o3d3xx::DeviceConfig::SetIOExternApplicationSwitch(int val) noexcept
{
  this->io_extern_application_switch_ = val;
}

int
o3d3xx::DeviceConfig::SessionTimeout() const noexcept
{
  return this->session_timeout_;
}

void
o3d3xx::DeviceConfig::SetSessionTimeout(int secs) noexcept
{
  this->session_timeout_ = secs;
}

double
o3d3xx::DeviceConfig::ExtrinsicCalibTransX() const noexcept
{
  return this->extrinsic_calib_trans_x_;
}

void
o3d3xx::DeviceConfig::SetExtrinsicCalibTransX(double x) noexcept
{
  this->extrinsic_calib_trans_x_ = x;
}

double
o3d3xx::DeviceConfig::ExtrinsicCalibTransY() const noexcept
{
  return this->extrinsic_calib_trans_y_;
}

void
o3d3xx::DeviceConfig::SetExtrinsicCalibTransY(double y) noexcept
{
  this->extrinsic_calib_trans_y_ = y;
}

double
o3d3xx::DeviceConfig::ExtrinsicCalibTransZ() const noexcept
{
  return this->extrinsic_calib_trans_z_;
}

void
o3d3xx::DeviceConfig::SetExtrinsicCalibTransZ(double z) noexcept
{
  this->extrinsic_calib_trans_z_ = z;
}

double
o3d3xx::DeviceConfig::ExtrinsicCalibRotX() const noexcept
{
  return this->extrinsic_calib_rot_x_;
}

void
o3d3xx::DeviceConfig::SetExtrinsicCalibRotX(double x) noexcept
{
  this->extrinsic_calib_rot_x_ = x;
}

double
o3d3xx::DeviceConfig::ExtrinsicCalibRotY() const noexcept
{
  return this->extrinsic_calib_rot_y_;
}

void
o3d3xx::DeviceConfig::SetExtrinsicCalibRotY(double y) noexcept
{
  this->extrinsic_calib_rot_y_ = y;
}

double
o3d3xx::DeviceConfig::ExtrinsicCalibRotZ() const noexcept
{
  return this->extrinsic_calib_rot_z_;
}

void
o3d3xx::DeviceConfig::SetExtrinsicCalibRotZ(double z) noexcept
{
  this->extrinsic_calib_rot_z_ = z;
}

int
o3d3xx::DeviceConfig::IPAddressConfig() const noexcept
{
  return this->ip_address_config_;
}

bool
o3d3xx::DeviceConfig::PasswordActivated() const noexcept
{
  return this->password_activated_;
}

int
o3d3xx::DeviceConfig::OperatingMode() const noexcept
{
  return this->operating_mode_;
}

std::string
o3d3xx::DeviceConfig::DeviceType() const noexcept
{
  return this->device_type_;
}

std::string
o3d3xx::DeviceConfig::ArticleNumber() const noexcept
{
  return this->article_number_;
}

std::string
o3d3xx::DeviceConfig::ArticleStatus() const noexcept
{
  return this->article_status_;
}

double
o3d3xx::DeviceConfig::Uptime() const noexcept
{
  return this->uptime_;
}

int
o3d3xx::DeviceConfig::ImageTimestampReference() const noexcept
{
  return this->image_timestamp_ref_;
}

double
o3d3xx::DeviceConfig::TemperatureFront1() const noexcept
{
  return this->temp_front1_;
}

double
o3d3xx::DeviceConfig::TemperatureFront2() const noexcept
{
  return this->temp_front2_;
}

double
o3d3xx::DeviceConfig::TemperatureIMX6() const noexcept
{
  return this->temp_imx6_;
}

double
o3d3xx::DeviceConfig::TemperatureIllu() const noexcept
{
  return this->temp_illu_;
}

const std::unordered_map<std::string,
                         std::function<void(o3d3xx::DeviceConfig*,
                                            const std::string&)> >
o3d3xx::DeviceConfig::mutator_map =
  {
    {"Name",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetName(val); } },

    {"Description",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetDescription(val); } },

    {"ActiveApplication",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetActiveApplication(std::stoi(val)); } },

    {"PcicTcpPort",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPcicTCPPort(std::stoi(val)); } },

    {"PcicProtocolVersion",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPcicProtocolVersion(std::stoi(val)); } },

    {"IOLogicType",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIOLogicType(std::stoi(val)); } },

    {"IODebouncing",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIODebouncing(o3d3xx::stob(val)); } },

    {"IOExternApplicationSwitch",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIOExternApplicationSwitch(std::stoi(val)); } },

    {"SessionTimeout",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetSessionTimeout(std::stoi(val)); } },

    {"ExtrinsicCalibTransX",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibTransX(std::stod(val)); } },

    {"ExtrinsicCalibTransY",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibTransY(std::stod(val)); } },

    {"ExtrinsicCalibTransZ",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibTransZ(std::stod(val)); } },

    {"ExtrinsicCalibRotX",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibRotX(std::stod(val)); } },

    {"ExtrinsicCalibRotY",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibRotY(std::stod(val)); } },

    {"ExtrinsicCalibRotZ",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibRotZ(std::stod(val)); } },
  };

std::string
o3d3xx::DeviceConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("Name", this->Name());
  pt.put("Description", this->Description());
  pt.put("ActiveApplication", this->ActiveApplication());
  pt.put("PcicTcpPort", this->PcicTCPPort());
  pt.put("PcicProtocolVersion", this->PcicProtocolVersion());
  pt.put("IOLogicType", this->IOLogicType());
  pt.put("IODebouncing", this->IODebouncing());
  pt.put("IOExternApplicationSwitch", this->IOExternApplicationSwitch());
  pt.put("SessionTimeout", this->SessionTimeout());
  pt.put("ExtrinsicCalibTransX", this->ExtrinsicCalibTransX());
  pt.put("ExtrinsicCalibTransY", this->ExtrinsicCalibTransY());
  pt.put("ExtrinsicCalibTransZ", this->ExtrinsicCalibTransZ());
  pt.put("ExtrinsicCalibRotX", this->ExtrinsicCalibRotX());
  pt.put("ExtrinsicCalibRotY", this->ExtrinsicCalibRotY());
  pt.put("ExtrinsicCalibRotZ", this->ExtrinsicCalibRotZ());
  pt.put("IPAddressConfig", this->IPAddressConfig());
  pt.put("PasswordActivated", this->PasswordActivated());
  pt.put("OperatingMode", this->OperatingMode());
  pt.put("DeviceType", this->DeviceType());
  pt.put("ArticleNumber", this->ArticleNumber());
  pt.put("ArticleStatus", this->ArticleStatus());
  pt.put("UpTime", this->Uptime());
  pt.put("ImageTimestampReference", this->ImageTimestampReference());
  pt.put("TemperatureFront1", this->TemperatureFront1());
  pt.put("TemperatureFront2", this->TemperatureFront2());
  pt.put("TemperatureIMX6", this->TemperatureIMX6());
  pt.put("TemperatureIllu", this->TemperatureIllu());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::DeviceConfig::Ptr
o3d3xx::DeviceConfig::FromJSON(const std::string& json)
{
  o3d3xx::DeviceConfig::Ptr dev =
    o3d3xx::DeviceConfig::Ptr(new o3d3xx::DeviceConfig());

  boost::property_tree::ptree pt;
  std::istringstream is(json);
  boost::property_tree::read_json(is, pt);

  for (auto& kv : pt)
    {
      try
        {
          auto func = o3d3xx::DeviceConfig::mutator_map.at(kv.first);
          func(dev.get(), kv.second.data());
        }
      catch (const std::out_of_range& ex)
        {
          // we expect this for read-only values
          DLOG(WARNING) << "In FromJSON: "
                        << kv.first << "=" << kv.second.data()
                        << ": "         << ex.what();
        }
    }

  return dev;
}
