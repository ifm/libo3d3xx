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

#include "o3d3xx_camera/device_config.h"
#include <exception>
#include <functional>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx_camera/util.h"

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
    service_report_passed_buffer_(15),
    service_report_failed_buffer_(15),
    extrinsic_calib_trans_x_(0.0),
    extrinsic_calib_trans_y_(0.0),
    extrinsic_calib_trans_z_(0.0),
    extrinsic_calib_rot_x_(0.0),
    extrinsic_calib_rot_y_(0.0),
    extrinsic_calib_rot_z_(0.0),
    evaluation_finished_min_hold_time_(10),
    save_restore_stats_on_appl_switch_(true),
    ip_address_config_(0),
    password_activated_(false),
    operating_mode_(0),
    pnio_device_name_(""),
    ethernet_field_bus_(0),
    ethernet_field_bus_endianness_(0),
    enable_acquisition_finished_pcic_(false),
    eip_producing_size_(0),
    eip_consuming_size_(0),
    pcic_tcp_schema_auto_update_(false)
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

int
o3d3xx::DeviceConfig::ServiceReportPassedBuffer() const noexcept
{
  return this->service_report_passed_buffer_;
}

void
o3d3xx::DeviceConfig::SetServiceReportPassedBuffer(int len) noexcept
{
  this->service_report_passed_buffer_ = len;
}

int
o3d3xx::DeviceConfig::ServiceReportFailedBuffer() const noexcept
{
  return this->service_report_failed_buffer_;
}

void
o3d3xx::DeviceConfig::SetServiceReportFailedBuffer(int len) noexcept
{
  this->service_report_failed_buffer_ = len;
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
o3d3xx::DeviceConfig::EvaluationFinishedMinHoldTime() const noexcept
{
  return this->evaluation_finished_min_hold_time_;
}

void
o3d3xx::DeviceConfig::SetEvaluationFinishedMinHoldTime(int t) noexcept
{
  this->evaluation_finished_min_hold_time_ = t;
}

bool
o3d3xx::DeviceConfig::SaveRestoreStatsOnApplSwitch() const noexcept
{
  return this->save_restore_stats_on_appl_switch_;
}

void
o3d3xx::DeviceConfig::SetSaveRestoreStatsOnApplSwitch(bool on) noexcept
{
  this->save_restore_stats_on_appl_switch_ = on;
}

int
o3d3xx::DeviceConfig::IPAddressConfig() const noexcept
{
  return this->ip_address_config_;
}

void
o3d3xx::DeviceConfig::SetIPAddressConfig(int c) noexcept
{
  this->ip_address_config_ = c;
}

bool
o3d3xx::DeviceConfig::PasswordActivated() const noexcept
{
  return this->password_activated_;
}

void
o3d3xx::DeviceConfig::SetPasswordActivated(bool b) noexcept
{
  this->password_activated_ = b;
}

int
o3d3xx::DeviceConfig::OperatingMode() const noexcept
{
  return this->operating_mode_;
}

void
o3d3xx::DeviceConfig::SetOperatingMode(int i) noexcept
{
  this->operating_mode_ = i;
}

std::string
o3d3xx::DeviceConfig::DeviceType() const noexcept
{
  return this->device_type_;
}

void
o3d3xx::DeviceConfig::SetDeviceType(const std::string& s) noexcept
{
  this->device_type_ = s;
}

std::string
o3d3xx::DeviceConfig::ArticleNumber() const noexcept
{
  return this->article_number_;
}

void
o3d3xx::DeviceConfig::SetArticleNumber(const std::string& s) noexcept
{
  this->article_number_ = s;
}

std::string
o3d3xx::DeviceConfig::ArticleStatus() const noexcept
{
  return this->article_status_;
}

void
o3d3xx::DeviceConfig::SetArticleStatus(const std::string& s) noexcept
{
  this->article_status_ = s;
}

double
o3d3xx::DeviceConfig::Uptime() const noexcept
{
  return this->uptime_;
}

void
o3d3xx::DeviceConfig::SetUptime(double d) noexcept
{
  this->uptime_ = d;
}

int
o3d3xx::DeviceConfig::ImageTimestampReference() const noexcept
{
  return this->image_timestamp_ref_;
}

void
o3d3xx::DeviceConfig::SetImageTimestampReference(int i) noexcept
{
  this->image_timestamp_ref_ = i;
}

double
o3d3xx::DeviceConfig::TemperatureFront1() const noexcept
{
  return this->temp_front1_;
}

void
o3d3xx::DeviceConfig::SetTemperatureFront1(double d) noexcept
{
  this->temp_front1_ = d;
}

double
o3d3xx::DeviceConfig::TemperatureFront2() const noexcept
{
  return this->temp_front2_;
}

void
o3d3xx::DeviceConfig::SetTemperatureFront2(double d) noexcept
{
  this->temp_front2_ = d;
}

double
o3d3xx::DeviceConfig::TemperatureIMX6() const noexcept
{
  return this->temp_imx6_;
}

void
o3d3xx::DeviceConfig::SetTemperatureIMX6(double d) noexcept
{
  this->temp_imx6_ = d;
}

double
o3d3xx::DeviceConfig::TemperatureIllu() const noexcept
{
  return this->temp_illu_;
}

void
o3d3xx::DeviceConfig::SetTemperatureIllu(double d) noexcept
{
  this->temp_illu_ = d;
}

std::string
o3d3xx::DeviceConfig::PNIODeviceName() const noexcept
{
  return this->pnio_device_name_;
}

void
o3d3xx::DeviceConfig::SetPNIODeviceName(const std::string& s) noexcept
{
  this->pnio_device_name_ = s;
}

int
o3d3xx::DeviceConfig::EthernetFieldBus() const noexcept
{
  return this->ethernet_field_bus_;
}

void
o3d3xx::DeviceConfig::SetEthernetFieldBus(int i) noexcept
{
  this->ethernet_field_bus_ = i;
}

int
o3d3xx::DeviceConfig::EthernetFieldBusEndianness() const noexcept
{
  return this->ethernet_field_bus_endianness_;
}

void
o3d3xx::DeviceConfig::SetEthernetFieldBusEndianness(int i) noexcept
{
  this->ethernet_field_bus_endianness_ = i;
}

bool
o3d3xx::DeviceConfig::EnableAcquisitionFinishedPCIC() const noexcept
{
  return this->enable_acquisition_finished_pcic_;
}

void
o3d3xx::DeviceConfig::SetEnableAcquisitionFinishedPCIC(bool on) noexcept
{
  this->enable_acquisition_finished_pcic_ = on;
}

int
o3d3xx::DeviceConfig::EIPProducingSize() const noexcept
{
  return this->eip_producing_size_;
}

void
o3d3xx::DeviceConfig::SetEIPProducingSize(int i) noexcept
{
  this->eip_producing_size_ = i;
}

int
o3d3xx::DeviceConfig::EIPConsumingSize() const noexcept
{
  return this->eip_consuming_size_;
}

void
o3d3xx::DeviceConfig::SetEIPConsumingSize(int i) noexcept
{
  this->eip_consuming_size_ = i;
}

bool
o3d3xx::DeviceConfig::PcicTcpSchemaAutoUpdate() const noexcept
{
  return this->pcic_tcp_schema_auto_update_;
}

void
o3d3xx::DeviceConfig::SetPcicTcpSchemaAutoUpdate(bool on) noexcept
{
  this->pcic_tcp_schema_auto_update_ = on;
}

const std::unordered_map<std::string,
                         std::function<void(o3d3xx::DeviceConfig*,
                                            const std::string&)> >
o3d3xx::DeviceConfig::mutator_map =
  {
    {"Name",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetName(val); }},

    {"Description",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetDescription(val); }},

    {"ActiveApplication",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetActiveApplication(std::stoi(val)); }},

    {"PcicTcpPort",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPcicTCPPort(std::stoi(val)); }},

    {"PcicProtocolVersion",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPcicProtocolVersion(std::stoi(val)); }},

    {"IOLogicType",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIOLogicType(std::stoi(val)); }},

    {"IODebouncing",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIODebouncing(o3d3xx::stob(val)); }},

    {"IOExternApplicationSwitch",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIOExternApplicationSwitch(std::stoi(val)); }},

    {"SessionTimeout",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetSessionTimeout(std::stoi(val)); }},

    {"ServiceReportPassedBuffer",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetServiceReportPassedBuffer(std::stoi(val)); }},

    {"ServiceReportFailedBuffer",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetServiceReportFailedBuffer(std::stoi(val)); }},

    {"ExtrinsicCalibTransX",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibTransX(std::stod(val)); }},

    {"ExtrinsicCalibTransY",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibTransY(std::stod(val)); }},

    {"ExtrinsicCalibTransZ",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibTransZ(std::stod(val)); }},

    {"ExtrinsicCalibRotX",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibRotX(std::stod(val)); }},

    {"ExtrinsicCalibRotY",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibRotY(std::stod(val)); }},

    {"ExtrinsicCalibRotZ",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetExtrinsicCalibRotZ(std::stod(val)); }},

    {"EvaluationFinishedMinHoldTime",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetEvaluationFinishedMinHoldTime(std::stoi(val)); }},

    {"SaveRestoreStatsOnApplSwitch",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetSaveRestoreStatsOnApplSwitch(o3d3xx::stob(val)); }},

    {"EnableAcquisitionFinishedPCIC",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetEnableAcquisitionFinishedPCIC(o3d3xx::stob(val)); }},

    //
    // Read-only parameters on the camera
    //

    {"IPAddressConfig",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetIPAddressConfig(std::stoi(val)); }},

    {"PasswordActivated",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPasswordActivated(o3d3xx::stob(val)); }},

    {"OperatingMode",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetOperatingMode(std::stoi(val)); }},

    {"DeviceType",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetDeviceType(val); }},

    {"ArticleNumber",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetArticleNumber(val); }},

    {"ArticleStatus",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetArticleStatus(val); }},

    {"UpTime",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetUptime(std::stod(val)); }},

    {"ImageTimestampReference",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetImageTimestampReference(std::stoi(val)); }},

    {"TemperatureFront1",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetTemperatureFront1(std::stod(val)); }},

    {"TemperatureFront2",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetTemperatureFront2(std::stod(val)); }},

    {"TemperatureIMX6",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetTemperatureIMX6(std::stod(val)); }},

    {"TemperatureIllu",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetTemperatureIllu(std::stod(val)); }},

    {"PNIODevicename",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPNIODeviceName(val); }},

    {"EthernetFieldBus",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetEthernetFieldBus(std::stoi(val)); }},

    {"EthernetFieldBusEndianness",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetEthernetFieldBusEndianness(std::stoi(val)); }},

    {"EIPProducingSize",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetEIPProducingSize(std::stoi(val)); }},

    {"EIPConsumingSize",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetEIPConsumingSize(std::stoi(val)); }},

    {"PcicTcpSchemaAutoUpdate",
     [](o3d3xx::DeviceConfig* dev, const std::string& val)
     { dev->SetPcicTcpSchemaAutoUpdate(o3d3xx::stob(val)); }}
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
  pt.put("ServiceReportPassedBuffer", this->ServiceReportPassedBuffer());
  pt.put("ServiceReportFailedBuffer", this->ServiceReportFailedBuffer());
  pt.put("ExtrinsicCalibTransX", this->ExtrinsicCalibTransX());
  pt.put("ExtrinsicCalibTransY", this->ExtrinsicCalibTransY());
  pt.put("ExtrinsicCalibTransZ", this->ExtrinsicCalibTransZ());
  pt.put("ExtrinsicCalibRotX", this->ExtrinsicCalibRotX());
  pt.put("ExtrinsicCalibRotY", this->ExtrinsicCalibRotY());
  pt.put("ExtrinsicCalibRotZ", this->ExtrinsicCalibRotZ());
  pt.put("EvaluationFinishedMinHoldTime",
         this->EvaluationFinishedMinHoldTime());
  pt.put("SaveRestoreStatsOnApplSwitch",
         this->SaveRestoreStatsOnApplSwitch());
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
  pt.put("PNIODeviceName", this->PNIODeviceName());
  pt.put("EthernetFieldBus", this->EthernetFieldBus());
  pt.put("EthernetFieldBusEndianness", this->EthernetFieldBusEndianness());
  pt.put("EnableAcquisitionFinishedPCIC",
         this->EnableAcquisitionFinishedPCIC());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::DeviceConfig::Ptr
o3d3xx::DeviceConfig::FromJSON(const std::string& json,
                               o3d3xx::DeviceConfig::Ptr devp)
{
  o3d3xx::DeviceConfig::Ptr dev =
    devp ? devp : std::make_shared<o3d3xx::DeviceConfig>();

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
