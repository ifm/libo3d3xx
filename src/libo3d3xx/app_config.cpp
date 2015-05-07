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

#include "o3d3xx/app_config.h"
#include <exception>
#include <functional>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx/camera.hpp"
#include "o3d3xx/util.hpp"

o3d3xx::AppConfig::AppConfig()
  : name_("New application"),
    description_(""),
    trigger_mode_(1),
    pcic_tcp_result_output_enabled_(true),
    pcic_tcp_result_schema_("")
{ }

o3d3xx::AppConfig::AppConfig(
  const std::unordered_map<std::string, std::string>& params)
  : AppConfig()
{
  for (auto& kv : params)
    {
      try
	{
	  auto func = o3d3xx::AppConfig::mutator_map.at(kv.first);
	  func(this, kv.second);
	}
      catch (const std::out_of_range& ex)
	{
	  // we expect this for any read-only params
	}
      catch (const std::invalid_argument& ia)
	{
	  LOG(ERROR) << "Invalid arg for: "
		     << kv.first << "=" << kv.second;
	}
    }
}

std::string
o3d3xx::AppConfig::Name() const noexcept
{
  return this->name_;
}

void
o3d3xx::AppConfig::SetName(const std::string& name) noexcept
{
  this->name_ = name;
}

std::string
o3d3xx::AppConfig::Description() const noexcept
{
  return this->description_;
}

void
o3d3xx::AppConfig::SetDescription(const std::string& descr) noexcept
{
  this->description_ = descr;
}

int
o3d3xx::AppConfig::TriggerMode() const noexcept
{
  return this->trigger_mode_;
}

void
o3d3xx::AppConfig::SetTriggerMode(int mode) noexcept
{
  this->trigger_mode_ = mode;
}

bool
o3d3xx::AppConfig::PcicTcpResultOutputEnabled() const noexcept
{
  return this->pcic_tcp_result_output_enabled_;
}

void
o3d3xx::AppConfig::SetPcicTcpResultOutputEnabled(bool on) noexcept
{
  this->pcic_tcp_result_output_enabled_ = on;
}

std::string
o3d3xx::AppConfig::PcicTcpResultSchema() const noexcept
{
  return this->pcic_tcp_result_schema_;
}

void
o3d3xx::AppConfig::SetPcicTcpResultSchema(const std::string& schema) noexcept
{
  this->pcic_tcp_result_schema_ = schema;
}

const std::unordered_map<std::string,
			 std::function<void(o3d3xx::AppConfig*,
					    const std::string&)> >
o3d3xx::AppConfig::mutator_map =
  {
    {"Name",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetName(val); } },

    {"Description",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetDescription(val); } },

    {"TriggerMode",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetTriggerMode(std::stoi(val)); } },

    {"PcicTcpResultOutputEnabled",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetPcicTcpResultOutputEnabled(o3d3xx::stob(val)); } },

    {"PcicTcpResultSchema",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetPcicTcpResultSchema(val); } },
  };

std::string
o3d3xx::AppConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("Name", this->Name());
  pt.put("Description", this->Description());
  pt.put("TriggerMode", this->TriggerMode());
  pt.put("PcicTcpResultOutputEnabled", this->PcicTcpResultOutputEnabled());
  pt.put("PcicTcpResultSchema", this->PcicTcpResultSchema());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::AppConfig::Ptr
o3d3xx::AppConfig::FromJSON(const std::string& json)
{
  o3d3xx::AppConfig::Ptr app =
    o3d3xx::AppConfig::Ptr(new o3d3xx::AppConfig());

  boost::property_tree::ptree pt;
  std::istringstream is(json);
  boost::property_tree::read_json(is, pt);

  for (auto& kv : pt)
    {
      try
	{
	  auto func = o3d3xx::AppConfig::mutator_map.at(kv.first);
	  func(app.get(), kv.second.data());
	}
      catch (const std::out_of_range& ex)
	{
	  DLOG(WARNING) << "In FromJSON: "
			<< kv.first << "=" << kv.second.data()
			<< ": " << ex.what();
	}
    }

  return app;
}
