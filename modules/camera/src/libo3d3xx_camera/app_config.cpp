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

#include "o3d3xx_camera/app_config.h"
#include <exception>
#include <functional>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx_camera/camera.hpp"
#include "o3d3xx_camera/util.h"

o3d3xx::AppConfig::AppConfig()
  : name_("New application"),
    description_(""),
    trigger_mode_(1),
    pcic_tcp_result_output_enabled_(true),
    pcic_tcp_result_schema_(""),
    pcic_eip_result_schema_(""),
    logic_graph_(""),
    type_("")
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

std::string
o3d3xx::AppConfig::PcicEipResultSchema() const noexcept
{
  return this->pcic_eip_result_schema_;
}

void
o3d3xx::AppConfig::SetPcicEipResultSchema(const std::string& schema) noexcept
{
  this->pcic_eip_result_schema_ = schema;
}

std::string
o3d3xx::AppConfig::LogicGraph() const noexcept
{
  return this->logic_graph_;
}

void
o3d3xx::AppConfig::SetLogicGraph(const std::string& graph) noexcept
{
  this->logic_graph_ = graph;
}

std::string
o3d3xx::AppConfig::Type() const noexcept
{
  return this->type_;
}

void
o3d3xx::AppConfig::SetType(const std::string& type) noexcept
{
  this->type_ = type;
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

    {"PcicEipResultSchema",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetPcicEipResultSchema(val); } },

    {"LogicGraph",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetLogicGraph(val); } },

    {"Type",
     [](o3d3xx::AppConfig* app, const std::string& val)
     { app->SetType(val); } },
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
  pt.put("PcicEipResultSchema", this->PcicEipResultSchema());
  pt.put("LogicGraph", this->LogicGraph());
  pt.put("Type", this->Type());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::AppConfig::Ptr
o3d3xx::AppConfig::FromJSON(const std::string& json,
                            o3d3xx::AppConfig::Ptr app_ptr)
{
  o3d3xx::AppConfig::Ptr app =
    app_ptr ? app_ptr : std::make_shared<o3d3xx::AppConfig>();

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
