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

#include "o3d3xx/imager_config.h"
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx/camera.hpp"
#include "o3d3xx/util.hpp"

o3d3xx::ImagerConfig::ImagerConfig()
  : type_("under5m_low"),
    frame_rate_(5.0),
    clipping_left_(0),
    clipping_top_(0),
    clipping_right_(175),
    clipping_bottom_(131),
    reduce_motion_artifacts_(false),
    spatial_filter_type_(0),
    average_filter_num_pictures_(1)
{ }

o3d3xx::ImagerConfig::ImagerConfig(
  const std::unordered_map<std::string, std::string>& params)
  : ImagerConfig()
{
  for (auto& kv : params)
    {
      try
	{
	  auto func = o3d3xx::ImagerConfig::mutator_map.at(kv.first);
	  func(this, kv.second);
	}
      catch (const std::out_of_range& ex)
	{
	  // we expect this for any read-only params
	}
    }
}

std::string
o3d3xx::ImagerConfig::Type() const noexcept
{
  return this->type_;
}

void
o3d3xx::ImagerConfig::SetType(const std::string& type) noexcept
{
  this->type_ = type;
}

std::string
o3d3xx::ImagerConfig::TypeHash() const noexcept
{
  return this->type_hash_;
}

void
o3d3xx::ImagerConfig::SetTypeHash(const std::string& hash) noexcept
{
  this->type_hash_ = hash;
}

int
o3d3xx::ImagerConfig::ExposureTime() const
{
  if (this->Type() == "under5m_high")
    {
      throw o3d3xx::error_t(O3D3XX_EXPOSURE_TIME_NOT_ACCESSIBLE);
    }

  return this->exposure_time_;
}

void
o3d3xx::ImagerConfig::SetExposureTime(int usecs)
{
  if (this->Type() == "under5m_high")
    {
      throw o3d3xx::error_t(O3D3XX_EXPOSURE_TIME_NOT_ACCESSIBLE);
    }

  this->exposure_time_ = usecs;
}

int
o3d3xx::ImagerConfig::Channel() const noexcept
{
  return this->channel_;
}

void
o3d3xx::ImagerConfig::SetChannel(int channel) noexcept
{
  this->channel_ = channel;
}

double
o3d3xx::ImagerConfig::FrameRate() const noexcept
{
  return this->frame_rate_;
}

void
o3d3xx::ImagerConfig::SetFrameRate(double rate) noexcept
{
  this->frame_rate_ = rate;
}

int
o3d3xx::ImagerConfig::ClippingLeft() const noexcept
{
  return this->clipping_left_;
}

void
o3d3xx::ImagerConfig::SetClippingLeft(int left) noexcept
{
  this->clipping_left_ = left;
}

int
o3d3xx::ImagerConfig::ClippingTop() const noexcept
{
  return this->clipping_top_;
}

void
o3d3xx::ImagerConfig::SetClippingTop(int top) noexcept
{
  this->clipping_top_ = top;
}

int
o3d3xx::ImagerConfig::ClippingRight() const noexcept
{
  return this->clipping_right_;
}

void
o3d3xx::ImagerConfig::SetClippingRight(int right) noexcept
{
  this->clipping_right_ = right;
}

int
o3d3xx::ImagerConfig::ClippingBottom() const noexcept
{
  return this->clipping_bottom_;
}

void
o3d3xx::ImagerConfig::SetClippingBottom(int bottom) noexcept
{
  this->clipping_bottom_ = bottom;
}

bool
o3d3xx::ImagerConfig::ReduceMotionArtifacts() const noexcept
{
  return this->reduce_motion_artifacts_;
}

void
o3d3xx::ImagerConfig::SetReduceMotionArtifacts(bool on) noexcept
{
  this->reduce_motion_artifacts_ = on;
}

int
o3d3xx::ImagerConfig::SpatialFilterType() const noexcept
{
  return this->spatial_filter_type_;
}

void
o3d3xx::ImagerConfig::SetSpatialFilterType(int type) noexcept
{
  this->spatial_filter_type_ = type;
}

int
o3d3xx::ImagerConfig::AverageFilterNumPictures() const noexcept
{
  return this->average_filter_num_pictures_;
}

void
o3d3xx::ImagerConfig::SetAverageFilterNumPictures(int num) noexcept
{
  this->average_filter_num_pictures_ = num;
}

const std::unordered_map<std::string,
			 std::function<void(o3d3xx::ImagerConfig*,
					    const std::string&)> >
o3d3xx::ImagerConfig::mutator_map =
  {
    {"Type",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetType(val); } },

    {"TypeHash",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetTypeHash(val); } },

    {"ExposureTime",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetExposureTime(std::stoi(val)); } },

    {"Channel",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetChannel(std::stoi(val)); } },

    {"FrameRate",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetFrameRate(std::stod(val)); } },

    {"ClippingLeft",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingLeft(std::stoi(val)); }},

    {"ClippingTop",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingTop(std::stoi(val)); }},

    {"ClippingRight",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingRight(std::stoi(val)); }},

    {"ClippingBottom",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingBottom(std::stoi(val)); }},

    {"ReduceMotionArtifacts",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetReduceMotionArtifacts(o3d3xx::stob(val)); }},

    {"SpatialFilterType",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetSpatialFilterType(std::stoi(val)); } },

    {"AverageFilterNumPictures",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetAverageFilterNumPictures(std::stoi(val)); } },

  };

std::string
o3d3xx::ImagerConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("Type", this->Type());
  pt.put("TypeHash", this->TypeHash());
  pt.put("ExposureTime", this->ExposureTime());
  pt.put("Channel", this->Channel());
  pt.put("FrameRate", this->FrameRate());
  pt.put("ClippingLeft", this->ClippingLeft());
  pt.put("ClippingTop", this->ClippingTop());
  pt.put("ClippingRight", this->ClippingRight());
  pt.put("ClippingBottom", this->ClippingBottom());
  pt.put("ReduceMotionArtifacts", this->ReduceMotionArtifacts());
  pt.put("SpatialFilterType", this->SpatialFilterType());
  pt.put("AverageFilterNumPictures", this->AverageFilterNumPictures());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::ImagerConfig::Ptr
o3d3xx::ImagerConfig::FromJSON(const std::string& json)
{
  o3d3xx::ImagerConfig::Ptr im = std::make_shared<o3d3xx::ImagerConfig>();

  boost::property_tree::ptree pt;
  std::istringstream is(json);
  boost::property_tree::read_json(is, pt);

  for (auto& kv : pt)
    {
      try
	{
	  auto func = o3d3xx::ImagerConfig::mutator_map.at(kv.first);
	  func(im.get(), kv.second.data());
	}
      catch (const std::out_of_range& ex)
	{
	  DLOG(WARNING) << "In FromJSON: "
			<< kv.first << "=" << kv.second.data()
			<< ": " << ex.what();
	}
    }

  return im;
}
