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
#include <boost/algorithm/string/predicate.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx/camera.hpp"
#include "o3d3xx/util.hpp"

o3d3xx::ImagerConfig::ImagerConfig()
  : channel_(0),
    clipping_bottom_(131),
    clipping_left_(0),
    clipping_right_(175),
    clipping_top_(0),
    exposure_time_(1000),
    exposure_time_ratio_(40),
    frame_rate_(5.0),
    minimum_amplitude_(0),
    reduce_motion_artifacts_(false),
    spatial_filter_type_(
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF)),
    symmetry_threshold_(0),
    temporal_filter_type_(
      static_cast<int>(o3d3xx::Camera::temporal_filter::OFF)),
    three_freq_max_3f_line_dist_percentage_(0),
    three_freq_max_2f_line_dist_percentage_(0),
    two_freq_max_line_dist_percentage_(0),
    type_("under5m_low"),
    type_hash_("")
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
	  LOG(WARNING) << "In ImagerConfig ctor, "
		       << "parameter not present in mutator map: "
		       << ex.what();
	}
      catch (const std::invalid_argument& ia)
	{
	  LOG(ERROR) << "Invalid arg for: "
		     << kv.first << "=" << kv.second;
	}
    }
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
o3d3xx::ImagerConfig::ExposureTime() const
{
  return this->exposure_time_;
}

void
o3d3xx::ImagerConfig::SetExposureTime(int usecs)
{
  this->exposure_time_ = usecs;
}

int
o3d3xx::ImagerConfig::ExposureTimeRatio() const
{
  return this->exposure_time_ratio_;
}

void
o3d3xx::ImagerConfig::SetExposureTimeRatio(int ratio)
{
  this->exposure_time_ratio_ = ratio;
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
o3d3xx::ImagerConfig::MinimumAmplitude() const noexcept
{
  return this->minimum_amplitude_;
}

void
o3d3xx::ImagerConfig::SetMinimumAmplitude(int min) noexcept
{
  this->minimum_amplitude_ = min;
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
o3d3xx::ImagerConfig::SetSpatialFilterType(int filt) noexcept
{
  this->spatial_filter_type_ = filt;
}

int
o3d3xx::ImagerConfig::SymmetryThreshold() const noexcept
{
  return this->symmetry_threshold_;
}

void
o3d3xx::ImagerConfig::SetSymmetryThreshold(int thresh) noexcept
{
  this->symmetry_threshold_ = thresh;
}

int
o3d3xx::ImagerConfig::TemporalFilterType() const noexcept
{
  return this->temporal_filter_type_;
}

void
o3d3xx::ImagerConfig::SetTemporalFilterType(int filt) noexcept
{
  this->temporal_filter_type_ = filt;
}

int
o3d3xx::ImagerConfig::ThreeFreqMax2FLineDistPercentage() const noexcept
{
  return this->three_freq_max_2f_line_dist_percentage_;
}

void
o3d3xx::ImagerConfig::SetThreeFreqMax2FLineDistPercentage(
  int percentage) noexcept
{
  this->three_freq_max_2f_line_dist_percentage_ = percentage;
}

int
o3d3xx::ImagerConfig::ThreeFreqMax3FLineDistPercentage() const noexcept
{
  return this->three_freq_max_3f_line_dist_percentage_;
}

void
o3d3xx::ImagerConfig::SetThreeFreqMax3FLineDistPercentage(
  int percentage) noexcept
{
  this->three_freq_max_3f_line_dist_percentage_ = percentage;
}

int
o3d3xx::ImagerConfig::TwoFreqMaxLineDistPercentage() const noexcept
{
  return this->two_freq_max_line_dist_percentage_;
}

void
o3d3xx::ImagerConfig::SetTwoFreqMaxLineDistPercentage(int percentage) noexcept
{
  this->two_freq_max_line_dist_percentage_ = percentage;
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

const std::unordered_map<std::string,
			 std::function<void(o3d3xx::ImagerConfig*,
					    const std::string&)> >
o3d3xx::ImagerConfig::mutator_map =
  {
    {"Channel",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetChannel(std::stoi(val)); } },

    {"ClippingBottom",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingBottom(std::stoi(val)); }},

    {"ClippingLeft",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingLeft(std::stoi(val)); }},

    {"ClippingRight",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingRight(std::stoi(val)); }},

    {"ClippingTop",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingTop(std::stoi(val)); }},

    {"ExposureTime",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetExposureTime(std::stoi(val)); } },

    {"ExposureTimeRatio",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetExposureTimeRatio(std::stoi(val)); } },

    {"FrameRate",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetFrameRate(std::stod(val)); } },

    {"MinimumAmplitude",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetMinimumAmplitude(std::stoi(val)); } },

    {"ReduceMotionArtifacts",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetReduceMotionArtifacts(o3d3xx::stob(val)); }},

    {"SpatialFilterType",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetSpatialFilterType(std::stoi(val)); } },

    {"SymmetryThreshold",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetSymmetryThreshold(std::stoi(val)); } },

    {"TemporalFilterType",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetTemporalFilterType(std::stoi(val)); } },

    {"ThreeFreqMax2FLineDistPercentage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetThreeFreqMax2FLineDistPercentage(std::stoi(val)); } },

    {"ThreeFreqMax3FLineDistPercentage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetThreeFreqMax3FLineDistPercentage(std::stoi(val)); } },

    {"TwoFreqMaxLineDistPercentage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetTwoFreqMaxLineDistPercentage(std::stoi(val)); } },

    {"Type",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetType(val); } },

    {"TypeHash",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetTypeHash(val); } },
  };

std::string
o3d3xx::ImagerConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("Channel", this->Channel());
  pt.put("ClippingBottom", this->ClippingBottom());
  pt.put("ClippingLeft", this->ClippingLeft());
  pt.put("ClippingRight", this->ClippingRight());
  pt.put("ClippingTop", this->ClippingTop());

  if (boost::algorithm::ends_with(this->Type(), "high"))
    {

    }
  else if (boost::algorithm::ends_with(this->Type(), "low"))
    {
      pt.put("ExposureTime", this->ExposureTime());
    }
  else
    {
      pt.put("ExposureTime", this->ExposureTime());
      pt.put("ExposureTimeRatio", this->ExposureTimeRatio());
    }

  pt.put("FrameRate", this->FrameRate());
  pt.put("MinimumAmplitude", this->MinimumAmplitude());
  pt.put("ReduceMotionArtifacts", this->ReduceMotionArtifacts());
  pt.put("SpatialFilterType", this->SpatialFilterType());
  pt.put("SymmetryThreshold", this->SymmetryThreshold());
  pt.put("TemporalFilterType", this->TemporalFilterType());
  pt.put("ThreeFreqMax2FLineDistPercentage",
	 this->ThreeFreqMax2FLineDistPercentage());
  pt.put("ThreeFreqMax3FLineDistPercentage",
	 this->ThreeFreqMax3FLineDistPercentage());
  pt.put("TwoFreqMaxLineDistPercentage",
	 this->TwoFreqMaxLineDistPercentage());
  pt.put("Type", this->Type());
  pt.put("TypeHash", this->TypeHash());

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
