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

#include "o3d3xx_camera/imager_config.h"
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
#include "o3d3xx_camera/camera.hpp"
#include "o3d3xx_camera/util.h"

o3d3xx::ImagerConfig::ImagerConfig()
  : channel_(0),
    clipping_bottom_(131),
    clipping_left_(0),
    clipping_right_(175),
    clipping_top_(0),
    continuous_auto_exposure_(false),
    enable_amplitude_correction_(true),
    enable_fast_frequency_(false),
    enable_filter_amplitude_image_(true),
    enable_filter_distance_image_(true),
    enable_rectification_amplitude_image_(true),
    enable_rectification_distance_image_(true),
    exposure_time_(1000),
    exposure_time_list_(""),
    exposure_time_ratio_(40),
    frame_rate_(5.0),
    minimum_amplitude_(0),
    resolution_(o3d3xx::RES_23K),
    clipping_cuboid_(""),
    spatial_filter_type_(
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF)),
    symmetry_threshold_(0),
    temporal_filter_type_(
      static_cast<int>(o3d3xx::Camera::temporal_filter::OFF)),
    three_freq_max_3f_line_dist_percentage_(0),
    three_freq_max_2f_line_dist_percentage_(0),
    two_freq_max_line_dist_percentage_(0),
    type_("under5m_low"),
    max_allowed_led_frame_rate_(0.0)
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

bool
o3d3xx::ImagerConfig::ContinuousAutoExposure() const noexcept
{
  return this->continuous_auto_exposure_;
}

void
o3d3xx::ImagerConfig::SetContinuousAutoExposure(bool on) noexcept
{
  this->continuous_auto_exposure_ = on;
}

bool
o3d3xx::ImagerConfig::EnableAmplitudeCorrection() const noexcept
{
  return this->enable_amplitude_correction_;
}

void
o3d3xx::ImagerConfig::SetEnableAmplitudeCorrection(bool enable) noexcept
{
  this->enable_amplitude_correction_ = enable;
}

bool
o3d3xx::ImagerConfig::EnableFastFrequency() const noexcept
{
  return this->enable_fast_frequency_;
}

void
o3d3xx::ImagerConfig::SetEnableFastFrequency(bool enable) noexcept
{
  this->enable_fast_frequency_ = enable;
}

bool
o3d3xx::ImagerConfig::EnableFilterAmplitudeImage() const noexcept
{
  return this->enable_filter_amplitude_image_;
}

void
o3d3xx::ImagerConfig::SetEnableFilterAmplitudeImage(bool enable) noexcept
{
  this->enable_filter_amplitude_image_ = enable;
}

bool
o3d3xx::ImagerConfig::EnableFilterDistanceImage() const noexcept
{
  return this->enable_filter_distance_image_;
}

void
o3d3xx::ImagerConfig::SetEnableFilterDistanceImage(bool enable) noexcept
{
  this->enable_filter_distance_image_ = enable;
}

bool
o3d3xx::ImagerConfig::EnableRectificationAmplitudeImage() const noexcept
{
  return this->enable_rectification_amplitude_image_;
}

void
o3d3xx::ImagerConfig::SetEnableRectificationAmplitudeImage(bool enable)
noexcept
{
  this->enable_rectification_amplitude_image_ = enable;
}

bool
o3d3xx::ImagerConfig::EnableRectificationDistanceImage() const noexcept
{
  return this->enable_rectification_distance_image_;
}

void
o3d3xx::ImagerConfig::SetEnableRectificationDistanceImage(bool enable) noexcept
{
  this->enable_rectification_distance_image_ = enable;
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

std::string
o3d3xx::ImagerConfig::ExposureTimeList() const noexcept
{
  return this->exposure_time_list_;
}

void
o3d3xx::ImagerConfig::SetExposureTimeList(const std::string& s) noexcept
{
  this->exposure_time_list_ = s;
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

int
o3d3xx::ImagerConfig::Resolution() const noexcept
{
  return this->resolution_;
}

void
o3d3xx::ImagerConfig::SetResolution(int res) noexcept
{
  this->resolution_ = res;
}

std::string
o3d3xx::ImagerConfig::ClippingCuboid() const noexcept
{
  return this->clipping_cuboid_;
}

void
o3d3xx::ImagerConfig::SetClippingCuboid(const std::string& s) noexcept
{
  this->clipping_cuboid_ = s;
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

double
o3d3xx::ImagerConfig::MaxAllowedLEDFrameRate() const noexcept
{
  return this->max_allowed_led_frame_rate_;
}

void
o3d3xx::ImagerConfig::SetMaxAllowedLEDFrameRate(double rate) noexcept
{
  this->max_allowed_led_frame_rate_ = rate;
}

const std::unordered_map<std::string,
                         std::function<void(o3d3xx::ImagerConfig*,
                                            const std::string&)> >
o3d3xx::ImagerConfig::mutator_map =
  {
    {"Channel",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetChannel(std::stoi(val)); }},

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

    {"ContinuousAutoExposure",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetContinuousAutoExposure(o3d3xx::stob(val)); }},

    {"EnableAmplitudeCorrection",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetEnableAmplitudeCorrection(o3d3xx::stob(val)); }},

    {"EnableFastFrequency",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetEnableFastFrequency(o3d3xx::stob(val)); }},

    {"EnableFilterAmplitudeImage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetEnableFilterAmplitudeImage(o3d3xx::stob(val)); }},

    {"EnableFilterDistanceImage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetEnableFilterDistanceImage(o3d3xx::stob(val)); }},

    {"EnableRectificationAmplitudeImage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetEnableRectificationAmplitudeImage(o3d3xx::stob(val)); }},

    {"EnableRectificationDistanceImage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetEnableRectificationDistanceImage(o3d3xx::stob(val)); }},

    {"ExposureTime",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetExposureTime(std::stoi(val)); }},

    {"ExposureTimeList",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetExposureTimeList(val); }},

    {"ExposureTimeRatio",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetExposureTimeRatio(std::stoi(val)); }},

    {"FrameRate",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetFrameRate(std::stod(val)); }},

    {"MinimumAmplitude",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetMinimumAmplitude(std::stoi(val)); }},

    {"Resolution",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetResolution(std::stoi(val)); }},

    {"ClippingCuboid",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetClippingCuboid(val); }},

    {"SpatialFilterType",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetSpatialFilterType(std::stoi(val)); }},

    {"SymmetryThreshold",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetSymmetryThreshold(std::stoi(val)); }},

    {"TemporalFilterType",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetTemporalFilterType(std::stoi(val)); }},

    {"ThreeFreqMax2FLineDistPercentage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetThreeFreqMax2FLineDistPercentage(std::stoi(val)); }},

    {"ThreeFreqMax3FLineDistPercentage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetThreeFreqMax3FLineDistPercentage(std::stoi(val)); }},

    {"TwoFreqMaxLineDistPercentage",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetTwoFreqMaxLineDistPercentage(std::stoi(val)); }},

    {"Type",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetType(val); }},

    {"MaxAllowedLEDFrameRate",
     [](o3d3xx::ImagerConfig* im, const std::string& val)
     { im->SetMaxAllowedLEDFrameRate(std::stod(val)); }},
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
  pt.put("ContinuousAutoExposure", this->ContinuousAutoExposure());
  pt.put("EnableAmplitudeCorrection", this->EnableAmplitudeCorrection());
  pt.put("EnableFastFrequency", this->EnableFastFrequency());
  pt.put("EnableFilterAmplitudeImage", this->EnableFilterAmplitudeImage());
  pt.put("EnableFilterDistanceImage", this->EnableFilterDistanceImage());
  pt.put("EnableRectificationAmplitudeImage",
         this->EnableRectificationAmplitudeImage());
  pt.put("EnableRectificationDistanceImage",
         this->EnableRectificationDistanceImage());

  if (boost::algorithm::ends_with(this->Type(), "high"))
    {
      pt.put("ExposureTimeList", this->ExposureTimeList());
    }
  else if (boost::algorithm::ends_with(this->Type(), "low"))
    {
      pt.put("ExposureTime", this->ExposureTime());
      pt.put("ExposureTimeList", this->ExposureTimeList());
    }
  else
    {
      pt.put("ExposureTime", this->ExposureTime());
      pt.put("ExposureTimeList", this->ExposureTimeList());
      pt.put("ExposureTimeRatio", this->ExposureTimeRatio());
    }

  pt.put("FrameRate", this->FrameRate());
  pt.put("MinimumAmplitude", this->MinimumAmplitude());
  pt.put("Resolution", this->Resolution());
  pt.put("ClippingCuboid", this->ClippingCuboid());
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
  pt.put("MaxAllowedLEDFrameRate", this->MaxAllowedLEDFrameRate());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::ImagerConfig::Ptr
o3d3xx::ImagerConfig::FromJSON(const std::string& json,
                               o3d3xx::ImagerConfig::Ptr im_ptr)
{
  o3d3xx::ImagerConfig::Ptr im =
    im_ptr ? im_ptr : std::make_shared<o3d3xx::ImagerConfig>();

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
