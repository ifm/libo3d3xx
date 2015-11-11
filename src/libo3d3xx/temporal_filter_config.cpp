/*
 * Copyright (C) 2015 Love Park Robotics, LLC
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

#include "o3d3xx/temporal_filter_config.h"
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx/camera.hpp"
#include "o3d3xx/err.h"

/////////////////////////////////
//
// TemporalFilterConfig class
//
/////////////////////////////////

o3d3xx::TemporalFilterConfig::TemporalFilterConfig(
  int type, const std::string& type_str)
  : type_(type),
    type_str_(type_str),
    number_of_images_(-1)
{ }

int
o3d3xx::TemporalFilterConfig::Type() const noexcept
{
  return this->type_;
}

std::string
o3d3xx::TemporalFilterConfig::TypeStr() const noexcept
{
  return this->type_str_;
}

int
o3d3xx::TemporalFilterConfig::NumberOfImages() const
{
  if (this->Type() !=
      static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER))
    {
      LOG(ERROR) << "Filter of type=" << this->Type()
                 << " does not support the 'NumberOfImages' parameter";
      throw o3d3xx::error_t(O3D3XX_VALUE_OUT_OF_RANGE);
    }

  return this->number_of_images_;
}

void
o3d3xx::TemporalFilterConfig::SetNumberOfImages(int n_imgs)
{
  if (this->Type() !=
      static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER))
    {
      LOG(ERROR) << "Filter of type=" << this->Type()
                 << " does not support the 'NumberOfImages' parameter";
      throw o3d3xx::error_t(O3D3XX_VALUE_OUT_OF_RANGE);
    }

  this->number_of_images_ = n_imgs;
}

o3d3xx::TemporalFilterConfig::Ptr
o3d3xx::TemporalFilterConfig::FromJSON(
  const std::string& json, o3d3xx::TemporalFilterConfig::Ptr filt_ptr)
{
  // NOTE: `filt_ptr' may be `nullptr'
  o3d3xx::TemporalFilterConfig::Ptr filt = filt_ptr;

  boost::property_tree::ptree pt;
  std::istringstream is(json);
  boost::property_tree::read_json(is, pt);

  int type;
  try
    {
      type = pt.get<int>("Type");
      if (! (filt && (type == filt->Type())))
        {
          switch (type)
            {
            case static_cast<int>(
             o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER):
              filt = std::make_shared<o3d3xx::TemporalMeanFilterConfig>();
              break;

            case static_cast<int>(
             o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER):
              filt =
                std::make_shared<
                  o3d3xx::TemporalAdaptiveExponentialFilterConfig>();
              break;

            default:
              filt = std::make_shared<o3d3xx::TemporalFilterConfig>();
              break;
            }
        }
    }
  catch (const boost::property_tree::ptree_bad_path& path_ex)
    {
      if (filt)
        {
          type = filt->Type();
        }
      else
        {
          throw;
        }
    }

  if (type ==
      static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER))
    {
      try
        {
          filt->SetNumberOfImages(pt.get<int>("NumberOfImages"));
        }
      catch (const boost::property_tree::ptree_bad_path& ex)
        {
          LOG(WARNING) << "Failed to extract 'NumberOfImages': "
                       << ex.what();
        }
    }

  return filt;
}

std::string
o3d3xx::TemporalFilterConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("Type", this->Type());
  pt.put("TypeStr_", this->TypeStr());

  if (this->Type() ==
      static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER))
    {
      pt.put("NumberOfImages", this->NumberOfImages());
    }

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

////////////////////////////////////
//
// TemporalMeanFilterConfig class
//
////////////////////////////////////

o3d3xx::TemporalMeanFilterConfig::TemporalMeanFilterConfig()
  : o3d3xx::TemporalFilterConfig(
      static_cast<int>(o3d3xx::Camera::temporal_filter::TEMPORAL_MEAN_FILTER),
      "Temporal Mean Filter")
{
  this->SetNumberOfImages(2);
}

/////////////////////////////////////////////////
//
// TemporalExponentialAdaptiveFilterConfig class
//
/////////////////////////////////////////////////

o3d3xx::TemporalAdaptiveExponentialFilterConfig::TemporalAdaptiveExponentialFilterConfig()
  : o3d3xx::TemporalFilterConfig(
      static_cast<int>(o3d3xx::Camera::temporal_filter::ADAPTIVE_EXPONENTIAL_FILTER),
      "Adaptive Exponential Filter")
{ }
