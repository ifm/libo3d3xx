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

#include "o3d3xx/spatial_filter_config.h"
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
// SpatialFilterConfig class
//
/////////////////////////////////

o3d3xx::SpatialFilterConfig::SpatialFilterConfig(
  int type, const std::string& type_str)
  : type_(type),
    type_str_(type_str),
    mask_size_(-1)
{ }

int
o3d3xx::SpatialFilterConfig::Type() const noexcept
{
  return this->type_;
}

std::string
o3d3xx::SpatialFilterConfig::TypeStr() const noexcept
{
  return this->type_str_;
}

int
o3d3xx::SpatialFilterConfig::MaskSize() const
{
  if (this->Type() ==
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF))
    {
      LOG(ERROR) << "No mask size for 'OFF' spatial filter";
      throw o3d3xx::error_t(O3D3XX_VALUE_OUT_OF_RANGE);
    }

  return this->mask_size_;
}

void
o3d3xx::SpatialFilterConfig::SetMaskSize(int size)
{
  if (this->Type() ==
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF))
    {
      LOG(ERROR) << "No mask size for 'OFF' spatial filter";
      throw o3d3xx::error_t(O3D3XX_VALUE_OUT_OF_RANGE);
    }

  switch (size)
    {
    case static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_3x3):
      this->mask_size_ = size;
      break;

    case static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_5x5):
      this->mask_size_ = size;
      break;

    default:
      LOG(ERROR) << "Illegal mask size: " << size;
      throw o3d3xx::error_t(O3D3XX_VALUE_OUT_OF_RANGE);
    }
}

std::string
o3d3xx::SpatialFilterConfig::MaskSizeStr() const
{
  switch (this->mask_size_)
    {
    case static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_3x3):
      return "3x3";

    case static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_5x5):
      return "5x5";

    default:
      return "MaskSize not supported for filter type";
    }
}

o3d3xx::SpatialFilterConfig::Ptr
o3d3xx::SpatialFilterConfig::FromJSON(
  const std::string& json, o3d3xx::SpatialFilterConfig::Ptr filt_ptr)
{
  // NOTE: `filt_ptr' may be `nullptr'
  o3d3xx::SpatialFilterConfig::Ptr filt = filt_ptr;

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
              o3d3xx::Camera::spatial_filter::MEDIAN_FILTER):
                filt = std::make_shared<o3d3xx::SpatialMedianFilterConfig>();
                break;

            case static_cast<int>(
              o3d3xx::Camera::spatial_filter::MEAN_FILTER):
                filt = std::make_shared<o3d3xx::SpatialMeanFilterConfig>();
                break;

            case static_cast<int>(
              o3d3xx::Camera::spatial_filter::BILATERAL_FILTER):
                filt = std::make_shared<o3d3xx::SpatialBilateralFilterConfig>();
                break;

            default:
              filt = std::make_shared<o3d3xx::SpatialFilterConfig>();
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

  //! @todo May want to use the more dynamic "mutator_map" approach
  //! used in the other config objects.
  if (type !=
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF))
    {
      try
        {
          filt->SetMaskSize(pt.get<int>("MaskSize"));
        }
      catch (const boost::property_tree::ptree_bad_path& ex)
        {
          LOG(WARNING) << "Failed to extract 'MaskSize': " << ex.what();
        }
    }

  return filt;
}

std::string
o3d3xx::SpatialFilterConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("Type", this->Type());
  pt.put("TypeStr_", this->TypeStr());

  if (this->Type() !=
      static_cast<int>(o3d3xx::Camera::spatial_filter::OFF))
    {
      pt.put("MaskSize", this->MaskSize());
      pt.put("MaskSizeStr_", this->MaskSizeStr());
    }

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}


////////////////////////////////////
//
// SpatialMeanFilterConfig class
//
////////////////////////////////////

o3d3xx::SpatialMeanFilterConfig::SpatialMeanFilterConfig()
  : o3d3xx::SpatialFilterConfig(
      static_cast<int>(o3d3xx::Camera::spatial_filter::MEAN_FILTER),
      "Mean Filter")
{
  this->SetMaskSize(
    static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_3x3));
}

/////////////////////////////////////
//
// SpatialMedianFilterConfig class
//
/////////////////////////////////////

o3d3xx::SpatialMedianFilterConfig::SpatialMedianFilterConfig()
  : o3d3xx::SpatialFilterConfig(
      static_cast<int>(o3d3xx::Camera::spatial_filter::MEDIAN_FILTER),
      "Median Filter")
{
  this->SetMaskSize(
    static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_3x3));
}

/////////////////////////////////////
//
// SpatialBilateralFilterConfig class
//
/////////////////////////////////////

o3d3xx::SpatialBilateralFilterConfig::SpatialBilateralFilterConfig()
  : o3d3xx::SpatialFilterConfig(
      static_cast<int>(o3d3xx::Camera::spatial_filter::BILATERAL_FILTER),
      "Bilateral Filter")
{
  this->SetMaskSize(
    static_cast<int>(o3d3xx::SpatialFilterConfig::mask_size::_3x3));
}
