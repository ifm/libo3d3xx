/*
 * Copyright (C) 2016 Love Park Robotics, LLC
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

#include "o3d3xx_framegrabber/pcic_schema.h"
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

const std::uint16_t o3d3xx::IMG_RDIS = 1;  // 2^0
const std::uint16_t o3d3xx::IMG_AMP  = 2;  // 2^1
const std::uint16_t o3d3xx::IMG_RAMP = 4;  // 2^2
const std::uint16_t o3d3xx::IMG_CART = 8;  // 2^3
const std::uint16_t o3d3xx::IMG_UVEC = 16; // 2^4
const std::uint16_t o3d3xx::EXP_TIME = 32; // 2^5

auto __o3d3xx_schema_mask__ = []()->std::uint16_t
  {
    try
      {
        return std::getenv("O3D3XX_MASK") == nullptr ?
            o3d3xx::IMG_RDIS|o3d3xx::IMG_AMP|o3d3xx::IMG_RAMP|o3d3xx::IMG_CART :
            std::stoul(std::string(std::getenv("O3D3XX_MASK"))) & 0xFFFF;
      }
    catch (const std::exception& ex)
      {
        return o3d3xx::IMG_RDIS |
               o3d3xx::IMG_AMP  |
               o3d3xx::IMG_RAMP |
               o3d3xx::IMG_CART;
      }
  };

const std::uint16_t o3d3xx::DEFAULT_SCHEMA_MASK = __o3d3xx_schema_mask__();

std::string
o3d3xx::make_pcic_schema(std::uint16_t mask)
{
  std::string schema =
  R"(
      {
        "layouter": "flexible",
        "format"  : {"dataencoding":"ascii"},
        "elements":
         [
           {"type":"string", "value":"star", "id":"start_string"})";

  if((mask & o3d3xx::IMG_RDIS) == o3d3xx::IMG_RDIS)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"distance_image"})";
    }

  if((mask & o3d3xx::IMG_AMP) == o3d3xx::IMG_AMP)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"normalized_amplitude_image"})";
    }

  if((mask & o3d3xx::IMG_RAMP) == o3d3xx::IMG_RAMP)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"amplitude_image"})";
    }

  if((mask & o3d3xx::IMG_CART) == o3d3xx::IMG_CART)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"x_image"},
           {"type":"blob", "id":"y_image"},
           {"type":"blob", "id":"z_image"})";
    }

  if((mask & o3d3xx::IMG_UVEC) == o3d3xx::IMG_UVEC)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"all_unit_vector_matrices"})";
    }

  // confidence_image and extrinsics are invariant
  schema +=
    R"(,
           {"type":"blob", "id":"confidence_image"},
           {"type":"blob", "id":"extrinsic_calibration"})";

  if((mask & o3d3xx::EXP_TIME) == o3d3xx::EXP_TIME)
    {
      schema +=
      R"(,
           {"type":"string", "id":"exposure_times", "value":"extime"},
           {
            "type":"uint32", "id":"exposure_time_1",
            "format":{"dataencoding":"binary", "order":"little"}
           },
           {
             "type":"uint32", "id":"exposure_time_2",
             "format":{"dataencoding":"binary", "order":"little"}
           },
           {
             "type":"uint32", "id":"exposure_time_3",
             "format":{"dataencoding":"binary", "order":"little"}
           })";
    }

  // other invariants
  schema +=
  R"(,
           {"type":"string", "value":"stop", "id":"end_string"}
         ]
      }
   )";

  return schema;
}

std::uint16_t
o3d3xx::schema_mask_from_string(const std::string& in)
{
  std::uint16_t mask = 0;
  std::vector<std::string> mask_parts;
  boost::split(mask_parts, in, boost::is_any_of("|"));
  for (auto part : mask_parts)
    {
      boost::algorithm::trim(part);
      if (part == "IMG_RDIS")
        {
          mask |= o3d3xx::IMG_RDIS;
        }
      else if (part == "IMG_AMP")
        {
          mask |= o3d3xx::IMG_AMP;
        }
      else if (part == "IMG_RAMP")
        {
          mask |= o3d3xx::IMG_RAMP;
        }
      else if (part == "IMG_CART")
        {
          mask |= o3d3xx::IMG_CART;
        }
      else if (part == "IMG_UVEC")
        {
          mask |= o3d3xx::IMG_UVEC;
        }
      else if (part == "EXP_TIME")
        {
          mask |= o3d3xx::EXP_TIME;
        }
    }

  return mask;
}
