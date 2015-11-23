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

#include "o3d3xx/util.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <xmlrpc-c/base.hpp>
#include "o3d3xx/image.h"
#include "o3d3xx/frame_grabber.h"
#include "o3d3xx/version.h"

//--------------------------------------------------
// Logging-related
//--------------------------------------------------

std::once_flag o3d3xx::Logging::init_;

void
o3d3xx::Logging::Init()
{
  std::call_once(o3d3xx::Logging::init_, o3d3xx::Logging::_Init);
}

void
o3d3xx::Logging::_Init()
{
  FLAGS_logbuflevel = -1;
  google::InitGoogleLogging(O3D3XX_LIBRARY_NAME);
  google::SetStderrLogging(google::FATAL);
}

//--------------------------------------------------
// XMLRPC utilities
//--------------------------------------------------

std::unordered_map<std::string, std::string> const
o3d3xx::value_struct_to_map(const xmlrpc_c::value_struct& vs)
{
  std::map<std::string, xmlrpc_c::value> const
    resmap(static_cast<std::map<std::string, xmlrpc_c::value> >(vs));

  std::unordered_map<std::string, std::string> retval;
  for (auto& kv : resmap)
    {
      retval[kv.first] = std::string(xmlrpc_c::value_string(kv.second));
    }

  return retval;
}

std::unordered_map<std::string,
                   std::unordered_map<std::string, std::string> > const
o3d3xx::value_struct_to_map_of_maps(const xmlrpc_c::value_struct& vs)
{
  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::string> >
    retval;


  std::map<std::string, xmlrpc_c::value> const
    outter_map(static_cast<std::map<std::string, xmlrpc_c::value> >
               (vs));

  for (auto& kv : outter_map)
    {
      xmlrpc_c::value_struct _vs(kv.second);

      std::map<std::string, xmlrpc_c::value> const
        inner_map(static_cast<std::map<std::string, xmlrpc_c::value> >
                  (_vs));

      std::unordered_map<std::string, std::string> inner_retval;

      for (auto& inner_kv : inner_map)
        {
          inner_retval[inner_kv.first] =
            std::string(xmlrpc_c::value_string(inner_kv.second));
        }

      retval[kv.first] = inner_retval;
    }

  return retval;
}

//--------------------------------------------------
// Image processing utilities
//--------------------------------------------------

cv::Mat
o3d3xx::hist1(const cv::Mat& img, int histsize)
{
  assert(img.channels() == 1);

  float range[] = {0, (float) histsize};
  const float* histrange = {range};

  cv::Mat hist;
  cv::calcHist(&img,        // source image
               1,           // number of source images
               0,           // channels used to compute hist
               cv::Mat(),   // mask
               hist,        // output histogram
               1,           // dimensionality of histogram
               &histsize,   // histogram size in each dimension
               &histrange,  // bin boundaries in each dimension
               true,        // uniform flag
               true);       // accumulate flag

  int hist_width = img.cols;
  int hist_height = img.rows;
  int bin_width = static_cast<int>(std::round((double) hist_width / histsize));

  cv::Mat histimg = cv::Mat::zeros(hist_height, hist_width, CV_16UC3);
  cv::normalize(hist, hist, 0, histimg.rows, cv::NORM_MINMAX, -1, cv::Mat());

  for (int i = 1; i < histsize; i++)
    {
      cv::line(histimg,
               cv::Point(bin_width*(i-1),
                         hist_height -
                           static_cast<int>(std::round(hist.at<float>(i-1)))),
               cv::Point(bin_width*(i),
                         hist_height -
                           static_cast<int>(std::round(hist.at<float>(i)))),
               CV_RGB(65535, 65535, 65535),  // line color
               1,                   // line thickness
               8,                   // line type (8 = connected)
               0);                  // fractional bits in the point coords
    }

  return histimg;
}

//--------------------------------------------------
// Framegrabber helpers
//--------------------------------------------------

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

  schema +=
  R"(,
           {"type":"blob", "id":"confidence_image"},
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
    }

  return mask;
}

//--------------------------------------------------
// Misc
//--------------------------------------------------

bool
o3d3xx::stob(const std::string& s)
{
  if ((s == "1") ||
      (boost::iequals(s, "true")) ||
      (boost::iequals(s, "yes")))
    {
      return true;
    }

  return false;
}
