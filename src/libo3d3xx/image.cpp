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

#include "o3d3xx/image.h"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "o3d3xx/util.hpp"
#include "o3d3xx/err.h"

const std::size_t o3d3xx::IMG_TICKET_SZ = 16; // bytes

bool
o3d3xx::verify_ticket_buffer(const std::vector<std::uint8_t>& buff)
{
  return buff.size() == o3d3xx::IMG_TICKET_SZ &&
         buff.at(4) == 'L' &&
         buff.at(14) == '\r' &&
         buff.at(15) == '\n';
}

bool
o3d3xx::verify_image_buffer(const std::vector<std::uint8_t>& buff)
{
  std::size_t buff_sz = buff.size();

  return buff_sz > 8 &&
         std::string(buff.begin()+4, buff.begin()+8) == "star" &&
         std::string(buff.end()-6, buff.end()-2) == "stop" &&
         buff.at(buff_sz - 2) == '\r' &&
         buff.at(buff_sz - 1) == '\n';
}

std::size_t
o3d3xx::get_image_buffer_size(const std::vector<std::uint8_t>& buff)
{
  return std::stoi(std::string(buff.begin()+5, buff.end()));
}

void
o3d3xx::image_buff_to_point_cloud(const std::vector<std::uint8_t>& buff,
				  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  // Get indices to the start of each chunk of interest in the image buffer
  std::size_t xidx =
    o3d3xx::get_chunk_index(buff, o3d3xx::image_chunk::CARTESIAN_X);
  std::size_t yidx =
    o3d3xx::get_chunk_index(buff, o3d3xx::image_chunk::CARTESIAN_Y);
  std::size_t zidx =
    o3d3xx::get_chunk_index(buff, o3d3xx::image_chunk::CARTESIAN_Z);
  std::size_t cidx =
    o3d3xx::get_chunk_index(buff, o3d3xx::image_chunk::CONFIDENCE);
  std::size_t aidx =
    o3d3xx::get_chunk_index(buff, o3d3xx::image_chunk::AMPLITUDE);

  DLOG(INFO) << "x_idx=" << xidx
	     << ", y_idx=" << yidx
	     << ", z_idx=" << zidx
	     << ", a_idx=" << aidx
	     << ", c_idx=" << cidx;

  // Get how many bytes to increment in the buffer for each pixel
  // NOTE: These can be discovered dynamically, however, for now
  // we use our apriori info of the pixel data types
  std::size_t xincr = 2; // uint16_t
    //    o3d3xx::get_num_bytes_in_pixel_format(
    //      o3d3xx::mkval<o3d3xx::pixel_format>(buff.data()+xidx+24));

  std::size_t yincr = 2; // uint16_t
    //    o3d3xx::get_num_bytes_in_pixel_format(
    //      o3d3xx::mkval<o3d3xx::pixel_format>(buff.data()+yidx+24));

  std::size_t zincr = 2; // uint16_t
    //    o3d3xx::get_num_bytes_in_pixel_format(
    //      o3d3xx::mkval<o3d3xx::pixel_format>(buff.data()+zidx+24));

  std::size_t aincr = 2; // uint16_t
    //    o3d3xx::get_num_bytes_in_pixel_format(
    //      o3d3xx::mkval<o3d3xx::pixel_format>(buff.data()+aidx+24));

  std::size_t cincr = 1; // uint8_t
    //    o3d3xx::get_num_bytes_in_pixel_format(
    //      o3d3xx::mkval<o3d3xx::pixel_format>(buff.data()+cidx+24));

  cloud->header.frame_id = "/o3d3xx";
  cloud->width = o3d3xx::mkval<std::uint32_t>(buff.data()+xidx+16);
  cloud->height = o3d3xx::mkval<std::uint32_t>(buff.data()+xidx+20);
  cloud->is_dense = true;

  std::uint32_t num_points = cloud->height * cloud->width;
  cloud->points.resize(num_points);

  DLOG(INFO) << "width=" << cloud->width
	     << ", height=" << cloud->height
	     << ", num_points=" << num_points;

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  // move all index pointers to where the pixel data starts
  xidx += 36; yidx += 36; zidx += 36; aidx += 36; cidx += 36;

  for (std::size_t i = 0; i < num_points;
       ++i, xidx += xincr, yidx += yincr, zidx += zincr,
	 cidx += cincr, aidx += aincr)
    {
      pcl::PointXYZI& pt = cloud->points[i];
      if (buff.at(cidx) & 0x1 == 1)
	{
	  pt.x = pt.y = pt.z = bad_point;
	  cloud->is_dense = false;
	}
      else
	{
	  // convert the units to meters and the coord frame
	  // to a right-handed frame.
	  pt.x = o3d3xx::mkval<std::uint16_t>(buff.data()+zidx) / 1000.0f;
	  pt.y = -o3d3xx::mkval<std::uint16_t>(buff.data()+xidx) / 1000.0f;
	  pt.z = -o3d3xx::mkval<std::uint16_t>(buff.data()+yidx) / 1000.0f;
	}

      pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
      pt.intensity = o3d3xx::mkval<std::uint16_t>(buff.data()+aidx);
    }

  cloud->sensor_origin_.setZero();
  cloud->sensor_orientation_.w() = 1.0f;
  cloud->sensor_orientation_.x() = 0.0f;
  cloud->sensor_orientation_.y() = 0.0f;
  cloud->sensor_orientation_.z() = 0.0f;
}

std::size_t
o3d3xx::get_chunk_index(const std::vector<std::uint8_t>& buff,
			o3d3xx::image_chunk chunk_type)
{
  std::size_t idx = 8; // start of first chunk

  while (buff.begin()+idx < buff.end())
    {
      if (static_cast<std::uint32_t>(chunk_type) ==
	  o3d3xx::mkval<std::uint32_t>(buff.data()+idx))
	{
	  return idx;
	}

      // move to the beginning of the next chunk
      idx += o3d3xx::mkval<std::uint32_t>(buff.data()+idx+4);
    }

  throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
}

std::size_t
o3d3xx::get_num_bytes_in_pixel_format(o3d3xx::pixel_format f)
{
  switch (f)
    {
    case o3d3xx::pixel_format::FORMAT_8U:
      return 1;

    case o3d3xx::pixel_format::FORMAT_8S:
      return 1;

    case o3d3xx::pixel_format::FORMAT_16U:
      return 2;

    case o3d3xx::pixel_format::FORMAT_16S:
      return 2;

    case o3d3xx::pixel_format::FORMAT_32U:
      return 4;

    case o3d3xx::pixel_format::FORMAT_32S:
      return 4;

    case o3d3xx::pixel_format::FORMAT_32F:
      return 4;

    case o3d3xx::pixel_format::FORMAT_64U:
      return 8;

    case o3d3xx::pixel_format::FORMAT_64F:
      return 8;

    default:
      return 0;
    }
}
