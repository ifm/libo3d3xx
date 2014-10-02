// -*- c++ -*-
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
#ifndef __O3D3XX_IMAGE_H__
#define __O3D3XX_IMAGE_H__

#include <cstddef>
#include <cstdint>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace o3d3xx
{
  extern const std::size_t IMG_TICKET_SZ; // bytes

  enum class pixel_format : std::uint8_t
  {
    FORMAT_8U = 0,
    FORMAT_8S = 1,
    FORMAT_16U = 2,
    FORMAT_16S = 3,
    FORMAT_32U = 4,
    FORMAT_32S = 5,
    FORMAT_32F = 6,
    FORMAT_64U = 7,
    FORMAT_64F = 8
  };

  enum class image_chunk : std::uint32_t
  {
    USERDATA = 0,
    RADIAL_DISTANCE = 100,
    AMPLITUDE = 101,
    CARTESIAN_X = 200,
    CARTESIAN_Y = 201,
    CARTESIAN_Z = 202,
    UNIT_VECTOR_X = 220,
    UNIT_VECTOR_Y = 221,
    UNIT_VECTOR_Z = 222,
    CONFIDENCE = 300,
    DIAGNOSTIC_DATA = 302,
    MIRA_RAW_DATA = 303
  };

  /**
   * Verifies that the passed in buffer contains a valid "ticket" from the
   * sensor.
   *
   * @return true if the ticket buffer is valid
   */
  bool verify_ticket_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Verifies that the passed in image buffer is valid.
   *
   * @return true if the image buffer is valid
   */
  bool verify_image_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Extracts the image buffer size from a ticket buffer received from the
   * sensor.
   *
   * NOTE: The size of the passed in buffer is not checked here. It is assumed
   * that this buffer has already passed the `verify_ticket_buffer' check. An
   * exception may be thrown if an inappropriately sized buffer is passed in to
   * this function.
   *
   * @return The expected size of the image buffer.
   */
  std::size_t get_image_buffer_size(const std::vector<std::uint8_t>& buff);

  /**
   * Converts an image buffer from the sensor into a PCL point cloud. The
   * unit of measure of the points will be in meters and oriented in a
   * right-handed coordinate frame.
   *
   * NOTE: For now, the intensity channel of the cloud will contain data from
   * the amplitude image.
   *
   * @param[in] buff The image buffer to convert
   * @param[out] cloud The point cloud to fill with point data
   */
  void image_buff_to_point_cloud(const std::vector<std::uint8_t>& buff,
				 pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  /**
   * Finds the index into the image buffer of where the chunk of `chunk_type'
   * begins.
   *
   * @param[in] buff The image buffer to search
   * @param[in] chunk_type The type of chunk to look for
   *
   * @return The index into the buffer of where the chunk begins.
   * @throw o3d3xx::error_t If the chunk is not found
   */
  std::size_t get_chunk_index(const std::vector<std::uint8_t>& buff,
			      o3d3xx::image_chunk chunk_type);

  /**
   * Returns the number of bytes contained in the passed in pixel format.
   */
  std::size_t get_num_bytes_in_pixel_format(o3d3xx::pixel_format f);

} // end: namespace o3d3xx

#endif // __O3D3XX_IMAGE_H__
