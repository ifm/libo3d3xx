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
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "o3d3xx/util.hpp"
#include "o3d3xx/err.h"

const std::size_t o3d3xx::IMG_TICKET_SZ = 16;
const std::size_t o3d3xx::IMG_CHUNK_HEADER_SZ = 36;

const std::uint16_t o3d3xx::IMG_RDIS = 1;  // 2^0
const std::uint16_t o3d3xx::IMG_AMP  = 2;  // 2^1
const std::uint16_t o3d3xx::IMG_RAMP = 4;  // 2^2
const std::uint16_t o3d3xx::IMG_CART = 8;  // 2^3
const std::uint16_t o3d3xx::IMG_UVEC = 16; // 2^4

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

  //throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
  return std::numeric_limits<std::size_t>::max();
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

    case o3d3xx::pixel_format::FORMAT_16U2:
      return 2 * 2;

    case o3d3xx::pixel_format::FORMAT_32F3:
      return 4 * 3;

    default:
      return 0;
    }
}

o3d3xx::ImageBuffer::ImageBuffer()
  : dirty_(false),
    cloud_(new pcl::PointCloud<o3d3xx::PointT>())
{ }

o3d3xx::ImageBuffer::ImageBuffer(const o3d3xx::ImageBuffer& src_buff)
  : o3d3xx::ImageBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

o3d3xx::ImageBuffer&
o3d3xx::ImageBuffer::operator= (const o3d3xx::ImageBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

o3d3xx::ImageBuffer::~ImageBuffer()
{
  DLOG(INFO) << "Image buffer dtor";
}

void
o3d3xx::ImageBuffer::SetBytes(std::vector<std::uint8_t>& buff,
                              bool copy)
{
  if (copy)
    {
      std::size_t sz = buff.size();
      this->bytes_.resize(sz);

      std::copy(buff.begin(),
                buff.begin() + sz,
                this->bytes_.begin());
    }
  else
    {
      buff.swap(this->bytes_);
    }

  this->_SetDirty(true);
}

void
o3d3xx::ImageBuffer::_SetDirty(bool flg) noexcept
{
  this->dirty_ = flg;
}

bool
o3d3xx::ImageBuffer::Dirty() const noexcept
{
  return this->dirty_;
}

cv::Mat
o3d3xx::ImageBuffer::DepthImage()
{
  this->Organize();
  return this->depth_;
}

cv::Mat
o3d3xx::ImageBuffer::AmplitudeImage()
{
  this->Organize();
  return this->amp_;
}

cv::Mat
o3d3xx::ImageBuffer::RawAmplitudeImage()
{
  this->Organize();
  return this->raw_amp_;
}

cv::Mat
o3d3xx::ImageBuffer::ConfidenceImage()
{
  this->Organize();
  return this->conf_;
}

cv::Mat
o3d3xx::ImageBuffer::XYZImage()
{
  this->Organize();
  return this->xyz_image_;
}

pcl::PointCloud<o3d3xx::PointT>::Ptr
o3d3xx::ImageBuffer::Cloud()
{
  this->Organize();
  return this->cloud_;
}

std::vector<std::uint8_t>
o3d3xx::ImageBuffer::Bytes()
{
  return this->bytes_;
}

void
o3d3xx::ImageBuffer::Organize()
{
  if (! this->Dirty())
    {
      return;
    }

  // get indices to the start of each chunk of interest in the image buffer
  // NOTE: These could get optimized by using apriori values if necessary
  std::size_t INVALID_IDX = std::numeric_limits<std::size_t>::max();
  std::size_t xidx, yidx, zidx, aidx, raw_aidx, cidx, didx = INVALID_IDX;

  xidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CARTESIAN_X);
  yidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CARTESIAN_Y);
  zidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CARTESIAN_Z);
  aidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::AMPLITUDE);
  raw_aidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::RAW_AMPLITUDE);
  cidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CONFIDENCE);
  didx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::RADIAL_DISTANCE);

  // We *must* have the confidence image. If we do not, we bail out now
  if (cidx == INVALID_IDX)
    {
      throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
    }

  bool AMP_OK = aidx != INVALID_IDX;
  bool RAW_AMP_OK = raw_aidx != INVALID_IDX;
  bool RDIST_OK = didx != INVALID_IDX;
  bool CARTESIAN_OK =
    (xidx != INVALID_IDX) && (yidx != INVALID_IDX) && (zidx != INVALID_IDX);

  DLOG(INFO) << "xidx=" << xidx
             << ", yidx=" << yidx
             << ", zidx=" << zidx
             << ", aidx=" << aidx
             << ", raw_aidx=" << raw_aidx
             << ", cidx=" << cidx
             << ", didx=" << didx;

  // Get how many bytes to increment in the buffer for each pixel
  // NOTE: These can be discovered dynamically, however, for now we use our a
  // priori info of the pixel data types
  std::size_t cincr = 1; // uint8_t
  std::size_t xincr = xidx != INVALID_IDX ? 2 : 0; // int16_t
  std::size_t yincr = yidx != INVALID_IDX ? 2 : 0; // int16_t
  std::size_t zincr = zidx != INVALID_IDX ? 2 : 0; // int16_t
  std::size_t aincr = aidx != INVALID_IDX ? 2 : 0; // uint16_t
  std::size_t raw_aincr = raw_aidx != INVALID_IDX ? 2 : 0; // uint16_t
  std::size_t dincr = didx != INVALID_IDX ? 2 : 0; // uint16_t

  // NOTE: we use the `cidx' corresponding to the confidence image because
  // it is an invariant in terms of what we send to the camera as valid pcic
  // schemas we wish to process.
  std::uint32_t width =
    o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+16);
  std::uint32_t height =
    o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+20);

  std::uint32_t num_points = width * height;

  DLOG(INFO) << "width=" << width
             << ", height=" << height
             << ", num_points=" << num_points;

  //
  // setup images
  //
  this->cloud_->header.frame_id = "/o3d3xx";
  this->cloud_->width = width;
  this->cloud_->height = height;
  this->cloud_->is_dense = true;
  this->cloud_->points.resize(num_points);

  this->xyz_image_.create(height, width, CV_16SC3);
  this->conf_.create(height, width, CV_8UC1);
  this->depth_.create(height, width, CV_16UC1);
  this->amp_.create(height, width, CV_16UC1);
  this->raw_amp_.create(height, width, CV_16UC1);

  // move index pointers to where pixel data starts
  cidx += o3d3xx::IMG_CHUNK_HEADER_SZ;
  didx += RDIST_OK ? o3d3xx::IMG_CHUNK_HEADER_SZ : 0;
  aidx += AMP_OK ? o3d3xx::IMG_CHUNK_HEADER_SZ : 0;
  raw_aidx += RAW_AMP_OK ? o3d3xx::IMG_CHUNK_HEADER_SZ : 0;
  if (CARTESIAN_OK)
    {
      xidx += o3d3xx::IMG_CHUNK_HEADER_SZ;
      yidx += o3d3xx::IMG_CHUNK_HEADER_SZ;
      zidx += o3d3xx::IMG_CHUNK_HEADER_SZ;
    }

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  std::uint16_t bad_pixel = std::numeric_limits<std::uint16_t>::quiet_NaN();
  std::int16_t bad_pixel_s = std::numeric_limits<std::int16_t>::quiet_NaN();

  int col = 0;
  int row = -1;
  int xyz_col = 0;

  std::uint16_t* depth_row_ptr;
  std::uint16_t* amp_row_ptr;
  std::uint16_t* raw_amp_row_ptr;
  std::uint8_t* conf_row_ptr;
  std::int16_t* xyz_row_ptr;

  std::int16_t x_, y_, z_;

  for (std::size_t i = 0; i < num_points;
       ++i, xidx += xincr, yidx += yincr, zidx += zincr,
         cidx += cincr, aidx += aincr, didx += dincr,
         raw_aidx += raw_aincr)
    {
      o3d3xx::PointT& pt = this->cloud_->points[i];

      col = i % width;
      xyz_col = col * 3; // 3 channels: xyz
      if (col == 0)
        {
          row += 1;
          depth_row_ptr = this->depth_.ptr<std::uint16_t>(row);
          amp_row_ptr = this->amp_.ptr<std::uint16_t>(row);
          raw_amp_row_ptr = this->raw_amp_.ptr<std::uint16_t>(row);
          conf_row_ptr = this->conf_.ptr<std::uint8_t>(row);
          xyz_row_ptr = this->xyz_image_.ptr<std::int16_t>(row);
        }

      conf_row_ptr[col] = this->bytes_.at(cidx);
      if ((conf_row_ptr[col] & 0x1) == 1)
        {
          pt.x = pt.y = pt.z = bad_point;
          this->cloud_->is_dense = false;

          depth_row_ptr[col] = bad_pixel;

          xyz_row_ptr[xyz_col] = bad_pixel_s;
          xyz_row_ptr[xyz_col + 1] = bad_pixel_s;
          xyz_row_ptr[xyz_col + 2] = bad_pixel_s;
        }
      else
        {
          // convert the coord frame  to a right-handed frame for the point
          // cloud
          if (CARTESIAN_OK)
            {
              x_ = o3d3xx::mkval<std::int16_t>(this->bytes_.data()+zidx);
              y_ = -o3d3xx::mkval<std::int16_t>(this->bytes_.data()+xidx);
              z_ = -o3d3xx::mkval<std::int16_t>(this->bytes_.data()+yidx);

              // convert units to meters for the point cloud
              pt.x = x_ / 1000.0f;
              pt.y = y_ / 1000.0f;
              pt.z = z_ / 1000.0f;
            }
          else
            {
              x_ = bad_pixel_s;
              y_ = bad_pixel_s;
              z_ = bad_pixel_s;

              pt.x = pt.y = pt.z = bad_point;
              this->cloud_->is_dense = false;
            }

          // keep depth image data as mm
          if (RDIST_OK)
            {
              depth_row_ptr[col] =
                o3d3xx::mkval<std::uint16_t>(this->bytes_.data()+didx);
            }
          else
            {
              depth_row_ptr[col] = bad_pixel;
            }

          // keep xyz image data as mm but same coord frame as point cloud
          xyz_row_ptr[xyz_col] = x_;
          xyz_row_ptr[xyz_col + 1] = y_;
          xyz_row_ptr[xyz_col + 2] = z_;
        }

      if (AMP_OK)
        {
          amp_row_ptr[col] =
            o3d3xx::mkval<std::uint16_t>(this->bytes_.data()+aidx);
        }
      else
        {
          amp_row_ptr[col] = bad_pixel;
        }

      if (RAW_AMP_OK)
        {
          raw_amp_row_ptr[col] =
            o3d3xx::mkval<std::uint16_t>(this->bytes_.data()+raw_aidx);
        }
      else
        {
          raw_amp_row_ptr[col] = bad_pixel;
        }

      pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
      pt.intensity = amp_row_ptr[col];
    }

  this->cloud_->sensor_origin_.setZero();
  this->cloud_->sensor_orientation_.w() = 1.0f;
  this->cloud_->sensor_orientation_.x() = 0.0f;
  this->cloud_->sensor_orientation_.y() = 0.0f;
  this->cloud_->sensor_orientation_.z() = 0.0f;

  this->_SetDirty(false);
}
