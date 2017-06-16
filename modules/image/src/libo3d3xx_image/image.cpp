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

#include "o3d3xx_image/image.h"
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
#include "o3d3xx_camera/err.h"
#include "o3d3xx_framegrabber/byte_buffer.hpp"

o3d3xx::ImageBuffer::ImageBuffer()
  : o3d3xx::ByteBuffer(),
    cloud_(new pcl::PointCloud<o3d3xx::PointT>()),
    extrinsics_({0., 0., 0., 0., 0., 0.}),
    exposure_times_({0,0,0})
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

cv::Mat
o3d3xx::ImageBuffer::DepthImage()
{
  this->Organize();
  return this->depth_;
}

cv::Mat
o3d3xx::ImageBuffer::UnitVectors()
{
  this->Organize();
  return this->uvec_;
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

std::vector<float>
o3d3xx::ImageBuffer::Extrinsics()
{
  this->Organize();
  return this->extrinsics_;
}

std::vector<std::uint32_t>
o3d3xx::ImageBuffer::ExposureTimes()
{
  this->Organize();
  return this->exposure_times_;
}

float
o3d3xx::ImageBuffer::IlluTemp()
{
  this->Organize();
  return this->illu_temp_;
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
  std::size_t xidx, yidx, zidx, aidx, raw_aidx, cidx, didx, uidx = INVALID_IDX;
  std::size_t extidx = INVALID_IDX;

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
  uidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::UNIT_VECTOR_ALL);
  extidx =
    o3d3xx::get_chunk_index(this->bytes_,
                            o3d3xx::image_chunk::EXTRINSIC_CALIBRATION);

  // We *must* have the confidence image. If we do not, we bail out now
  if (cidx == INVALID_IDX)
    {
      throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
    }

  bool AMP_OK = aidx != INVALID_IDX;
  bool RAW_AMP_OK = raw_aidx != INVALID_IDX;
  bool RDIST_OK = didx != INVALID_IDX;
  bool UVEC_OK = uidx != INVALID_IDX;
  bool CARTESIAN_OK =
    (xidx != INVALID_IDX) && (yidx != INVALID_IDX) && (zidx != INVALID_IDX);
  bool EXTRINSICS_OK = extidx != INVALID_IDX;

  DLOG(INFO) << "xidx=" << xidx
             << ", yidx=" << yidx
             << ", zidx=" << zidx
             << ", aidx=" << aidx
             << ", raw_aidx=" << raw_aidx
             << ", cidx=" << cidx
             << ", didx=" << didx
             << ", uidx=" << uidx;

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
  std::size_t uincr = uidx != INVALID_IDX ? 4 * 3 : 0; // float32 * 3
  std::size_t extincr = extidx != INVALID_IDX ? 4 : 0; // float32

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
  this->cloud_->header.frame_id = "o3d3xx";
  this->cloud_->width = width;
  this->cloud_->height = height;
  this->cloud_->is_dense = true;
  this->cloud_->points.resize(num_points);

  this->xyz_image_.create(height, width, CV_16SC3);
  this->uvec_.create(height, width, CV_32FC3);
  this->conf_.create(height, width, CV_8UC1);
  this->depth_.create(height, width, CV_16UC1);
  this->amp_.create(height, width, CV_16UC1);
  this->raw_amp_.create(height, width, CV_16UC1);

  // move index pointers to where pixel data starts -- we assume
  // (I think safely) that all header sizes will be uniform in the data stream,
  // so, we use our invariant of the confidence image header
  std::uint32_t pixel_data_offset =
    o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+8);

  cidx += pixel_data_offset;
  didx += RDIST_OK ? pixel_data_offset : 0;
  uidx += UVEC_OK ? pixel_data_offset : 0;
  aidx += AMP_OK ? pixel_data_offset : 0;
  raw_aidx += RAW_AMP_OK ? pixel_data_offset : 0;
  if (CARTESIAN_OK)
    {
      xidx += pixel_data_offset;
      yidx += pixel_data_offset;
      zidx += pixel_data_offset;
    }
  extidx += EXTRINSICS_OK ? pixel_data_offset : 0;

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  std::uint16_t bad_pixel = std::numeric_limits<std::uint16_t>::quiet_NaN();
  std::int16_t bad_pixel_s = std::numeric_limits<std::int16_t>::quiet_NaN();

  int col = 0;
  int row = -1;
  int xyz_col = 0;
  int uvec_col = 0;

  std::uint16_t* depth_row_ptr;
  std::uint16_t* amp_row_ptr;
  std::uint16_t* raw_amp_row_ptr;
  std::uint8_t* conf_row_ptr;
  std::int16_t* xyz_row_ptr;
  float* uvec_row_ptr;

  std::int16_t x_, y_, z_;
  float e_x, e_y, e_z;

  for (std::size_t i = 0; i < num_points;
       ++i, xidx += xincr, yidx += yincr, zidx += zincr,
         cidx += cincr, aidx += aincr, didx += dincr,
         raw_aidx += raw_aincr, uidx += uincr)
    {
      o3d3xx::PointT& pt = this->cloud_->points[i];

      col = i % width;
      xyz_col = col * 3; // 3 channels: xyz
      uvec_col = col * 3; // 3 channels: u_x, u_y, u_z
      if (col == 0)
        {
          row += 1;
          depth_row_ptr = this->depth_.ptr<std::uint16_t>(row);
          amp_row_ptr = this->amp_.ptr<std::uint16_t>(row);
          raw_amp_row_ptr = this->raw_amp_.ptr<std::uint16_t>(row);
          conf_row_ptr = this->conf_.ptr<std::uint8_t>(row);
          xyz_row_ptr = this->xyz_image_.ptr<std::int16_t>(row);
          uvec_row_ptr = this->uvec_.ptr<float>(row);
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

      if (UVEC_OK)
        {
          e_x = o3d3xx::mkval<float>(this->bytes_.data()+uidx);
          e_y = o3d3xx::mkval<float>(this->bytes_.data()+uidx+4);
          e_z = o3d3xx::mkval<float>(this->bytes_.data()+uidx+8);

          uvec_row_ptr[uvec_col] = e_x;
          uvec_row_ptr[uvec_col + 1] = e_y;
          uvec_row_ptr[uvec_col + 2] = e_z;

          // Intentionlly no 'else' clause on this. We do not want to stomp
          // on any previously stored unit vector data.
        }

      pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
      pt.intensity = amp_row_ptr[col];
    }

  this->cloud_->sensor_origin_.setZero();
  this->cloud_->sensor_orientation_.w() = 1.0f;
  this->cloud_->sensor_orientation_.x() = 0.0f;
  this->cloud_->sensor_orientation_.y() = 0.0f;
  this->cloud_->sensor_orientation_.z() = 0.0f;

  //
  // Parse out the extrinsics
  //
  if (EXTRINSICS_OK)
    {
      for (std::size_t i = 0; i < 6; ++i, extidx += extincr)
        {
          this->extrinsics_[i] =
            o3d3xx::mkval<float>(this->bytes_.data()+extidx);
        }
    }
  else
    {
      LOG(WARNING) << "Extrinsics are invalid!";
    }

  //
  // OK, now we want to see if the temp illu and exposure times are present,
  // if they are, we want to parse them out and store them in the image buffer.
  // Since the extrinsics are invariant and should *always* be present, we use
  // the current index of the extrinsics.
  if (EXTRINSICS_OK)
    {
      std::size_t extime_idx = extidx;
      int bytes_left = this->bytes_.size() - extime_idx;

      // Read extime (6 bytes string + 3x 4 bytes uint32_t)
      if(bytes_left >= 18
	 && std::equal(this->bytes_.begin() + extidx,
		       this->bytes_.begin() + extidx + 6,
		       std::begin("extime")))
	{
	  extime_idx += 6;
	  bytes_left -= 6;

	  // 3 exposure times
	  for (std::size_t i = 0; i < 3; ++i)
	    {
	      if ((bytes_left - 6) <= 0)
		{
		  break;
		}
	      
	      std::uint32_t extime =
		o3d3xx::mkval<std::uint32_t>(
	          this->bytes_.data()+extime_idx);
	      
	      this->exposure_times_.at(i) = extime;
	      
	      extime_idx += 4;
	      bytes_left -= 4;
	    }
	}
      else
	{
	  std::fill(this->exposure_times_.begin(),
		    this->exposure_times_.end(), 0);
	}

      // Read temp_illu (9 bytes string + 4 bytes float)
      if(bytes_left >= 13
	 && std::equal(this->bytes_.begin() + extidx,
		       this->bytes_.begin() + extidx + 8,
		       std::begin("temp_illu")))
	{
	  extime_idx += 9;
	  bytes_left -= 9;

	  this->illu_temp_ =
	    o3d3xx::mkval<float>(this->bytes_.data() + extime_idx);

	  extime_idx += 4;
	  bytes_left -= 4;
	}
      else
	{
	  this->illu_temp_ = 0;
	}
    }
  else
    {
      LOG(WARNING) << "Checking for illu temp and exposure times skipped (cant trust extidx)";
    }



  this->_SetDirty(false);
}
