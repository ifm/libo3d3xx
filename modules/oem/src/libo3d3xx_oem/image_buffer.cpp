/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#include <o3d3xx_oem/image_buffer.h>
#include <cstdint>
#include <limits>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <opencv2/core/core.hpp>
#include <o3d3xx_camera/err.h>
#include <o3d3xx_framegrabber/byte_buffer.hpp>
#include <o3d3xx_framegrabber/pcic_schema.h>
#include <pcl/point_cloud.h>
#include <glog/logging.h>
#include <resultsync.hpp>

o3d3xx::oem::ImageBuffer::ImageBuffer()
  : cloud_(new pcl::PointCloud<o3d3xx::oem::PointT>()),
    extrinsics_({0.,0.,0.,0.,0.,0.}),
    exposure_times_({0,0,0}),
    illu_temperature_(INVALID_TEMPERATURE),
    time_stamp_(std::chrono::system_clock::now())
{
  this->cloud_->sensor_origin_.setZero();
  this->cloud_->sensor_orientation_.w() = 1.0f;
  this->cloud_->sensor_orientation_.x() = 0.0f;
  this->cloud_->sensor_orientation_.y() = 0.0f;
  this->cloud_->sensor_orientation_.z() = 0.0f;
}

cv::Mat
o3d3xx::oem::ImageBuffer::DepthImage()
{
  return this->depth_;
}

cv::Mat
o3d3xx::oem::ImageBuffer::UnitVectors()
{
  return this->uvec_;
}

cv::Mat
o3d3xx::oem::ImageBuffer::AmplitudeImage()
{
  return this->amp_;
}

cv::Mat
o3d3xx::oem::ImageBuffer::RawAmplitudeImage()
{
  return this->raw_amp_;
}

cv::Mat
o3d3xx::oem::ImageBuffer::ConfidenceImage()
{
  return this->conf_;
}

cv::Mat
o3d3xx::oem::ImageBuffer::XYZImage()
{
  return this->xyz_image_;
}

pcl::PointCloud<o3d3xx::oem::PointT>::Ptr
o3d3xx::oem::ImageBuffer::Cloud()
{
  return this->cloud_;
}

std::vector<float>
o3d3xx::oem::ImageBuffer::Extrinsics()
{
  return this->extrinsics_;
}

std::vector<std::uint32_t>
o3d3xx::oem::ImageBuffer::ExposureTimes()
{
  return this->exposure_times_;
}

float
o3d3xx::oem::ImageBuffer::IlluTemperature()
{
  return this->illu_temperature_;
}

o3d3xx::oem::TimePointT
o3d3xx::oem::ImageBuffer::TimeStamp()
{
  return this->time_stamp_;
}

void
o3d3xx::oem::ImageBuffer::Organize(ifm::resultsync::Frame* frame,
                                   std::uint16_t mask)
{
  if (frame == nullptr)
    {
      LOG(WARNING) << "nullptr passed to Organize!";
      return;
    }

  // pixel index for each image
  std::size_t xidx = 0;
  std::size_t yidx = 0;
  std::size_t zidx = 0;
  std::size_t aidx = 0;
  std::size_t raw_aidx = 0;
  std::size_t cidx = 0;
  std::size_t didx = 0;
  std::size_t uidx = 0;
  std::size_t extidx = 0;

  // number of bytes to increment for each image (exploits a-priori
  // information about pixel types for each respective image)
  std::size_t cincr = 1; // uint8
  std::size_t xincr = 2; // int16
  std::size_t yincr = 2; // int16
  std::size_t zincr = 2; // int16
  std::size_t aincr = 2; // uint16
  std::size_t raw_aincr = 2; // uint16
  std::size_t dincr = 2; // uint16
  std::size_t uincr = 12; // 4*3=12 --> float32 * 3
  std::size_t extincr = 4; // float32

  //
  // We always populate the confidence image, so we use its headers to grab
  // some meta-data about all images as well
  //
  std::size_t conf_sz;
  const std::uint8_t* conf_ptr =
    reinterpret_cast<const std::uint8_t*>(
      frame->constData("confidence_image", &conf_sz));

  if (conf_sz < (cidx + 20))
    {
      LOG(ERROR) << "Confidence image error: size=" << conf_sz;
      throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
    }

  std::uint32_t width = o3d3xx::mkval<std::uint32_t>(conf_ptr+cidx+16);
  std::uint32_t height = o3d3xx::mkval<std::uint32_t>(conf_ptr+cidx+20);
  std::uint32_t num_points = width * height;
  std::uint32_t pixel_data_offset =
    o3d3xx::mkval<std::uint32_t>(conf_ptr+cidx+8);

  const std::uint32_t header_version =  o3d3xx::mkval<std::uint32_t>(conf_ptr+cidx+12);
  // for the *big* time stamp minimum header version 2 is needed
  if( header_version > 1 )
    {
      // Retrieve the timestamp information from the confidence data
      const std::uint32_t timestampSec =
        o3d3xx::mkval<std::uint32_t>(conf_ptr+cidx+40);
      const std::uint32_t timestampNsec =
        o3d3xx::mkval<std::uint32_t>(conf_ptr+cidx+44);
      // convert the time stamp into a TimePointT
      this->time_stamp_ = std::chrono::system_clock::time_point {
                            std::chrono::seconds{timestampSec}
                            + std::chrono::nanoseconds{timestampNsec}
                          };
    }
  else
    {
      // There is no *big* time stamp in chunk version 1
      this->time_stamp_ = std::chrono::system_clock::now();
    }

  cidx += pixel_data_offset; // advance to beginning of pixel data

  //
  // Get initial pointers to data buffers
  //
  std::size_t d_sz = 0;
  std::size_t a_sz = 0;
  std::size_t raw_a_sz = 0;
  std::size_t x_sz = 0;
  std::size_t y_sz = 0;
  std::size_t z_sz = 0;
  std::size_t u_sz = 0;
  std::size_t ext_sz = 0;

  const std::uint8_t* d_ptr = nullptr;
  const std::uint8_t* a_ptr = nullptr;
  const std::uint8_t* raw_a_ptr = nullptr;
  const std::uint8_t* x_ptr = nullptr;
  const std::uint8_t* y_ptr = nullptr;
  const std::uint8_t* z_ptr = nullptr;
  const std::uint8_t* u_ptr = nullptr;
  const std::uint8_t* ext_ptr = nullptr;

  //
  // setup images
  //
  this->conf_.create(height, width, CV_8UC1);

  if ((mask & o3d3xx::IMG_RDIS) == o3d3xx::IMG_RDIS)
    {
      this->depth_.create(height, width, CV_16UC1);

      d_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("distance_image", &d_sz));

      if (d_sz != (pixel_data_offset + (num_points * dincr)))
        {
          LOG(ERROR) << "Radial distance image error: size=" << d_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      didx += pixel_data_offset;
    }

  if ((mask & o3d3xx::IMG_AMP) == o3d3xx::IMG_AMP)
    {
      this->amp_.create(height, width, CV_16UC1);

      a_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("normalized_amplitude_image", &a_sz));

      if (a_sz != (pixel_data_offset + (num_points * aincr)))
        {
          LOG(ERROR) << "Amplitude image error: size=" << a_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      aidx += pixel_data_offset;
    }

  if ((mask & o3d3xx::IMG_RAMP) == o3d3xx::IMG_RAMP)
    {
      this->raw_amp_.create(height, width, CV_16UC1);

      raw_a_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("amplitude_image", &raw_a_sz));

      if (raw_a_sz != (pixel_data_offset + (num_points * raw_aincr)))
        {
          LOG(ERROR) << "Raw amplitude image error: size=" << raw_a_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      raw_aidx += pixel_data_offset;
    }

  if ((mask & o3d3xx::IMG_CART) == o3d3xx::IMG_CART)
    {
      this->cloud_->header.frame_id = "o3d3xx";
      this->cloud_->width = width;
      this->cloud_->height = height;
      this->cloud_->is_dense = true;
      this->cloud_->points.resize(num_points);

      this->xyz_image_.create(height, width, CV_16SC3);

      x_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("x_image", &x_sz));

      if (x_sz != (pixel_data_offset + (num_points * xincr)))
        {
          LOG(ERROR) << "X image error: size=" << x_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      xidx += pixel_data_offset;

      y_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("y_image", &y_sz));

      if (y_sz != (pixel_data_offset + (num_points * yincr)))
        {
          LOG(ERROR) << "Y image error: size=" << y_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      yidx += pixel_data_offset;

      z_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("z_image", &z_sz));

      if (z_sz != (pixel_data_offset + (num_points * zincr)))
        {
          LOG(ERROR) << "Z image error: size=" << z_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      zidx += pixel_data_offset;
    }

  if ((mask & o3d3xx::IMG_UVEC) == o3d3xx::IMG_UVEC)
    {
      this->uvec_.create(height, width, CV_32FC3);

      u_ptr =
        reinterpret_cast<const std::uint8_t*>(
          frame->constData("all_unit_vector_matrices", &u_sz));

      if (u_sz != (pixel_data_offset + (num_points * uincr)))
        {
          LOG(ERROR) << "Uvec image error: size=" << u_sz;
          throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
        }

      uidx += pixel_data_offset;
    }

  // libo3d3xx treats the extrinsics as an invariant
  ext_ptr =
    reinterpret_cast<const std::uint8_t*>(
      frame->constData("extrinsic_calibration", &ext_sz));

  if (ext_sz != (pixel_data_offset + (6 * 4)))
    {
      LOG(ERROR) << "Extrinsics error: size=" << ext_sz;
      throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
    }

  extidx += pixel_data_offset;

  //
  // Copy in the pixel data
  //

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  std::uint16_t bad_pixel = std::numeric_limits<std::uint16_t>::quiet_NaN();
  std::uint16_t bad_pixel_s = std::numeric_limits<std::int16_t>::quiet_NaN();

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
      o3d3xx::oem::PointT& pt = this->cloud_->points[i];

      col = i % width;
      xyz_col = col * 3;
      uvec_col = col * 3;

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

      conf_row_ptr[col] = conf_ptr[cidx];
      if ((conf_row_ptr[col] & 0x1) == 0x1)
        {
          //
          // bad pixel
          //
          if ((mask & o3d3xx::IMG_CART) == o3d3xx::IMG_CART)
            {
              pt.x = pt.y = pt.z = bad_point;
              this->cloud_->is_dense = false;

              xyz_row_ptr[xyz_col] = bad_pixel_s;
              xyz_row_ptr[xyz_col + 1] = bad_pixel_s;
              xyz_row_ptr[xyz_col + 2] = bad_pixel_s;

              pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
              pt.intensity = bad_pixel;
            }

          if ((mask & o3d3xx::IMG_RDIS) == o3d3xx::IMG_RDIS)
            {
              depth_row_ptr[col] = bad_pixel;
            }
        }
      else
        {
          //
          // good pixel
          //
          if ((mask & o3d3xx::IMG_CART) == o3d3xx::IMG_CART)
            {
              // convert the coord frame to a right-handed frame for the point
              // cloud
              x_ = o3d3xx::mkval<std::int16_t>(z_ptr + zidx);
              y_ = -o3d3xx::mkval<std::int16_t>(x_ptr + xidx);
              z_ = -o3d3xx::mkval<std::int16_t>(y_ptr + yidx);

              // convert units to meters for point cloud
              pt.x = x_ / 1000.f;
              pt.y = y_ / 1000.f;
              pt.z = z_ / 1000.f;

              // keep xyz image in mm
              xyz_row_ptr[xyz_col] = x_;
              xyz_row_ptr[xyz_col + 1] = y_;
              xyz_row_ptr[xyz_col + 2] = z_;

              pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
              pt.intensity = bad_pixel;
            }

          if ((mask & o3d3xx::IMG_RDIS) == o3d3xx::IMG_RDIS)
            {
              depth_row_ptr[col] = o3d3xx::mkval<std::uint16_t>(d_ptr + didx);
            }
        }

      // regardless of whether the pixel was flagged as good/bad,
      // we populate the amplitude, raw amplitude, and unit vectors

      if ((mask & o3d3xx::IMG_AMP) == o3d3xx::IMG_AMP)
        {
          amp_row_ptr[col] = o3d3xx::mkval<std::uint16_t>(a_ptr + aidx);
          if ((mask & o3d3xx::IMG_CART) == o3d3xx::IMG_CART)
            {
              pt.intensity = amp_row_ptr[col];
            }
        }

      if ((mask & o3d3xx::IMG_RAMP) == o3d3xx::IMG_RAMP)
        {
          raw_amp_row_ptr[col] =
            o3d3xx::mkval<std::uint16_t>(raw_a_ptr + raw_aidx);
        }

      if ((mask & o3d3xx::IMG_UVEC) == o3d3xx::IMG_UVEC)
        {
          e_x = o3d3xx::mkval<float>(u_ptr + uidx);
          e_y = o3d3xx::mkval<float>(u_ptr + uidx + 4);
          e_z = o3d3xx::mkval<float>(u_ptr + uidx + 8);

          uvec_row_ptr[uvec_col] = e_x;
          uvec_row_ptr[uvec_col + 1] = e_y;
          uvec_row_ptr[uvec_col + 2] = e_z;
        }
    }

  // extrinsics are an invariant
  for (std::size_t i = 0; i < 6; ++i, extidx += extincr)
    {
      this->extrinsics_[i] = o3d3xx::mkval<float>(ext_ptr + extidx);
    }

  if ((mask & o3d3xx::EXP_TIME) == o3d3xx::EXP_TIME)
  {
    try
    {
      std::string raw_model_result = frame->getInfo("ModelResult");
      boost::property_tree::ptree pt;
      std::istringstream is(raw_model_result);
      is.seekg(0, is.beg);
      boost::property_tree::read_json(is, pt);

      uint32_t exposure_time_1 = pt.get<uint32_t>("ExposureTime1", 0);
      uint32_t exposure_time_2 = pt.get<uint32_t>("ExposureTime2", 0);
      uint32_t exposure_time_3 = pt.get<uint32_t>("ExposureTime3", 0);

      exposure_times_.at(0) = exposure_time_1;
      exposure_times_.at(1) = exposure_time_2;
      exposure_times_.at(2) = exposure_time_3;
    }
    catch (const boost::property_tree::ptree_bad_path& ex)
    {
      LOG(WARNING) << "In `Organize(...)', skipping exposure times";
      LOG(WARNING) << "ptree_bad_path: " << ex.what();
    }
  }
  else
  {
    exposure_times_ = { 0, 0, 0 };
  }

  if ((mask & o3d3xx::ILLU_TEMP) == o3d3xx::ILLU_TEMP)
  {
    try
    {
      std::string raw_diagnostic_data = frame->getInfo("DiagnosticData");
      boost::property_tree::ptree pt;
      std::istringstream is(raw_diagnostic_data);
      is.seekg(0, is.beg);
      boost::property_tree::read_json(is, pt);

      illu_temperature_ = pt.get<float>("TemperatureIllu", 3276.7f);
    }
    catch (const boost::property_tree::ptree_bad_path& ex)
    {
      LOG(WARNING) << "In `Organize(...)', skipping illu temperature";
      LOG(WARNING) << "ptree_bad_path: " << ex.what();
    }
  }
  else
  {
    illu_temperature_ = INVALID_TEMPERATURE;
  }

}
