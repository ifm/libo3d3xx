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

#include "o3d3xx/frame_grabber.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <glog/logging.h>
#include "o3d3xx/image.h"
#include "o3d3xx/app_config.h"
#include "o3d3xx/device_config.h"
#include "o3d3xx/camera.hpp"
#include "o3d3xx/err.h"

const std::string o3d3xx::DEFAULT_PCIC_TCP_RESULT_SCHEMA =
  "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"ascii\" }, \
  \"elements\": [ { \"type\": \"string\", \"value\": \"star\", \"id\": \
  \"start_string\" }, { \"type\": \"blob\", \"id\": \
  \"normalized_amplitude_image\" }, { \"type\": \"blob\", \"id\": \
  \"distance_image\" }, { \"type\": \"blob\", \"id\": \"x_image\" }, { \
  \"type\": \"blob\", \"id\": \"y_image\" }, { \"type\": \"blob\", \"id\": \
  \"z_image\" }, { \"type\": \"blob\", \"id\": \"confidence_image\" }, { \
  \"type\": \"blob\", \"id\": \"diagnostic_data\" }, { \"type\": \"string\", \
  \"value\": \"stop\", \"id\": \"end_string\" } ] }";

o3d3xx::FrameGrabber::FrameGrabber(o3d3xx::Camera::Ptr cam)
  : cam_(cam),
    io_service_()
{
  // Set the result schema to a known byte stream
  // saving the original schema
  this->cam_->RequestSession();
  this->cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
  o3d3xx::DeviceConfig::Ptr dev = this->cam_->GetDeviceConfig();
  this->cam_->EditApplication(dev->ActiveApplication());
  o3d3xx::AppConfig::Ptr app = this->cam_->GetAppConfig();
  this->result_schema_ = app->PcicTcpResultSchema();
  LOG(WARNING) << "Temporarily applying the default result schema";
  app->SetPcicTcpResultSchema(o3d3xx::DEFAULT_PCIC_TCP_RESULT_SCHEMA);
  this->cam_->SetAppConfig(app.get());
  this->cam_->SaveApp();
  this->cam_->StopEditingApplication();
  this->cam_->CancelSession();

  // start the frame grabbing thread
  this->thread_ =
    std::unique_ptr<std::thread>(
      new std::thread(std::bind(&o3d3xx::FrameGrabber::Run, this)));
}

o3d3xx::FrameGrabber::~FrameGrabber()
{
  DLOG(INFO) << "FrameGrabber dtor running...";

  if (this->thread_ && this->thread_->joinable())
    {
      // NOTE: If Stop() was already called, that is fine
      // because the ASIO event loop is already done so the posted exception
      // will never get emitted.
      this->Stop();
      this->thread_->join();
    }

  // restore the original result schema
  try
    {
      this->cam_->RequestSession();
      this->cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
      o3d3xx::DeviceConfig::Ptr dev = this->cam_->GetDeviceConfig();
      this->cam_->EditApplication(dev->ActiveApplication());
      o3d3xx::AppConfig::Ptr app = this->cam_->GetAppConfig();
      LOG(WARNING) << "Restoring PCIC TCP Result Schema";
      app->SetPcicTcpResultSchema(this->result_schema_);
      this->cam_->SetAppConfig(app.get());
      this->cam_->SaveApp();
      this->cam_->StopEditingApplication();
      this->cam_->CancelSession();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Could not restore original result schema!"
                 << ex.what();
    }

  DLOG(INFO) << "FrameGrabber done.";
}

void
o3d3xx::FrameGrabber::Stop()
{
  this->io_service_.post([]() {
      throw o3d3xx::error_t(O3D3XX_THREAD_INTERRUPTED); });
}

bool
o3d3xx::FrameGrabber::WaitForFrame(o3d3xx::ImageBuffer* img,
                                   long timeout_millis,
                                   bool copy_buff,
                                   bool organize)
{
  // mutex will unlock in `unique_lock' dtor if not explicitly unlocked prior
  std::unique_lock<std::mutex> lock(this->front_buffer_mutex_);

  try
    {
      if (timeout_millis <= 0)
        {
          this->front_buffer_cv_.wait(lock);
        }
      else
        {
          if (this->front_buffer_cv_.wait_for(
               lock, std::chrono::milliseconds(timeout_millis)) ==
              std::cv_status::timeout)
            {
              LOG(WARNING) << "Timeout waiting for image buffer from camera";
              return false;
            }
        }
    }
  catch (const std::system_error& ex)
    {
      LOG(WARNING) << "WaitForFrame: " << ex.what();
      return false;
    }

  DLOG(INFO) << "Client fetching new image data";
  img->SetBytes(this->front_buffer_, copy_buff);
  lock.unlock();

  if (organize)
    {
      img->Organize();
    }

  return true;
}

void
o3d3xx::FrameGrabber::Run()
{
  boost::asio::io_service::work work(this->io_service_);

  //
  // setup the camera for image acquistion
  //
  std::string cam_ip;
  int cam_port;
  try
    {
      cam_ip = this->cam_->GetIP();
      cam_port = std::stoi(this->cam_->GetParameter("PcicTcpPort"));
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Could not get IP/Port of the camera: "
                 << ex.what();
      return;
    }

  LOG(INFO) << "Camera connection info: ip=" << cam_ip
            << ", port=" << cam_port;

  try
    {
      this->cam_->RequestSession();
       this->cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::RUN);
      this->cam_->CancelSession();
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Failed to setup camera for image acquisition: "
                 << ex.what();
      return;
    }

  //
  // init the asio structures
  //
  boost::asio::ip::tcp::socket sock(this->io_service_);
  boost::asio::ip::tcp::endpoint endpoint(
    boost::asio::ip::address::from_string(cam_ip), cam_port);

  //
  // Forward declare our two read handlers (because they need to call
  // eachother).
  //
  o3d3xx::FrameGrabber::ReadHandler ticket_handler;
  o3d3xx::FrameGrabber::ReadHandler image_handler;

  //
  // image data callback
  //
  std::size_t bytes_read = 0;
  std::size_t buff_sz = 0; // bytes

  image_handler =
    [&, this]
    (const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
      if (ec) { throw o3d3xx::error_t(ec.value()); }

      bytes_read += bytes_transferred;
      //DLOG(INFO) << "Read " << bytes_read << " image bytes of "
      //           << buff_sz;

      if (bytes_read == buff_sz)
        {
          DLOG(INFO) << "Got full image!";
          bytes_read = 0;

          // 1. verify the data
          if (o3d3xx::verify_image_buffer(this->back_buffer_))
            {
              DLOG(INFO) << "Image OK";

              // 2. move the data to the front buffer in O(1) time complexity
              this->front_buffer_mutex_.lock();
              this->back_buffer_.swap(this->front_buffer_);
              this->front_buffer_mutex_.unlock();

              // 3. notify waiting clients
              this->front_buffer_cv_.notify_all();
            }
          else
            {
              LOG(WARNING) << "Bad image!";
            }

          // read another ticket
          sock.async_read_some(
               boost::asio::buffer(this->ticket_buffer_.data(),
                                   o3d3xx::IMG_TICKET_SZ),
               ticket_handler);

          return;
        }

      sock.async_read_some(
        boost::asio::buffer(&this->back_buffer_[bytes_read],
                            buff_sz - bytes_read),
        image_handler);
    };

  //
  // ticket callback
  //
  std::size_t ticket_bytes_read = 0;
  std::size_t ticket_buff_sz = o3d3xx::IMG_TICKET_SZ;
  this->ticket_buffer_.resize(ticket_buff_sz);

  ticket_handler =
    [&, this]
    (const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
      if (ec) { throw o3d3xx::error_t(ec.value()); }

      ticket_bytes_read += bytes_transferred;
      DLOG(INFO) << "Read " << ticket_bytes_read
                 << " ticket bytes of " << ticket_buff_sz;

      if (ticket_bytes_read == ticket_buff_sz)
        {
          DLOG(INFO) << "Got full ticket!";
          ticket_bytes_read = 0;

          if (o3d3xx::verify_ticket_buffer(this->ticket_buffer_))
            {
              DLOG(INFO) << "Ticket OK";

              buff_sz = o3d3xx::get_image_buffer_size(this->ticket_buffer_);
              DLOG(INFO) << "Image buffer size: " << buff_sz;
              this->back_buffer_.resize(buff_sz);

              sock.async_read_some(
                   boost::asio::buffer(this->back_buffer_.data(),
                                       buff_sz),
                   image_handler);

              return;
            }

          LOG(WARNING) << "Bad ticket!";
        }

      sock.async_read_some(
           boost::asio::buffer(&this->ticket_buffer_[ticket_bytes_read],
                               ticket_buff_sz - ticket_bytes_read),
           ticket_handler);
    };

  //
  // connect to the sensor and start streaming in image data
  //
  try
    {
      sock.async_connect(endpoint,
                         [&, this]
                         (const boost::system::error_code& ec)
                         {
                           if (ec) { throw o3d3xx::error_t(ec.value()); }

                           sock.async_read_some(
                             boost::asio::buffer(
                               this->ticket_buffer_.data(), ticket_buff_sz),
                             ticket_handler);
                         });

      this->io_service_.run();
    }
  catch (const std::exception& ex)
    {
      //
      // In here we should discern why the exception with thrown.
      //
      // Special case the "Stop()" request from the control thread
      //

      LOG(WARNING) << "Exception: " << ex.what();
    }

  LOG(INFO) << "Framegrabber thread done.";
}
