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

#include "o3d3xx_framegrabber/frame_grabber.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <glog/logging.h>
#include "o3d3xx_camera/camera.hpp"
#include "o3d3xx_camera/err.h"
#include "o3d3xx_camera/device_config.h"
#include "o3d3xx_camera/app_config.h"
#include "o3d3xx_framegrabber/byte_buffer.hpp"
#include "o3d3xx_framegrabber/pcic_schema.h"

// <Ticket><Length>CR+LF (16 bytes)
const std::size_t o3d3xx::TICKET_ID_SZ = 16;
const std::string o3d3xx::TICKET_image = "0000";
const std::string o3d3xx::TICKET_c = "1000";
const std::string o3d3xx::TICKET_t = "1001";

o3d3xx::FrameGrabber::FrameGrabber(o3d3xx::Camera::Ptr cam,
                                   std::uint16_t mask)
  : cam_(cam),
    io_service_(),
    sock_(io_service_),
    pcic_ready_(false),
    mask_(mask)
{
  this->SetSchemaBuffer(this->mask_);
  this->SetTriggerBuffer();

  try
    {
      this->cam_ip_ = this->cam_->GetIP();
      this->cam_port_ = std::stoi(this->cam_->GetParameter("PcicTcpPort"));
    }
  catch (const o3d3xx::error_t& ex)
    {
      LOG(ERROR) << "Could not get IP/Port of the camera: "
                 << ex.what();
      // NOTE: GetIP() won't throw, so, the problem must be getting the PCIC
      // port. Here we assume the default. Former behavior was to throw!
      LOG(WARNING) << "Assuming default PCIC port!";
      this->cam_port_ = o3d3xx::DEFAULT_PCIC_PORT;
    }

  LOG(INFO) << "Camera connection info: ip=" << this->cam_ip_
            << ", port=" << this->cam_port_;

  this->endpoint_ =
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(this->cam_ip_), this->cam_port_);

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

  DLOG(INFO) << "FrameGrabber done.";
}

void
o3d3xx::FrameGrabber::SetSchemaBuffer(std::uint16_t mask)
{
  std::string schema = o3d3xx::make_pcic_schema(mask);
  int c_len = 4 + 1 + 9 + schema.size() + 2;
  std::ostringstream str;
  str << o3d3xx::TICKET_c
      << 'L' << std::setfill('0') << std::setw(9) << c_len
      << '\r' << '\n'
      << o3d3xx::TICKET_c << 'c'
      << std::setfill('0') << std::setw(9)
      << schema.size()
      << schema
      << '\r' << '\n';

  std::string c_command = str.str();
  this->schema_buffer_.assign(c_command.begin(), c_command.end());
  DLOG(INFO) << "c_command: " << c_command;
}

void
o3d3xx::FrameGrabber::SetTriggerBuffer()
{
  int t_len = 4 + 1 + 2;
  std::ostringstream str;
  str << o3d3xx::TICKET_t
      << 'L' << std::setfill('0') << std::setw(9) << t_len
      << '\r' << '\n'
      << o3d3xx::TICKET_t << 't' << '\r' << '\n';

  std::string t_command = str.str();
  this->trigger_buffer_.assign(t_command.begin(), t_command.end());
}

void
o3d3xx::FrameGrabber::SWTrigger()
{
  int i = 0;
  while (! this->pcic_ready_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      i++;

      if (i > 2000)
        {
          LOG(WARNING) << "pcic_ready_ flag not set!";
          return;
        }
    }

  this->io_service_.post(
    [=]()
    {
      boost::asio::async_write(
        this->sock_,
        boost::asio::buffer(this->trigger_buffer_.data(),
                            this->trigger_buffer_.size()),
        [=] (const boost::system::error_code& ec,
             std::size_t bytes_transferred)
        {
          if (ec) { throw o3d3xx::error_t(ec.value()); }
        });
    });
}

void
o3d3xx::FrameGrabber::Stop()
{
  this->io_service_.post([]() {
      throw o3d3xx::error_t(O3D3XX_THREAD_INTERRUPTED); });
}

bool
o3d3xx::FrameGrabber::WaitForFrame(o3d3xx::ByteBuffer* buff,
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
  buff->SetBytes(this->front_buffer_, copy_buff);
  lock.unlock();

  if (organize)
    {
      buff->Organize();
    }

  return true;
}

void
o3d3xx::FrameGrabber::ImageHandler(const boost::system::error_code& ec,
                                   std::size_t bytes_transferred,
                                   std::size_t bytes_read)
{
  if (ec) { throw o3d3xx::error_t(ec.value()); }

  bytes_read += bytes_transferred;

  if (bytes_read != this->back_buffer_.size())
    {
      this->sock_.async_read_some(
        boost::asio::buffer(&this->back_buffer_[bytes_read],
                            this->back_buffer_.size() - bytes_read),
        std::bind(&o3d3xx::FrameGrabber::ImageHandler, this,
                  std::placeholders::_1, std::placeholders::_2, bytes_read));
      return;
    }

  if (o3d3xx::verify_image_buffer(this->back_buffer_))
    {
      // Move data to front buffer in O(1)
      this->front_buffer_mutex_.lock();
      this->back_buffer_.swap(this->front_buffer_);
      this->front_buffer_mutex_.unlock();

      // notify waiting clients
      this->front_buffer_cv_.notify_all();
    }
  else
    {
      LOG(WARNING) << "Bad image!";
    }

  this->ticket_buffer_.clear();
  this->ticket_buffer_.resize(o3d3xx::TICKET_ID_SZ);
  this->sock_.async_read_some(
    boost::asio::buffer(this->ticket_buffer_.data(),
                        o3d3xx::TICKET_ID_SZ),
    std::bind(&o3d3xx::FrameGrabber::TicketHandler, this,
              std::placeholders::_1, std::placeholders::_2, 0));
}

void
o3d3xx::FrameGrabber::TicketHandler(const boost::system::error_code& ec,
                                    std::size_t bytes_transferred,
                                    std::size_t bytes_read)
{
  if (ec) { throw o3d3xx::error_t(ec.value()); }

  bytes_read += bytes_transferred;

  if (bytes_read < o3d3xx::TICKET_ID_SZ)
    {
      bytes_read +=
        boost::asio::read(
          this->sock_,
          boost::asio::buffer(&this->ticket_buffer_[bytes_read],
                              o3d3xx::TICKET_ID_SZ - bytes_read));

      if (bytes_read != o3d3xx::TICKET_ID_SZ)
        {
          LOG(ERROR) << "Timeout reading ticket!";
          throw o3d3xx::error_t(O3D3XX_IO_ERROR);
        }
    }

  std::string ticket;
  ticket.assign(this->ticket_buffer_.begin(),
                this->ticket_buffer_.begin() + 4);

  std::string payload_sz_str;
  payload_sz_str.assign(this->ticket_buffer_.begin()+5,
                        this->ticket_buffer_.begin()+14);
  int payload_sz = std::stoi(payload_sz_str);
  int ticket_sz = o3d3xx::TICKET_ID_SZ;

  if (ticket != o3d3xx::TICKET_image)
    {
      this->ticket_buffer_.resize(ticket_sz + payload_sz);

      bytes_read +=
        boost::asio::read(
          this->sock_,
          boost::asio::buffer(&this->ticket_buffer_[bytes_read],
                              (ticket_sz + payload_sz) - bytes_read));

      if (bytes_read != (ticket_sz + payload_sz))
        {
          LOG(ERROR) << "Timeout reading whole response!";
          LOG(ERROR) << "Got " << bytes_read << " bytes of "
                     << ticket_sz << " bytes expected";

          throw o3d3xx::error_t(O3D3XX_IO_ERROR);
        }
    }

  std::string ticket_str;
  ticket_str.assign(this->ticket_buffer_.begin(),
                    this->ticket_buffer_.end());
  DLOG(INFO) << "Ticket Full: '" << ticket_str << "'";

  if (ticket == o3d3xx::TICKET_image)
    {
      if (o3d3xx::verify_ticket_buffer(this->ticket_buffer_))
        {
          this->back_buffer_.resize(
            o3d3xx::get_image_buffer_size(this->ticket_buffer_));

          this->sock_.async_read_some(
            boost::asio::buffer(this->back_buffer_.data(),
                                this->back_buffer_.size()),
            std::bind(&o3d3xx::FrameGrabber::ImageHandler, this,
                      std::placeholders::_1, std::placeholders::_2, 0));
          return;
        }
      else
        {
          LOG(ERROR) << "Bad image ticket: " << ticket_str;
          throw(o3d3xx::error_t(O3D3XX_PCIC_BAD_REPLY));
        }
    }
  else if ((ticket == o3d3xx::TICKET_c) ||
           (ticket == o3d3xx::TICKET_t))
    {
      if (this->ticket_buffer_.at(20) != '*')
        {
          LOG(ERROR) << "Bad ticket: " << ticket_str;

          if ((ticket == o3d3xx::TICKET_t) &&
              (this->ticket_buffer_.at(20) == '!'))
            {
              LOG(WARNING) << "Are you software triggering in free run mode?";
            }
          else
            {
              throw(o3d3xx::error_t(O3D3XX_PCIC_BAD_REPLY));
            }
        }

      this->ticket_buffer_.clear();
      this->ticket_buffer_.resize(o3d3xx::TICKET_ID_SZ);
      this->sock_.async_read_some(
        boost::asio::buffer(this->ticket_buffer_.data(),
                            o3d3xx::TICKET_ID_SZ),
        std::bind(&o3d3xx::FrameGrabber::TicketHandler, this,
                  std::placeholders::_1, std::placeholders::_2, 0));

      return;
    }
  else
    {
      LOG(ERROR) << "Unexpected ticket: " << ticket;
      throw(std::logic_error("Unexpected ticket type: " + ticket));
    }
}

void
o3d3xx::FrameGrabber::Run()
{
  boost::asio::io_service::work work(this->io_service_);

  auto result_schema_write_handler =
    [&, this]
    (const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
      if (ec) { throw o3d3xx::error_t(ec.value()); }
      this->ticket_buffer_.clear();
      this->ticket_buffer_.resize(o3d3xx::TICKET_ID_SZ);

      this->sock_.async_read_some(
        boost::asio::buffer(this->ticket_buffer_.data(),
                            o3d3xx::TICKET_ID_SZ),
        std::bind(&o3d3xx::FrameGrabber::TicketHandler,
                  this, std::placeholders::_1,
                  std::placeholders::_2, 0));

      this->pcic_ready_.store(true);
    };

  //
  // Establish TCP connection to sensor and
  // set PCIC schema for this session
  //
  try
    {
      this->sock_.async_connect(
        this->endpoint_,
        [&, this] (const boost::system::error_code& ec)
        {
          if (ec) { throw o3d3xx::error_t(ec.value()); }
          boost::asio::async_write(
            this->sock_,
            boost::asio::buffer(this->schema_buffer_.data(),
                                this->schema_buffer_.size()),
            result_schema_write_handler);
        });

      this->io_service_.run();
    }
  catch (const std::exception& ex)
    {
      LOG(WARNING) << "Exception: " << ex.what();
    }

  LOG(INFO) << "Framegrabber thread done.";
}
