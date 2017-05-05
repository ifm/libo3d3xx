// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
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

#include "o3d3xx_pcicclient.h"

#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <glog/logging.h>
#include "o3d3xx_camera/camera.hpp"

o3d3xx::PCICClient::PCICClient(o3d3xx::Camera::Ptr cam)
  : cam_(cam),
    connected_(false),
    io_service_(),
    sock_(io_service_),
    in_pre_content_buffer_(20),
    in_content_buffer_(),
    in_post_content_buffer_(2),
    out_pre_content_buffer_(20),
    out_content_buffer_(),
    out_post_content_buffer_({'\r','\n'})
{
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
     new std::thread(std::bind(&o3d3xx::PCICClient::DoConnect, this)));
}

o3d3xx::PCICClient::~PCICClient()
{
  DLOG(INFO) << "PCICClient dtor running...";

  if(this->thread_ && this->thread_->joinable())
    {
      // NOTE: If Stop() was already called, that is fine
      // because the ASIO event loop is already done so the posted exception
      // will never get emitted.
      this->Stop();
      this->thread_->join();
    }

  DLOG(INFO) << "PCICClient done.";
}

void
o3d3xx::PCICClient::Stop()
{
  this->io_service_.post([]() {
    throw o3d3xx::error_t(O3D3XX_THREAD_INTERRUPTED); });
}

void
o3d3xx::PCICClient::Call(const std::vector<std::uint8_t>&& request,
			 std::function<void(std::vector<std::uint8_t>& response)> callback)
{
  // TODO Better solution for this connection waiting ..
  int i = 0;
  while (! this->connected_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      i++;
      
      if (i > 2000)
        {
          LOG(WARNING) << "connected_ flag not set!";
          return;
        }
    }

  // PCICClient is unbuffered, so block further calls
  std::unique_lock<std::mutex> lock(this->out_mutex_);

  this->out_completed_ = false;

  int ticket_id = this->NextTicketId();

  // Add callback to pending callbacks
  this->pending_calls_[ticket_id] = callback;

  // Transform ticket and length to string
  std::ostringstream pre_content_ss;
  pre_content_ss << ticket_id << 'L' << std::setw(9) << std::setfill('0')
		 << (request.size()+6) << "\r\n" << ticket_id;
  std::string pre_content_str = pre_content_ss.str();

  // Fill buffers
  std::copy(pre_content_str.begin(), pre_content_str.end(),
	    this->out_pre_content_buffer_.begin());
  this->out_content_buffer_ = std::move(request);

  DLOG(INFO) << "Client sending request";

  // Send command
  this->DoWrite(State::PRE_CONTENT);

  // Wait until sending is complete
  while(!this->out_completed_)
    {
      this->out_cv_.wait(lock);
    }
  lock.unlock();
}

void
o3d3xx::PCICClient::DoConnect()
{
  boost::asio::io_service::work work(this->io_service_);

  // Establish TCP connection to sensor
  try
    {
      this->sock_.async_connect(this->endpoint_,
				std::bind(&o3d3xx::PCICClient::ConnectHandler,
					  this, std::placeholders::_1));
      this->io_service_.run();
    }
  catch(const std::exception& ex)
    {
        LOG(WARNING) << "Exception: " << ex.what();
    }

  LOG(INFO) << "PCICClient thread done.";
}

void
o3d3xx::PCICClient::ConnectHandler(const boost::system::error_code& ec)
{
  if(ec) { throw o3d3xx::error_t(ec.value()); }
  this->connected_.store(true);
  this->DoRead(State::PRE_CONTENT);
}

void
o3d3xx::PCICClient::DoRead(State state, int bytes_remaining)
{
  std::vector<std::uint8_t> &buffer = this->ReadBufferByState(state);
  if(bytes_remaining==UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->sock_.async_read_some(
			      boost::asio::buffer(&buffer[buffer.size()-bytes_remaining], 
						  bytes_remaining), 
			      std::bind(&o3d3xx::PCICClient::ReadHandler, 
					this,
					state,
					std::placeholders::_1, 
					std::placeholders::_2,
					bytes_remaining));
}

void
o3d3xx::PCICClient::ReadHandler(State state, const boost::system::error_code& ec,
				std::size_t bytes_transferred, std::size_t bytes_remaining)
{
  if(ec) { throw o3d3xx::error_t(ec.value()); }
  
  if(bytes_remaining - bytes_transferred > 0)
    {
      this->DoRead(state, bytes_remaining - bytes_transferred);
    }
  else
    {
      std::string ticket_str;
      int ticket;
      std::string length_str;
      int length;
      switch(state)
	{
	case PRE_CONTENT:
	  length_str.assign(this->in_pre_content_buffer_.begin()+5,
			    this->in_pre_content_buffer_.begin()+14);
	  length = std::stoi(length_str);
	  this->in_content_buffer_.resize(length-6);
	  this->DoRead(State::CONTENT);
	  break;

	case CONTENT:
	  this->DoRead(State::POST_CONTENT);
	  break;

	case POST_CONTENT:
	  ticket_str.assign(this->in_pre_content_buffer_.begin(),
			    this->in_pre_content_buffer_.begin()+4);
	  ticket = std::stoi(ticket_str);
	  this->out_mutex_.lock();
	  if(this->pending_calls_.find(ticket)!=pending_calls_.end())
	    {
	      this->pending_calls_[ticket](this->in_content_buffer_);
	      this->pending_calls_.erase(ticket);
	    }
	  this->out_mutex_.unlock();
	  this->DoRead(State::PRE_CONTENT);
	  break;
	}
    }


}

std::vector<std::uint8_t>&
o3d3xx::PCICClient::ReadBufferByState(State state)
{
  switch(state)
    {
    case PRE_CONTENT: return this->in_pre_content_buffer_;
    case CONTENT: return this->in_content_buffer_;
    case POST_CONTENT: return this->in_post_content_buffer_;
    }
}

void
o3d3xx::PCICClient::DoWrite(State state, int bytes_remaining)
{
  std::vector<std::uint8_t> &buffer = this->WriteBufferByState(state);
  if(bytes_remaining==UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->sock_.async_write_some(
			      boost::asio::buffer(&buffer[buffer.size()-bytes_remaining], 
						  bytes_remaining), 
			      std::bind(&o3d3xx::PCICClient::WriteHandler, 
					this,
					state,
					std::placeholders::_1, 
					std::placeholders::_2,
					bytes_remaining));
}

void
o3d3xx::PCICClient::WriteHandler(State state, const boost::system::error_code& ec,
				 std::size_t bytes_transferred, std::size_t bytes_remaining)
{
  if(ec) { throw o3d3xx::error_t(ec.value()); }
  
  if(bytes_remaining - bytes_transferred > 0)
    {
      this->DoWrite(state, bytes_remaining - bytes_transferred);
    }
  else
    {
      switch(state)
	{
	case PRE_CONTENT:
	  this->DoWrite(State::CONTENT);
	  break;

	case CONTENT:
	  this->DoWrite(State::POST_CONTENT);
	  break;

	case POST_CONTENT:
	  this->out_completed_ = true;
	  this->out_cv_.notify_all();
	  break;
	}
    }
}

std::vector<std::uint8_t>&
o3d3xx::PCICClient::WriteBufferByState(State state)
{
  switch(state)
    {
    case PRE_CONTENT: return this->out_pre_content_buffer_;
    case CONTENT: return this->out_content_buffer_;
    case POST_CONTENT: return this->out_post_content_buffer_;
    }
}

int
o3d3xx::PCICClient::NextTicketId()
{
  int ticket_id = 1000;
  if(!this->pending_calls_.empty())
    {
      ticket_id = (this->pending_calls_.rbegin()->first)-1000;
      while(this->pending_calls_.find(((++ticket_id)%9000) + 1000)
	    != this->pending_calls_.end());
      ticket_id = (ticket_id%9000) + 1000;
    }
  return ticket_id;
}
