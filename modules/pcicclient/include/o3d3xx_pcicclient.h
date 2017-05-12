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
#ifndef __O3D3XX_PCIC_CLIENT_H__
#define __O3D3XX_PCIC_CLIENT_H__

#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include "o3d3xx_camera/camera.hpp"

namespace o3d3xx
{
  /**
   * The PCICClient is a class that, when given access to an
   * o3d3xx::Camera::Ptr, it provides unbuffered communication with
   * the PCIC interface.
   */
  class PCICClient
  {
  public:
    using Ptr = std::shared_ptr<PCICClient>;

    /**
     * Stores reference to the passed in camera and starts connect/receive
     * thread
     *
     * @param[in] cam The camera instance to grab frames from
     */
    PCICClient(o3d3xx::Camera::Ptr cam);

    /**
     * Cleans up any resources held by the receive thread object and
     * blocks until the operating system thread stops.
     */
    virtual ~PCICClient();

    // copy and move semantics
    PCICClient(PCICClient&&) = delete;
    PCICClient& operator=(PCICClient&&) = delete;
    PCICClient(PCICClient&) = delete;
    PCICClient& operator=(const PCICClient&) = delete;

    /**
     * Interrupts the running thread by throwing an o3d3xx::error_t with code
     * O3D3XX_THREAD_INTERRUPTED.
     *
     * While this is (currently) part of the public interface, clients should
     * be aware that there is really no way to restart a stopped PCICClient
     * instance. To do that, you would need to instantiate a new PCICClient.
     */
    void Stop();

    /**
     * Sends a PCIC command to the camera and returns the response
     * asynchronously through a callback.
     *
     * Note: Since the PCICClient is unbuffered, the calling thread
     * will be blocked until the request is completely sent. Also,
     * the receiving thread will be blocked until the response callback
     * returns.
     *
     * @param[in] request Vector containing the complete command
     * (without any header information, like ticket, length, etc.)
     *
     * @param[in] callback Function, called after receiving the response
     * from the camera, providing the complete response data by a vector
     * (without any header information, like ticket, length, etc.)
     */
    void Call(const std::vector<std::uint8_t>&& request,
	      std::function<void(std::vector<std::uint8_t>& response)> callback);

  private:

    /**
     * Commands consist of content data surrounded by some meta data.
     * The State provides information which buffer is currently
     * used in writing to and reading from network
     */
    enum class State { PRE_CONTENT, CONTENT, POST_CONTENT };

    /**
     * Connects to the camera
     */
    void DoConnect();

    /**
     * Handles DoConnect results
     */
    void ConnectHandler(const boost::system::error_code& ec);

    /**
     * Reads data from network into one of the three "in" buffers
     * depending on current reading state.
     */
    void DoRead(State state, int bytes_remaining = UNSET);

    /**
     * Handles DoRead results: Triggers further reads and in
     * case an incoming message is completely received, does
     * the callback (if existent).
     */
    void ReadHandler(State state, const boost::system::error_code& ec, std::size_t bytes_transferred, std::size_t bytes_remaining);

    /**
     * Returns buffer to be filled depending on specified reading state
     */
    std::vector<uint8_t>& ReadBufferByState(State state);

    /**
     * Writes data to network from one of the three "out" buffers
     * depending on current writing state.
     */
    void DoWrite(State state, int bytes_remaining = UNSET);

    /**
     * Handles DoWrite results: Triggers further writes and in
     * case a request is completely sent, unblocks calling thread.
     */
    void WriteHandler(State state, const boost::system::error_code& ec, std::size_t bytes_transferred, std::size_t bytes_remaining);

    /**
     * Returns buffer to be read from depending on specified state
     */
    std::vector<uint8_t>& WriteBufferByState(State state);

    /**
     * Finds and returns the next free ticket id
     */
    int NextTicketId();

  private:

    /**
     * Shared pointer to the camera this PCIC client will communicate with.
     */
    o3d3xx::Camera::Ptr cam_;

    /**
     * Cached copy of the camera IP address
     */
    std::string cam_ip_;

    /**
     * Cached copy of the camera PCIC TCP port
     */
    int cam_port_;

    /**
     * Flag indicating that client is connected.
     */
    std::atomic<bool> connected_;

    /**
     * Flag which is used as default parameter in DoRead and DoWrite
     * in order to indicate that a new buffer is used for read/write
     * so that the remaining size can be initialized by buffers size.
     */
    static const int UNSET = -1;

    /**
     * The ASIO event loop handle
     */
    boost::asio::io_service io_service_;
    
    /**
     * The ASIO socket to PCIC
     */
    boost::asio::ip::tcp::socket sock_;

    /**
     * The ASIO endpoint to PCIC
     */
    boost::asio::ip::tcp::endpoint endpoint_;

    /**
     * A pointer to the wrapped thread object. This is the thread that
     * communicates directly with the sensor.
     */
    std::unique_ptr<std::thread> thread_;
    
    /**
     * Maps PCIC tickets to callbacks. When receiving an incoming message,
     * the accordant callback can be found (and triggered).
     */
    std::map<int, std::function<void(std::vector<std::uint8_t>& content)>> pending_calls_;

    /**
     * Pre-content buffer for incoming messages (<ticket><length>\r\n<ticket>)
     */
    std::vector<std::uint8_t> in_pre_content_buffer_;

    /**
     * Content buffer for incoming messages, which is provided
     * through the callback to the caller
     */
    std::vector<std::uint8_t> in_content_buffer_;

    /**
     * Post-content buffer for incoming messages (\r\n)
     */
    std::vector<std::uint8_t> in_post_content_buffer_;
    
    /**
     * Pre-content buffer for outgoing requests (<ticket><length>\r\n<ticket>)
     */
    std::vector<std::uint8_t> out_pre_content_buffer_;

    /**
     * Content buffer for outgoing requests, which is provided
     * by the caller
     */
    std::vector<std::uint8_t> out_content_buffer_;

    /**
     * Post-content buffer for outgoing messages (\r\n)
     */
    std::vector<std::uint8_t> out_post_content_buffer_;

    /**
     * Flag that indicates whether an incoming messages is completely read.
     */
    bool out_completed_;

    /**
     * Ensures single outgoing message
     */
    std::mutex out_mutex_;

    /**
     * Condition variable used to unblock call
     */
    std::condition_variable out_cv_;

  }; // end: class PCICClient
  
} // end: namespace o3d3xx

#endif // __O3D3XX_PCIC_CLIENT_H__
