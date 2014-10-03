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
#ifndef __O3D3XX_FRAME_GRABBER_H__
#define __O3D3XX_FRAME_GRABBER_H__

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "o3d3xx/image.h"
#include "o3d3xx/camera.hpp"

namespace o3d3xx
{
  /**
   * The FrameGrabber is a class that, when given access to an
   * o3d3xx::Camera::Ptr, it will grab frames from the camera in a separate
   * thread of execution.
   */
  class FrameGrabber
  {
  public:
    using Ptr = std::shared_ptr<FrameGrabber>;
    using ReadHandler =
      std::function<void(const boost::system::error_code&, std::size_t)>;

    /**
     * Stores reference to the passed in camera and starts the worker thread
     *
     * @param[in] cam The camera instance to grab frames from
     */
    FrameGrabber(o3d3xx::Camera::Ptr cam);

    /**
     * Cleans up any resources held by the frame grabbing thread object and
     * blocks until the operating system thread stops.
     */
    virtual ~FrameGrabber();

    // copy and move semantics
    FrameGrabber(FrameGrabber&&) = delete;
    FrameGrabber& operator=(FrameGrabber&&) = delete;
    FrameGrabber(FrameGrabber&) = delete;
    FrameGrabber& operator=(const FrameGrabber&) = delete;

    /**
     * Interrupts the running thread by throwing an o3d3xx::error_t with code
     * O3D3XX_THREAD_INTERRUPTED.
     *
     * While this is (currently) part of the public interface, clients should
     * be aware that there is really no way to restart a stopped FrameGrabber
     * instance. To do that, you would need to instantiate a new FrameGrabber.
     */
    void Stop();

    /**
     * This function is used by clients for retrieving point clouds from the
     * FrameGrabber's wrapped camera instance. This function will block for up
     * to `timeout_millis' milliseconds or possibly indefinitely if
     * `timeout_millis' is set to 0.
     *
     * @param[out] cloud A (smart) pointer to a point cloud to fill will data
     * from the last image frame captured off the camera.
     *
     * @param[in] timeout_millis Amount of time, in milliseconds, to wait for
     * new image data from the FrameGrabber. If `timeout_millis' is set to 0,
     * this function will block indefinitely.
     *
     * @return true if a point cloud was constructed from a new camera buffer
     * acquired within `timeout_millis', false otherwise.
     *
     * NOTE: The `intensity' channel of the point cloud will be filled with
     * data from the sensor's `Amplitude' image.
     */
    bool WaitForCloud(pcl::PointCloud<o3d3xx::PointT>::Ptr& cloud,
		      long timeout_millis = 0);

    /**
     * This function is used by clients for retrieving image frames from the
     * FrameGrabber's wrapped Camera instance. This function will block for up
     * to `timeout_millis' milliseconds or possibly indefintely if
     * `timeout_millis' is set to 0.
     *
     * NOTE: This is a "low-level" function used for getting the raw image
     * buffers off the camera. Generally, clients will prefer to use
     * WaitForCloud(...) or similar.
     *
     * @param[out] client_buff Buffer to be filled with image data. The buffer
     * will be resized as appropriate to accomodate the data.
     *
     * @param[in] timeout_millis Amount of time, in milliseconds, to wait for
     * new image data from the FrameGrabber. If `timeout_millis' is set to 0,
     * this function will block indefinitely.
     *
     * @return true if a new buffer was acquired within `timeout_millis', false
     * otherwise.
     *
     * @see o3d3xx::FrameGrabber::WaitForCloud
     */
    bool WaitForFrame(std::vector<std::uint8_t>& client_buff,
		      long timeout_millis = 0);

  protected:
    /**
     * This is the thread worker function -- its execution occurs in a separate
     * operating system thread. This thread will run its own ASIO event loop
     * continually streaming in images from the camera independent of other
     * control threads within the driver.
     */
    virtual void Run();

  private:
    /**
     * Shared pointer to the camera this frame grabber will grab frames from.
     */
    o3d3xx::Camera::Ptr cam_;

    /**
     * The ASIO event loop handle
     */
    boost::asio::io_service io_service_;

    /**
     * A pointer to the wrapped thread object. This is the thread that
     * communicates directly with the sensor.
     */
    std::unique_ptr<std::thread> thread_;

    /**
     * Holds the raw 'Ticket' bytes received from the sensor.
     *
     * PCIC V3 (16 bytes total):
     * 4 bytes ticket number
     * 'L' + 9 bytes length + CR + LF
     *
     * The '9 bytes length' is a string of chars to be converted to an integer
     * number representing the length of the payload data.
     */
    std::vector<std::uint8_t> ticket_buffer_;

    /**
     * The FrameGrabber double buffers images when acquiring from the
     * camera. This implements the _back buffer_ in the double buffering
     * scheme.
     *
     * According to the PCIC V3 protocol, this will hold from the four digit
     * ticket sequence up to and including the ending CR + LF ('\r\n').
     */
    std::vector<std::uint8_t> back_buffer_;

    /**
     * The buffer that subscribed clients fetch data from.
     */
    std::vector<std::uint8_t> front_buffer_;

    /**
     * Protects front_buffer_
     */
    std::mutex front_buffer_mutex_;

    /**
     * Condition variable used to notify clients when image data are ready
     */
    std::condition_variable front_buffer_cv_;

  }; // end: class FrameGrabber

} // end: namespace o3d3xx

#endif // __O3D3XX_FRAME_GRABBER_H__
