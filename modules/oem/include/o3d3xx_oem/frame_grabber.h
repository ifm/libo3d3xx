// -*- c++ -*-
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
#ifndef __O3D3XX_OEM_FRAME_GRABBER_H__
#define __O3D3XX_OEM_FRAME_GRABBER_H__

#include <cstdint>
#include <memory>
#include <o3d3xx_camera/camera.hpp>
#include <o3d3xx_framegrabber/pcic_schema.h>
#include <o3d3xx_oem/image_buffer.h>
#include <resultsync.hpp>

namespace o3d3xx
{
namespace oem
{
  class FrameGrabber
  {
  public:
    using Ptr = std::shared_ptr<FrameGrabber>;

    /**
     * Stores reference to the passed in camera and
     * instantiates the resultsync frame client.
     *
     * @param[in] cam The camera instance
     * @param[in] mask A bitmask encoding the result schema of interest
     *
     * We note that the camera instance is not strictly necessary. That is,
     * since this is the OEM framegrabber, it will be grabbing frames from the
     * local imager. However, we include the camera in this interface for two
     * reasons: 1) in the event we want to gain access to imager parameters (in
     * the future), 2) it is consistent with the TCP framegrabber interface
     * making it more friendly to templated functions in client code that may
     * want to swap between OEM and TCP framegrabbing.
     */
    FrameGrabber(o3d3xx::Camera::Ptr cam,
                 std::uint16_t mask = o3d3xx::DEFAULT_SCHEMA_MASK);

    virtual ~FrameGrabber() = default;
    FrameGrabber(FrameGrabber&&) = delete;
    FrameGrabber& operator=(FrameGrabber&&) = delete;
    FrameGrabber(FrameGrabber&) = delete;
    FrameGrabber& operator=(const FrameGrabber&) = delete;

    /**
     * This function is used to grab and parse out time synchronized image data
     * from the camera. It will call `Organize' implicitly on the passed in
     * `buff`, so the `buff' output parameter is assumed to be synchronized and
     * ready for analysis provided this function returns true.
     *
     * @param[out] buff A pointer to an o3d3xx::oem::ImageBuffer object to
     * update with the latest data from the camera.
     *
     * @param[in] timeout_millis Amount of time, in milliseconds, to wait for
     * new image data from the FrameGrabber. If `timeout_millis' is set to 0,
     * this function will block indefinitely.
     *
     * @return true if a new buffer was acquired within `timeout_millis', false
     * otherwise.
     */
    bool WaitForFrame(o3d3xx::oem::ImageBuffer* buff,
                      long timeout_millis = 0);

  protected:
    o3d3xx::Camera::Ptr cam_;
    std::uint16_t mask_;
    std::shared_ptr<ifm::resultsync::FrameClient> frame_client_;

  }; // end: class FrameGrabber

} // end: namespace oem
} // end: namespace o3d3xx

#endif // __O3D3XX_OEM_FRAME_GRABBER_H__
