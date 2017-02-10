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

#include <o3d3xx_oem/frame_grabber.h>
#include <cstdint>
#include <memory>
#include <o3d3xx_camera/camera.hpp>
#include <o3d3xx_oem/image_buffer.h>
#include <glog/logging.h>
#include <resultsync.hpp>

o3d3xx::oem::FrameGrabber::FrameGrabber(o3d3xx::Camera::Ptr cam,
                                        std::uint16_t mask)
  : cam_(cam),
    mask_(mask),
    frame_client_(std::make_shared<ifm::resultsync::FrameClient>())
{ }

bool
o3d3xx::oem::FrameGrabber::WaitForFrame(o3d3xx::oem::ImageBuffer* buff,
                                        long timeout_millis)
{
  ifm::resultsync::Frame* frame =
    this->frame_client_->waitForFrame(timeout_millis);

  if (frame == nullptr)
    {
      LOG(WARNING) << "Timeout waiting for frame from resultsync!";
      return false;
    }

  buff->Organize(frame, this->mask_);

  delete frame;
  return true;
}
