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
#ifndef __O3D3XX_OEM_IMAGE_BUFFER_H__
#define __O3D3XX_OEM_IMAGE_BUFFER_H__

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <o3d3xx_framegrabber/pcic_schema.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <resultsync.hpp>

namespace o3d3xx
{
namespace oem
{
  using PointT = pcl::PointXYZI;

  /**
   * The ImageBuffer class is used to hold time synchronized images from the
   * camera, organized into OpenCV and PCL data structures.
   *
   * This interface mimics that of the `o3d3xx::ImageBuffer` class without
   * being tied to the `o3d3xx::ByteBuffer` inheritance hierarchy. The reason
   * for simply mimicing the interface is two-fold: 1) looser coupling, 2) it
   * would have been a _forced_ interface as the idoms of acquiring data from
   * the zero-copy framegrabber vs. the TCP framegrabber are quite different.
   */
  class ImageBuffer
  {
  public:
    using Ptr = std::shared_ptr<ImageBuffer>;

    /**
     * Allocates space for the individual component images.
     */
    ImageBuffer();

    virtual ~ImageBuffer() = default;

    // non-copyable, non-movable
    ImageBuffer(ImageBuffer&&) = delete;
    ImageBuffer& operator=(ImageBuffer&&) = delete;
    ImageBuffer(const ImageBuffer& src_buff) = delete;
    ImageBuffer& operator=(const ImageBuffer& src_buff) = delete;

    /**
     * Returns the wrapped radial distance image (called "DepthImage" here for
     * compatibility with `o3d3xx::ImageBuffer`).
     *
     * M x N x 1, uint16_t
     */
    cv::Mat DepthImage();

    /**
     * Returns the wrapped unit vector image
     *
     * M x N x 3, float32
     * For each pixel in the radial distance image, there exists the vector
     * e_x, e_y, e_z (the three channels of this matrix).
     */
    cv::Mat UnitVectors();

    /**
     * Returns the normalized (wrt exposure time) amplitude image
     *
     * M x N x 1, uint16_t
     */
    cv::Mat AmplitudeImage();

    /**
     * Returns the raw amplitude image
     *
     * M x N x 1, uint16_t
     */
    cv::Mat RawAmplitudeImage();

    /**
     * Returns the wrapped confidence image.
     *
     * M x N x 1, uint8_t
     */
    cv::Mat ConfidenceImage();

    /**
     * Returns the wrapped Cartesian data as a multi-channel opencv image.
     *
     * The xyz_image is a 3-channel OpenCV image encoding of the point cloud
     * where the three channels are spatial planes (x, y, z) as opposed to
     * color planes. It should be noted that while this encoding of the point
     * cloud contains the same data as the PCL encoded point cloud (for the
     * cartesian components) the data are kept in mm (as opposed to meters) and
     * the data types are int16_t as opposed to float. However, the coord frame
     * for this point cloud data is consistent with the PCL point cloud.
     *
     * M x N x 3, int16_t
     */
    cv::Mat XYZImage();

    /**
     * Returns the shared pointer to the wrapped point cloud
     */
    pcl::PointCloud<o3d3xx::oem::PointT>::Ptr Cloud();

    /**
     * Returns the 6-element vector containing the extrinsic calibration. NOTE:
     * This is the extrinsics WRT the IFM optical frame and NOT the `libo3d3xx`
     * coordinate frame (i.e., the encoding of the point cloud).
     */
    std::vector<float> Extrinsics();

    /**
     * Returns a 3-element vector containing the exposure times (usec) for the
     * current frame. Unused exposure times are reported as 0.
     *
     * If all elements are reported as 0 either the exposure times are not
     * configured to be returned back in the data stream from the camera or an
     * error in parsing them has occured.
     *
     * NOTE: To get the exposure data registered to the frame, you need to make
     * sure your current schema asks for them.
     */
    std::vector<std::uint32_t> ExposureTimes();

    /**
     * Organizes the passed in frame data into the internally wrapped data
     * structures according to the desired results as specified by the mask.
     *
     * @param[in] frame A pointer to a resultsync frame
     * @param[in] mask  A schema mask encoding the results of interest.
     *                  This is the same schema encoding as the TCP-based PCIC
     *                  framegrabber.
     */
    virtual void Organize(ifm::resultsync::Frame* frame,
                          std::uint16_t mask = o3d3xx::DEFAULT_SCHEMA_MASK);

  protected:
    std::vector<float> extrinsics_;
    std::vector<std::uint32_t> exposure_times_;
    pcl::PointCloud<o3d3xx::oem::PointT>::Ptr cloud_;
    cv::Mat depth_;
    cv::Mat uvec_;
    cv::Mat amp_;
    cv::Mat raw_amp_;
    cv::Mat conf_;
    cv::Mat xyz_image_;

  }; // end: class ImageBuffer

} // end: namespace oem
} // end: namespace o3d3xx

#endif // __O3D3XX_OEM_IMAGE_BUFFER_H__
