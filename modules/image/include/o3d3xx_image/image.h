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
#ifndef __O3D3XX_IMAGE_H__
#define __O3D3XX_IMAGE_H__

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "o3d3xx_framegrabber/byte_buffer.hpp"

namespace o3d3xx
{
  using PointT = pcl::PointXYZI;

  /**
   * The ImageBuffer class is used to hold time synchronized images from the
   * camera. That is, to hold image data from a single buffer read off the
   * wire but organized into its component parts.
   *
   * NOTE: The ImageBuffer class is NOT thread safe!
   */
  class ImageBuffer : public o3d3xx::ByteBuffer
  {
  public:
    using Ptr = std::shared_ptr<ImageBuffer>;

    /**
     * Allocates space for the individual component images
     */
    ImageBuffer();

    /**
     * RAII deallocations
     */
    virtual ~ImageBuffer();

    // disable move semantics (for now)
    ImageBuffer(ImageBuffer&&) = delete;
    ImageBuffer& operator=(ImageBuffer&&) = delete;

    // copy ctor/assignment operator
    ImageBuffer(const ImageBuffer& src_buff);
    ImageBuffer& operator=(const ImageBuffer& src_buff);

    /**
     * Returns the wrapped depth image. This does NOT make a copy of the data.
     *
     * NOTE: Since the depth image is a cv::Mat the correct thing to do here is
     * to return by value as cv::Mat does it's own memory management /
     * reference counting.
     */
    cv::Mat DepthImage();

    /**
     * Returns the wrapped unit vector image. This does NOT make a copy of the
     * data.
     *
     * The unit vector matrix is a 3-channel matrix where for each pixel in the
     * RadialDistanceImage, there exists the vector e_x, e_y, e_z (the three
     * channels of this matrix).
     */
    cv::Mat UnitVectors();

    /**
     * Returns the wrapped amplitude image. This does NOT make a copy of the
     * data.
     *
     * It should be noted that this is the normalized (wrt, exposure time)
     * amplitude image.
     *
     * NOTE: Since the amplitude image is a cv::Mat the correct thing to do
     * here is to return by value as cv::Mat does it's own memory management /
     * reference counting.
     */
    cv::Mat AmplitudeImage();

    /**
     * Returns the wrapped (raw) amplitude image. This does NOT make a copy of
     * the data.
     *
     * It should be noted that this is the raw amplitude image. Per the IFM
     * docs: "In double exposure mode, the lack of normalization my lead
     * (depending on the chosen exposure times) to inhomogeneous amplitude
     * impression, if a certain pixel is taken from the short exposure time and
     * some of its neighbors are not."
     *
     * NOTE: Since the amplitude image is a cv::Mat the correct thing to do
     * here is to return by value as cv::Mat does it's own memory management /
     * reference counting.
     */
    cv::Mat RawAmplitudeImage();

    /**
     * Returns the wrapped confidence image. This does NOT make a copy of the
     * data.
     *
     * NOTE: Since the confidence image is a cv::Mat the correct thing to do
     * here is to return by value as cv::Mat does it's own memory management /
     * reference counting.
     */
    cv::Mat ConfidenceImage();

    /**
     * Returns the wrapped xyz_image.
     *
     * The xyz_image is a 3-channel OpenCV image encoding of the point cloud
     * where the three channels are spatial planes (x, y, z) as opposed to
     * color planes. It should be noted that while this encoding of the point
     * cloud contains the same data as the PCL encoded point cloud (for the
     * cartesian components) the data are kept in mm (as opposed to meters) and
     * the data types are int16_t as opposed to float. However, the coord frame
     * for this point cloud data is consistent with the PCL point cloud.
     */
    cv::Mat XYZImage();

    /**
     * Returns the shared pointer to the wrapped point cloud
     */
    pcl::PointCloud<o3d3xx::PointT>::Ptr Cloud();

    /**
     * Returns a 6-element vector containing the extrinsic calibration
     * data. NOTE: This is the extrinsics WRT the IFM optical frame.
     *
     * The elements are:
     * tx, ty, tz, rot_x, rot_y, rot_z
     *
     * The translation units are mm
     * The rotation units are degrees
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
     * sure your current pcic schema asks for them.
     */
    std::vector<std::uint32_t> ExposureTimes();

    /**
     * Returns the temperature of the illumination unit.
     *
     * NOTE: To get the temperature of the illumination unit to the frame, you
     * need to make sure your current pcic schema asks for it.
     */
    float IlluTemp();

    /**
     * Synchronizes the parsed out image data with the internally wrapped byte
     * buffer.
     */
    virtual void Organize();

  private:
    /**
     * The extrinsic calibration data
     */
    std::vector<float> extrinsics_;

    /**
     * The exposure time data
     */
    std::vector<std::uint32_t> exposure_times_;

    /**
     * The temperature of the illumination unit
     */
    float illu_temp_;

    /**
     * Point cloud used to hold the cartesian xyz and amplitude data (intensity
     * channel).
     *
     * NOTE: Data in the point cloud are converted to meters and utilize a
     * right-handed coordinate frame.
     */
    pcl::PointCloud<o3d3xx::PointT>::Ptr cloud_;

    /**
     * OpenCV image encoding of the radial image data
     *
     * NOTE: Unlike the point cloud, the data in the depth image remain in
     * millimeters.
     */
    cv::Mat depth_;

    /**
     * OpenCV image encoding of the unit vectors
     */
    cv::Mat uvec_;

    /**
     * OpenCV image encoding of the normalized amplitude data
     */
    cv::Mat amp_;

    /**
     * OpenCV image encoding of the raw amplitude data
     */
    cv::Mat raw_amp_;

    /**
     * OpenCV image encoding of the confidence data
     */
    cv::Mat conf_;

    /**
     * OpenCV image encoding of the point cloud
     */
    cv::Mat xyz_image_;

  }; // end: class ImageBuffer

} // end: namespace o3d3xx

#endif // __O3D3XX_IMAGE_H__
