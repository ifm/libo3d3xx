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

namespace o3d3xx
{
  using PointT = pcl::PointXYZI;
  extern const std::size_t IMG_TICKET_SZ; // bytes

  enum class pixel_format : std::uint8_t
  {
    FORMAT_8U = 0,
    FORMAT_8S = 1,
    FORMAT_16U = 2,
    FORMAT_16S = 3,
    FORMAT_32U = 4,
    FORMAT_32S = 5,
    FORMAT_32F = 6,
    FORMAT_64U = 7,
    FORMAT_64F = 8,
    FORMAT_16U2 = 9,
    FORMAT_32F3 = 10
  };

  enum class image_chunk : std::uint32_t
  {
    USERDATA = 0,
    RADIAL_DISTANCE = 100,
    AMPLITUDE = 101, // normalized amplitude
    RAW_AMPLITUDE = 103,
    CARTESIAN_X = 200,
    CARTESIAN_Y = 201,
    CARTESIAN_Z = 202,
    CARTESIAN_ALL = 203,
    UNIT_VECTOR_E1 = 220,
    UNIT_VECTOR_E2 = 221,
    UNIT_VECTOR_E3 = 222,
    UNIT_VECTOR_ALL = 223,
    CONFIDENCE = 300,
    DIAGNOSTIC_DATA = 302,
    EXTRINSIC_CALIBRATION = 400,
    JSON_MODEL = 500,
    SNAPSHOT_IMAGE = 600
  };

  /**
   * The ImageBuffer class is used to hold time synchronized images from the
   * camera. That is, to hold image data from a single buffer read off the
   * wire but organized into its component parts.
   *
   * NOTE: The ImageBuffer class is NOT thread safe!
   */
  class ImageBuffer
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
     * Returns the wrapped amplitude image. This does NOT make a copy of the
     * data.
     *
     * NOTE: Since the amplitude image is a cv::Mat the correct thing to do
     * here is to return by value as cv::Mat does it's own memory management /
     * reference counting.
     */
    cv::Mat AmplitudeImage();

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
     * Returns (a copy of) the underlying byte buffer read from the camera.
     */
    std::vector<std::uint8_t> Bytes();

    /**
     * Returns the state of the `dirty' flag
     */
    bool Dirty() const noexcept;

    /**
     * Synchronizes the parsed out image data with the internally wrapped byte
     * buffer. This call has been separated out from `SetBytes' in order to
     * minimize mutex contention with the frame grabbers main thread.
     */
    void Organize();

    /**
     * Sets the data from the passed in `buff' to the internally wrapped byte
     * buffer. This function assumes the passed in `buff' is a valid byte
     * buffer from the camera.
     *
     * By default (see `copy' parameter below) this function will take in
     * `buff' and `swap' contents with its internal buffer so that the
     * operation is O(1). If you want copy behavior specify the copy flag and
     * complexity will be linear in the size of the byte buffer.
     *
     * @see o3d3xx::verify_image_buffer
     *
     * @param[in] buff Raw data bytes to copy to internal buffers
     *
     * @param[in] copy If true the data are copied from `buff' to the
     * internally wrapped buffer and `buff' will remain unchanged.
     */
    void SetBytes(std::vector<std::uint8_t>& buff, bool copy = false);

  private:
    /**
     * Flag used to indicate if the wrapped byte buffer and the individual
     * component images are in sync.
     */
    bool dirty_;

    /**
     * Raw bytes read off the wire from the camera
     */
    std::vector<std::uint8_t> bytes_;

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
     * NOTE: Unlike the point cloude, the data in the depth image remain in
     * millimeters.
     */
    cv::Mat depth_;

    /**
     * OpenCV image encoding of the amplitude data
     */
    cv::Mat amp_;

    /**
     * OpenCV image encoding of the confidence data
     */
    cv::Mat conf_;

    /**
     * OpenCV image encoding of the point cloud
     */
    cv::Mat xyz_image_;

    /**
     * Mutates the `dirty' flag
     */
    void _SetDirty(bool flg) noexcept;

  }; // end: class ImageBuffer

  /**
   * Verifies that the passed in buffer contains a valid "ticket" from the
   * sensor.
   *
   * @return true if the ticket buffer is valid
   */
  bool verify_ticket_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Verifies that the passed in image buffer is valid.
   *
   * @return true if the image buffer is valid
   */
  bool verify_image_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Extracts the image buffer size from a ticket buffer received from the
   * sensor.
   *
   * NOTE: The size of the passed in buffer is not checked here. It is assumed
   * that this buffer has already passed the `verify_ticket_buffer' check. An
   * exception may be thrown if an inappropriately sized buffer is passed in to
   * this function.
   *
   * @param[in] buff A ticket buffer
   *
   * @return The expected size of the image buffer.
   */
  std::size_t get_image_buffer_size(const std::vector<std::uint8_t>& buff);

  /**
   * Finds the index into the image buffer of where the chunk of `chunk_type'
   * begins.
   *
   * @param[in] buff The image buffer to search
   * @param[in] chunk_type The type of chunk to look for
   *
   * @return The index into the buffer of where the chunk begins.
   * @throw o3d3xx::error_t If the chunk is not found
   */
  std::size_t get_chunk_index(const std::vector<std::uint8_t>& buff,
                              o3d3xx::image_chunk chunk_type);

  /**
   * Returns the number of bytes contained in the passed in pixel format.
   */
  std::size_t get_num_bytes_in_pixel_format(o3d3xx::pixel_format f);

} // end: namespace o3d3xx

#endif // __O3D3XX_IMAGE_H__
