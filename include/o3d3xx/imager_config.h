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
#ifndef __O3D3XX_IMAGER_CONFIG_H__
#define __O3D3XX_IMAGER_CONFIG_H__

#include <functional>
#include <memory>
#include <unordered_map>
#include <string>

namespace o3d3xx
{
  /**
   * Value object for holding imager configuration information. This class is
   * not thread safe.
   */
  class ImagerConfig
  {
  public:
    using Ptr = std::shared_ptr<ImagerConfig>;

    /**
     * Maps a parameter name as it is stored on the camera to a mutator
     * function that is used to set its value on an ImagerConfig instance.
     */
    static const
    std::unordered_map<std::string,
		       std::function<void(ImagerConfig*,
					  const std::string&)> >
    mutator_map;

    /**
     * Factory function for instantiating an ImagerConfig instnace from a JSON
     * string.
     *
     * @param[in] json A string of JSON encoding imager config parameters.
     */
    static o3d3xx::ImagerConfig::Ptr FromJSON(const std::string& json);

    /**
     * Initializes an imager configuration utilizing default values (per the
     * sensor documentation)
     */
    ImagerConfig();

    /**
     * Initializes an imager configuration based on a hash table mapping keys
     * to values.
     */
    ImagerConfig(const std::unordered_map<std::string, std::string>& params);

    /**
     * Serializes the current state of the imager config to JSON
     */
    std::string ToJSON() const;

    // Accessor / Mutators
    std::string Type() const noexcept;
    void SetType(const std::string& type) noexcept;

    std::string TypeHash() const noexcept;
    void SetTypeHash(const std::string& hash) noexcept;

    int ExposureTime() const;
    void SetExposureTime(int usecs);

    int Channel() const noexcept;
    void SetChannel(int channel) noexcept;

    double FrameRate() const noexcept;
    void SetFrameRate(double rate) noexcept;

    int ClippingLeft() const noexcept;
    void SetClippingLeft(int left) noexcept;

    int ClippingTop() const noexcept;
    void SetClippingTop(int top) noexcept;

    int ClippingRight() const noexcept;
    void SetClippingRight(int right) noexcept;

    int ClippingBottom() const noexcept;
    void SetClippingBottom(int bottom) noexcept;

    bool ReduceMotionArtifacts() const noexcept;
    void SetReduceMotionArtifacts(bool on) noexcept;

    int SpatialFilterType() const noexcept;
    void SetSpatialFilterType(int type) noexcept;

    int AverageFilterNumPictures() const noexcept;
    void SetAverageFilterNumPictures(int num) noexcept;

  protected:
    /** Type of imager */
    std::string type_;

    /** Unique hash for imager type */
    std::string type_hash_;

    /**
     * For the low dynamic range imager: this is the exposure time
     *
     * For the moderate dynamic range imager: this is the long exposure
     * time. The short exposure time is derived from the long by the sensor
     * (the short-to-long ratio is 40:1).
     *
     * For the high dynamic range imager: a triple exposure time is used and it
     * is not possible to change it.
     *
     * Exposure time is expressing in microsecs.
     */
    int exposure_time_;

    /** channel */
    int channel_;

    /** Frame rate of the sensor */
    double frame_rate_;

    /** Clipping-area lower value in width dimension */
    int clipping_left_;

    /** Clipping-area lower value in height dimension */
    int clipping_top_;

    /** Clipping-area upper value in width dimension */
    int clipping_right_;

    /** Clipping-area upper value in height dimension */
    int clipping_bottom_;

    /** Enables filtering for motion artifacts */
    bool reduce_motion_artifacts_;

    /** Applies a spatial filter */
    int spatial_filter_type_;

    /** Temporal filter, number of images to average */
    int average_filter_num_pictures_;

  }; // end: class ImagerConfig

} // end: namespace o3d3xx


#endif // __O3D3XX_IMAGER_CONFIG_H__
