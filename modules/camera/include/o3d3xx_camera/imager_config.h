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
     *
     * @param[in] im_ptr A shared pointer to an ImagerConfig to bootstrap
     *                   the default values from.
     *
     * @return A shared pointer to an ImagerConfig instance based on
     *         the passed in JSON encoding.
     */
    static o3d3xx::ImagerConfig::Ptr
    FromJSON(const std::string& json,
             o3d3xx::ImagerConfig::Ptr im_ptr = nullptr);

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
    int Channel() const noexcept;
    void SetChannel(int channel) noexcept;

    int ClippingBottom() const noexcept;
    void SetClippingBottom(int bottom) noexcept;

    int ClippingLeft() const noexcept;
    void SetClippingLeft(int left) noexcept;

    int ClippingRight() const noexcept;
    void SetClippingRight(int right) noexcept;

    int ClippingTop() const noexcept;
    void SetClippingTop(int top) noexcept;

    bool ContinuousAutoExposure() const noexcept;
    void SetContinuousAutoExposure(bool on) noexcept;

    bool EnableAmplitudeCorrection() const noexcept;
    void SetEnableAmplitudeCorrection(bool enable) noexcept;

    bool EnableFastFrequency() const noexcept;
    void SetEnableFastFrequency(bool enable) noexcept;

    bool EnableFilterAmplitudeImage() const noexcept;
    void SetEnableFilterAmplitudeImage(bool enable) noexcept;

    bool EnableFilterDistanceImage() const noexcept;
    void SetEnableFilterDistanceImage(bool enable) noexcept;

    bool EnableRectificationAmplitudeImage() const noexcept;
    void SetEnableRectificationAmplitudeImage(bool enable) noexcept;

    bool EnableRectificationDistanceImage() const noexcept;
    void SetEnableRectificationDistanceImage(bool enable) noexcept;

    int ExposureTime() const;
    void SetExposureTime(int usecs);

    std::string ExposureTimeList() const noexcept;
    void SetExposureTimeList(const std::string& s) noexcept;

    int ExposureTimeRatio() const;
    void SetExposureTimeRatio(int ratio);

    double FrameRate() const noexcept;
    void SetFrameRate(double rate) noexcept;

    int MinimumAmplitude() const noexcept;
    void SetMinimumAmplitude(int min) noexcept;

    int Resolution() const noexcept;
    void SetResolution(int res) noexcept;

    std::string ClippingCuboid() const noexcept;
    void SetClippingCuboid(const std::string& s) noexcept;

    int SpatialFilterType() const noexcept;
    void SetSpatialFilterType(int filt) noexcept;

    //! @todo: This data type may change. It is not clear that this is even
    //! implemented in the camera yet. I believe it has to do with flagging a
    //! pixel as bad due to motion artifacts across exposure times ... but it
    //! does not appear to be documented.
    int SymmetryThreshold() const noexcept;
    void SetSymmetryThreshold(int thresh) noexcept;

    int TemporalFilterType() const noexcept;
    void SetTemporalFilterType(int filt) noexcept;

    int ThreeFreqMax2FLineDistPercentage() const noexcept;
    void SetThreeFreqMax2FLineDistPercentage(int percentage) noexcept;

    int ThreeFreqMax3FLineDistPercentage() const noexcept;
    void SetThreeFreqMax3FLineDistPercentage(int percentage) noexcept;

    int TwoFreqMaxLineDistPercentage() const noexcept;
    void SetTwoFreqMaxLineDistPercentage(int percentage) noexcept;

    std::string Type() const noexcept;
    void SetType(const std::string& type) noexcept;

    double MaxAllowedLEDFrameRate() const noexcept;
    void SetMaxAllowedLEDFrameRate(double rate) noexcept;

  protected:
    /** channel */
    int channel_;

    /** Clipping-area upper value in height dimension */
    int clipping_bottom_;

    /** Clipping-area lower value in width dimension */
    int clipping_left_;

    /** Clipping-area upper value in width dimension */
    int clipping_right_;

    /** Clipping-area lower value in height dimension */
    int clipping_top_;

    /** Undocumented: Flag to turn on/off continuous auto-exposure */
    bool continuous_auto_exposure_;

    /** Undocumented: Flag to turn on/off amplitude correction */
    bool enable_amplitude_correction_;

    /** Gets higher framerates for longer range (>5m) imagers */
    bool enable_fast_frequency_;

    /** Undocumented: Flag to turn on/off aplitude image filter */
    bool enable_filter_amplitude_image_;

    /** Undocumented: Flag to turn on/off distance image filter */
    bool enable_filter_distance_image_;

    /** Undocumented: Flag to turn on/off amplitude image rectification */
    bool enable_rectification_amplitude_image_;

    /** Undocumented: Flag to turn on/off distance image rectification */
    bool enable_rectification_distance_image_;

    /**
     * For the low dynamic range imager: this is the exposure time
     *
     * For the moderate dynamic range imager: this is the long exposure
     * time. The short exposure time is derived from the long via the
     * ExposureTimeRatio setting.
     *
     * For the high dynamic range imager: a triple exposure time is used and it
     * is not possible to change it.
     *
     * Exposure time is expressed in microsecs.
     */
    int exposure_time_;

    /**
     * This is a semi-colon separated string of absolute exposure times in
     * microseconds. As opposed to computing the absolute exposure from the
     * `exposure_time_` and `exposure_time_ratio_` (where applicable), this can
     * be consulted for absolute values.
     */
    std::string exposure_time_list_;

    /** Ratio of long to short exposure time */
    int exposure_time_ratio_;

    /** Frame rate of the sensor */
    double frame_rate_;

    /** Minimum amplitude return required for a good pixel */
    int minimum_amplitude_;

    /** Toggles the image resolution */
    int resolution_;

    /** Sets a cuboid region of interest */
    std::string clipping_cuboid_;

    /** The type code for the applied spatial filter */
    int spatial_filter_type_;

    int symmetry_threshold_;

    /** The type code for the applied temporal filter */
    int temporal_filter_type_;

    int three_freq_max_3f_line_dist_percentage_;
    int three_freq_max_2f_line_dist_percentage_;
    int two_freq_max_line_dist_percentage_;

    /** Type of imager */
    std::string type_;

    /** Maximum frame rate allowed given the overall imager config */
    double max_allowed_led_frame_rate_;

  }; // end: class ImagerConfig

} // end: namespace o3d3xx


#endif // __O3D3XX_IMAGER_CONFIG_H__
