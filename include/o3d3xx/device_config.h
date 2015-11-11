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
#ifndef __O3D3XX_DEVICE_CONFIG_H__
#define __O3D3XX_DEVICE_CONFIG_H__

#include <functional>
#include <memory>
#include <unordered_map>
#include <string>

namespace o3d3xx
{
  /**
   * Value object for holding device configuration information. This class is
   * not thread safe.
   */
  class DeviceConfig
  {
  public:
    using Ptr = std::shared_ptr<DeviceConfig>;

    /**
     * Maps a parameter name as it is stored on the camera to
     * a mutator function that is used to set its value on a DeviceConfig
     * instance.
     */
    static const
    std::unordered_map<std::string,
                       std::function<void(DeviceConfig*,
                                          const std::string&)> >
    mutator_map;

    /**
     * Factory function for instantiating a DeviceConfig instance from a JSON
     * string.
     *
     * @param[in] json A string of JSON encoding device config parameters.
     *
     * @param[in] devp A shared pointer to a DeviceConfig to boot strap the
     *                 default values from.
     *
     * @return A shared pointer to a device config instance based on
     *         the passed in JSON encoding.
     */
    static o3d3xx::DeviceConfig::Ptr
    FromJSON(const std::string& json,
             o3d3xx::DeviceConfig::Ptr devp = nullptr);

    /**
     * Initializes a device configuration utilizing default values (per the
     * sensor documentation ... as of this writing).
     */
    DeviceConfig();

    /**
     * Initializes a device configuration based on a hash table mapping
     * keys to values. Typically this would be as a result of calling
     * `GetAllParameters' on a `camera' instance.
     */
    DeviceConfig(const std::unordered_map<std::string, std::string>& params);

    /**
     * Serializes the current state of the device config to JSON.
     */
    std::string ToJSON() const;

    // Accessor / Mutators
    std::string Name() const noexcept;
    void SetName(const std::string& name) noexcept;

    std::string Description() const noexcept;
    void SetDescription(const std::string& description) noexcept;

    int ActiveApplication() const noexcept;
    void SetActiveApplication(int idx) noexcept;

    bool PcicEipEnabled() const noexcept;
    void SetPcicEipEnabled(bool on) noexcept;

    int PcicTCPPort() const noexcept;
    void SetPcicTCPPort(int port) noexcept;

    int PcicProtocolVersion() const noexcept;
    void SetPcicProtocolVersion(int version) noexcept;

    int IOLogicType() const noexcept;
    void SetIOLogicType(int type) noexcept;

    bool IODebouncing() const noexcept;
    void SetIODebouncing(bool on) noexcept;

    int IOExternApplicationSwitch() const noexcept;
    void SetIOExternApplicationSwitch(int val) noexcept;

    int SessionTimeout() const noexcept;
    void SetSessionTimeout(int secs) noexcept;

    int ServiceReportPassedBuffer() const noexcept;
    void SetServiceReportPassedBuffer(int len) noexcept;

    int ServiceReportFailedBuffer() const noexcept;
    void SetServiceReportFailedBuffer(int len) noexcept;

    double ExtrinsicCalibTransX() const noexcept;
    void SetExtrinsicCalibTransX(double x) noexcept;

    double ExtrinsicCalibTransY() const noexcept;
    void SetExtrinsicCalibTransY(double y) noexcept;

    double ExtrinsicCalibTransZ() const noexcept;
    void SetExtrinsicCalibTransZ(double z) noexcept;

    double ExtrinsicCalibRotX() const noexcept;
    void SetExtrinsicCalibRotX(double x) noexcept;

    double ExtrinsicCalibRotY() const noexcept;
    void SetExtrinsicCalibRotY(double y) noexcept;

    double ExtrinsicCalibRotZ() const noexcept;
    void SetExtrinsicCalibRotZ(double z) noexcept;

    int IPAddressConfig() const noexcept;
    bool PasswordActivated() const noexcept;
    int OperatingMode() const noexcept;
    std::string DeviceType() const noexcept;
    std::string ArticleNumber() const noexcept;
    std::string ArticleStatus() const noexcept;
    double Uptime() const noexcept;
    int ImageTimestampReference() const noexcept;
    double TemperatureFront1() const noexcept;
    double TemperatureFront2() const noexcept;
    double TemperatureIMX6() const noexcept;
    double TemperatureIllu() const noexcept;

  protected:
    /**
     * User-defined name of the device (max 64 characters)
     */
    std::string name_;

    /**
     * User-defined description of the device (max 500 characters)
     */
    std::string description_;

    /**
     * Index of the active application
     */
    int active_application_;

    /**
     * Flag indicating EIP interface enabled status
     */
    bool pcic_eip_enabled_;

    /**
     * TCP/IP port for the PCIC connections
     */
    int pcic_tcp_port_;

    /**
     * Sub-protocol of PCIC, see specification of PCIC
     */
    int pcic_protocol_version_;

    /**
     * Defines the logic-type of all digital pins
     */
    int io_logic_type_;

    /**
     * Whether to apply or not apply I/O debounding to all inputs
     */
    bool io_debouncing_;

    /**
     * @todo fill in meaning of parameter
     */
    int io_extern_application_switch_;

    /**
     * number of seconds which a session stays before a call to "heartbeat" is
     * needed.
     */
    int session_timeout_;

    /**
     * @todo fill in meaning of parameter
     */
    int service_report_passed_buffer_;

    /**
     * @todo fill in meaning of parameter
     */
    int service_report_failed_buffer_;

    /**
     * Extrinsic calibration, translation in x-direction
     */
    double extrinsic_calib_trans_x_;

    /**
     * Extrinsic calibration, translation in y-direction
     */
    double extrinsic_calib_trans_y_;

    /**
     * Extrinsic calibration, translation in z-direction
     */
    double extrinsic_calib_trans_z_;

    /**
     * Extrinsic calibration, rotation around x-axis
     */
    double extrinsic_calib_rot_x_;

    /**
     * Extrinsic calibration, rotation around y-axis
     */
    double extrinsic_calib_rot_y_;

    /**
     * Extrinsic calibration, rotation around z-axis
     */
    double extrinsic_calib_rot_z_;

    /**
     * READONLY: How the sensor obtained its IP address
     */
    int ip_address_config_;

    /**
     * READONLY: true if password protection is enabled
     */
    bool password_activated_;

    /**
     * READONLY: mode of device (RUN or EDIT)
     */
    int operating_mode_;

    /**
     * READONLY: a type description, unique by imager, evaluation-logic, and
     * device interface.
     */
    std::string device_type_;

    /**
     * READONLY: official catalog number
     */
    std::string article_number_;

    /**
     * READONLY: official two-letter status code
     */
    std::string article_status_;

    /**
     * READONLY: Hours since last reboot
     */
    double uptime_;

    /**
     * READONLY: The image data contains a timestamp (32-bit int, usec). This
     * should return the current timestamp as a reference to the images
     * received.
     */
    int image_timestamp_ref_;

    /**
     * READONLY: Temperature measured in the device (deg Celcius) by the first
     * sensor on the imager board.
     */
    double temp_front1_;

    /**
     * READONLY: Temperature measured in the device (deg Celcius) by the second
     * sensor on the imager board.
     */
    double temp_front2_;

    /**
     * READONLY: Temperature of the main CPU (deg Celcius)
     */
    double temp_imx6_;

    /**
     * READONLY: Temperature of the illumination board (deg Celcius)
     */
    double temp_illu_;

  }; // end: class DeviceConfig

} // end: namespace 03d3xx

#endif // __O3D3XX_DEVICE_CONFIG_H__
