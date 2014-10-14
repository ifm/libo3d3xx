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
#ifndef __O3D3XX_NET_CONFIG_H__
#define __O3D3XX_NET_CONFIG_H__

#include <functional>
#include <memory>
#include <unordered_map>
#include <string>

namespace o3d3xx
{
  /**
   * Value object for holding network configuration information. This class is
   * not thread safe.
   */
  class NetConfig
  {
  public:
    using Ptr = std::shared_ptr<NetConfig>;

    /**
     * Maps a parameter name as it is stored on the camera to a mutator
     * function that is used to set its value on a NetConfig instance.
     */
    static const
    std::unordered_map<std::string,
		       std::function<void(NetConfig*,
					  const std::string&)> >
    mutator_map;

    /**
     * Factory function for instantiating a NetConfig instance from a JSON
     * string.
     *
     * @param[in] json A string of JSON encoding network config parameters.
     */
    static o3d3xx::NetConfig::Ptr FromJSON(const std::string& json);

    /**
     * Initializes a network configuration utilizing default values (per the
     * sensor documentation ... as of this writing).
     */
    NetConfig();

    /**
     * Initializes a network configuration based on a hash table mapping keys
     * to values.
     */
    NetConfig(const std::unordered_map<std::string, std::string>& params);

    /**
     * Serializes the current state of the network config to JSON.
     */
    std::string ToJSON() const;

    // Accessor / Mutators
    std::string MACAddress() const noexcept;
    int NetworkSpeed() const noexcept;

    std::string StaticIPv4Address() const noexcept;
    void SetStaticIPv4Address(const std::string& ip) noexcept;

    std::string StaticIPv4Gateway() const noexcept;
    void SetStaticIPv4Gateway(const std::string& gw) noexcept;

    std::string StaticIPv4SubNetMask() const noexcept;
    void SetStaticIPv4SubNetMask(const std::string& mask) noexcept;

    bool UseDHCP() const noexcept;
    void SetUseDHCP(bool on) noexcept;

  protected:
    /** MAC address of the sensor */
    std::string mac_address_;

    /** Network speed code: XXX @todo decode these values */
    int network_speed_;

    /** static ip address */
    std::string ip_;

    /** static gateway ip address */
    std::string gw_;

    /** static subnetwork mask */
    std::string subnet_;

    /** flag indicating whether or not the IP address of the sensor is obtained
	via DHCP or static */
    bool use_dhcp_;

  }; // end: class NetConfig

} // end: namespace o3d3xx

#endif // __O3D3XX_NET_CONFIG_H__
