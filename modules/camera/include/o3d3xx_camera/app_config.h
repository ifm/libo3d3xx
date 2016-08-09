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
#ifndef __O3D3XX_APP_CONFIG_H__
#define __O3D3XX_APP_CONFIG_H__

#include <functional>
#include <memory>
#include <unordered_map>
#include <string>

namespace o3d3xx
{
  /**
   * Value object for holding application configuration information. This class
   * is not thread safe.
   */
  class AppConfig
  {
  public:
    using Ptr = std::shared_ptr<AppConfig>;

    /**
     * Maps a parameter name as it is stored on the camera to a mutator
     * function that is used to set its value on an AppConfig instance.
     */
    static const
    std::unordered_map<std::string,
                       std::function<void(AppConfig*,
                                          const std::string&)> >
    mutator_map;

    /**
     * Factory function for instantiating an AppConfig instance from a JSON
     * string.
     *
     * @param[in] json A string of JSON encoding application config
     * parameters.
     *
     * @parma[in] app_ptr A shared pointer to an AppConfig to bootstrap
     *                    the default values from.
     *
     * @return A shared pointer to an AppConfig instance based on the
     *         JSON encoding.
     */
    static o3d3xx::AppConfig::Ptr
    FromJSON(const std::string& json,
             o3d3xx::AppConfig::Ptr app_ptr = nullptr);

    /**
     * Initializes an application configuration utilizing default values (per
     * the sensor documentation)
     */
    AppConfig();

    /**
     * Initializes an application configuration based on a hash table mapping
     * keys to values.
     */
    AppConfig(const std::unordered_map<std::string, std::string>& params);

    /**
     * Serializes the current state of the application config to JSON.
     */
    std::string ToJSON() const;

    // Accessor / Mutators
    std::string Name() const noexcept;
    void SetName(const std::string& name) noexcept;

    std::string Description() const noexcept;
    void SetDescription(const std::string& descr) noexcept;

    int TriggerMode() const noexcept;
    void SetTriggerMode(int mode) noexcept;

    std::string PcicTcpResultSchema() const noexcept;
    void SetPcicTcpResultSchema(const std::string& schema) noexcept;

    std::string PcicEipResultSchema() const noexcept;
    void SetPcicEipResultSchema(const std::string& schema) noexcept;

    std::string PcicPnioResultSchema() const noexcept;
    void SetPcicPnioResultSchema(const std::string& schema) noexcept;

    std::string LogicGraph() const noexcept;
    void SetLogicGraph(const std::string& graph) noexcept;

    std::string Type() const noexcept;
    void SetType(const std::string& type) noexcept;

  protected:
    /** Name of the application */
    std::string name_;

    /** Description of the application */
    std::string description_;

    /** Trigger mode for image acquisition */
    int trigger_mode_;

    /**
     * The schema that defines which images and result data will be sent back
     * when acquiring images with this application set as active. It defines
     * the order of data elements and separators. The allowed images include:
     * `AmplitudeImage', `IntensityImage', `DistanceImage', `XImage', `YImage',
     * `ZImage', `ConfidenceImage', `DiagnosticData'.
     */
    std::string pcic_tcp_result_schema_;

    /**
     * Same as the `pcic_tcp_result_schema` except this result schema applies
     * to the data sent over the EtherNet/IP (and industrial ethernet standard)
     * interface.
     */
    std::string pcic_eip_result_schema_;

    /**
     * @see pcic_tcp_result_schema_
     * @see pcic_eip_result_schema_
     */
    std::string pcic_pnio_result_schema_;

    /**
     * @todo document this parameter
     */
    std::string logic_graph_;

    /**
     * READ-ONLY
     *
     * @todo document this parameter
     */
    std::string type_;

  }; // end: class AppConfig

} // end: namespace o3d3xx

#endif // __O3D3XX_APP_CONFIG_H__
