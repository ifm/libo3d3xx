// -*- c++ -*-
/*
 * Copyright (C) 2015 Love Park Robotics, LLC
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
#ifndef __O3D3XX_TEMPORAL_FILTER_CONFIG_H__
#define __O3D3XX_TEMPORAL_FILTER_CONFIG_H__

#include <functional>
#include <memory>
#include <unordered_map>
#include <string>

namespace o3d3xx
{
  /**
   * Base class for temporal filters.
   */
  class TemporalFilterConfig
  {
  public:
    using Ptr = std::shared_ptr<TemporalFilterConfig>;
    TemporalFilterConfig(int type = 0, const std::string& type_str = "Off");

    /**
     * Factory function for instantiating a TemporalFilterConfig instance
     * from a JSON string.
     *
     * @param[in] json A string of json encoding the filter config parameters.
     */
    static o3d3xx::TemporalFilterConfig::Ptr
    FromJSON(const std::string& json);

    /**
     * Serializes the current state of the filter confgi to JSON
     */
    std::string ToJSON() const;

    // accessor/mutators
    int Type() const noexcept;
    std::string TypeStr() const noexcept;

    int NumberOfImages() const;
    void SetNumberOfImages(int n_imgs);

  protected:
    int type_;
    std::string type_str_;
    int number_of_images_;

  }; // end: class TemporalFilterConfig

  class TemporalMeanFilterConfig : public TemporalFilterConfig
  {
  public:
    using Ptr = std::shared_ptr<TemporalMeanFilterConfig>;
    TemporalMeanFilterConfig();

  }; // end: class TemporalMeanFilterConfig

  class TemporalAdaptiveExponentialFilterConfig : public TemporalFilterConfig
  {
  public:
    using Ptr = std::shared_ptr<TemporalAdaptiveExponentialFilterConfig>;
    TemporalAdaptiveExponentialFilterConfig();

  }; // end: class TemporalAdaptiveExponentialFilterConfig

} // end: namespace o3d3xx

#endif // __O3D3XX_TEMPORAL_FILTER_CONFIG_H__
