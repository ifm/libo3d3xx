// -*- c++ -*-
/*
 * Copyright (C) 2016 Love Park Robotics, LLC
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

#ifndef __O3D3XX_PCIC_SCHEMA_H__
#define __O3D3XX_PCIC_SCHEMA_H__

#include <cstdint>
#include <string>

namespace o3d3xx
{
  extern const std::uint16_t DEFAULT_SCHEMA_MASK;

  // Constants used to create "pluggable schema masks"
  extern const std::uint16_t IMG_RDIS;  // radial distance
  extern const std::uint16_t IMG_AMP;   // normalized amplitude
  extern const std::uint16_t IMG_RAMP;  // raw amplitude
  extern const std::uint16_t IMG_CART;  // Cartesian
  extern const std::uint16_t IMG_UVEC;  // Unit vectors
  extern const std::uint16_t EXP_TIME;  // Exposure times
  extern const std::uint16_t ILLU_TEMP; // Illumination temperature

  /**
   * Utility function to build a PCIC schema from a mask
   *
   * @param[in] mask The mask to use to build the schema
   * @return The PCIC schema as a string
   */
  std::string make_pcic_schema(std::uint16_t mask);

  /**
   * Utility function to create a schema mask from a string.
   *
   * The passed in string should contain valid symbolic constants `OR'd`
   * together. For example: IMG_RDIS|IMG_AMP|IMG_RAMP|IMG_CART|IMG_UVEC
   *
   * @param[in] in The string to parse to generate the mask
   * @return The PCIC schema encoed by the `in` string.
   */
  std::uint16_t schema_mask_from_string(const std::string& in);

} // end: namespace o3d3xx

#endif // __O3D3XX_PCIC_SCHEMA_H__
