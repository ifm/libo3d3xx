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

#ifndef __O3D3XX_UTIL_H__
#define __O3D3XX_UTIL_H__

#include <algorithm>
#include <mutex>
#include <string>
#include <unordered_map>
#include <endian.h>
#include <xmlrpc-c/base.hpp>

namespace o3d3xx
{
  /**
   * Wrapper to ensure google logging only initializes once. Secondarily, this
   * also acts as a very thin abstraction layer over the logging subsystem.
   */
  class Logging
  {
  public:
    /**
     * Function used to initialize the library's logging subystem. This function
     * is thread safe _and_ idempotent. However, it must be called _at least
     * once_ prior to writing log messages.
     */
    static void Init();

  private:
    /**
     * Actually runs the one-time initialization code.
     */
    static void _Init();

    /**
     * Flag indicating the initialization state of the logging subsystem.
     */
    static std::once_flag init_;

  }; // end: class Logging

  /**
   * The function converts an xmlrpc_c::value_struct into a
   * std::unordered_map. The assumption is that all keys and values are
   * strings.
   *
   * @param[in] xmlrcp_c::value_struct Structure of strings mapped to strings
   * @return std::unordered_map<std::string, std::string>
   */
  std::unordered_map<std::string, std::string> const
  value_struct_to_map(const xmlrpc_c::value_struct& vs);

  /**
   * Converts a string to a bool.
   */
  bool stob(const std::string& s);

  /**
   * Create a value of type T from sizeof(T) bytes of the passed in byte
   * buffer. Given that the o3d3xx camera transmits data in little endian
   * format, this function will swap bytes if necessary for the host
   * representation of T.
   *
   * @param[in] buff A pointer to a buffer in memory intended to be interpreted
   * as data of type T and assuming the buffer is little endian.
   *
   * @return An interpretation of `buff` as type T with bytes swapped as
   * appropriate for the host's byte ordering semantics.
   */
  template<typename T>
  T mkval(const unsigned char *buff)
  {
    union
    {
      T v;
      unsigned char bytes[sizeof(T)];
    } value;

#if __BYTE_ORDER == __BIG_ENDIAN
    std::reverse_copy(buff, buff + sizeof(T), value.bytes);
#else
    std::copy(buff, buff + sizeof(T), value.bytes);
#endif

    return value.v;
  }

} // end: namespace o3d3xx

#endif // __O3D3XX_UTIL_H__
