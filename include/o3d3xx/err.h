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

#ifndef __O3D3XX_ERR_H__
#define __O3D3XX_ERR_H__

#include <exception>

extern const int O3D3XX_NO_ERRORS;
extern const int O3D3XX_XMLRPC_FAILURE;
extern const int O3D3XX_THREAD_INTERRUPTED;

namespace o3d3xx
{
  /**
   * @brief Human consumable string for a O3D3XX error
   *
   * @param[in] errnum The error number to translate into a string
   * @return A stringified version of the error
   */
  const char *strerror(int errnum);

  /**
   * @brief Exception wrapper for library and system errors encountered by the
   * library.
   */
  class error_t : public std::exception
  {
  public:
    /**
     * The ctor simply sets the error value into a local instance variable that
     * may be retrieved with a call to code().
     */
    error_t(int errnum);

    /**
     * Exception message.
     */
    virtual const char *what() const noexcept;

    /**
     * Accessor to the underlying sensor error code.
     */
    int code() const noexcept;

  private:
    /**
     * The error code from the sensor that this exception wraps.
     */
    int errnum_;

  }; // end: class error_t

} // end: namespace o3d3xx

#endif // __O3D3XX_ERR_H__
