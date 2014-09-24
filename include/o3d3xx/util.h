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

#include <boost/thread/once.hpp>

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
    static boost::once_flag init_;

  }; // end: class Logging

} // end: namespace o3d3xx

#endif // __O3D3XX_UTIL_H__