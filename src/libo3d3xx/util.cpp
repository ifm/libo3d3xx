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

#include "o3d3xx/util.h"
#include <boost/thread/once.hpp>
#include <glog/logging.h>
#include "o3d3xx/version.h"

//--------------------------------------------------
// Logging-related
//--------------------------------------------------

boost::once_flag o3d3xx::Logging::init_ = BOOST_ONCE_INIT;

void
o3d3xx::Logging::Init()
{
  boost::call_once(o3d3xx::Logging::_Init,
		   o3d3xx::Logging::init_);
}

void
o3d3xx::Logging::_Init()
{
  google::InitGoogleLogging(O3D3XX_LIBRARY_NAME);
}