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
#include <mutex>
#include <string>
#include <unordered_map>
#include <glog/logging.h>
#include <xmlrpc-c/base.hpp>
#include "o3d3xx/version.h"

//--------------------------------------------------
// Logging-related
//--------------------------------------------------

std::once_flag o3d3xx::Logging::init_;

void
o3d3xx::Logging::Init()
{
  std::call_once(o3d3xx::Logging::init_, o3d3xx::Logging::_Init);
}

void
o3d3xx::Logging::_Init()
{
  google::InitGoogleLogging(O3D3XX_LIBRARY_NAME);
}

//--------------------------------------------------
// XMLRPC utilities
//--------------------------------------------------

std::unordered_map<std::string, std::string> const
o3d3xx::value_struct_to_map(const xmlrpc_c::value_struct& vs)
{
  std::map<std::string, xmlrpc_c::value> const
    resmap(static_cast<std::map<std::string, xmlrpc_c::value> >(vs));

  std::unordered_map<std::string, std::string> retval;
  for (auto& kv : resmap)
    {
      retval[kv.first] = std::string(xmlrpc_c::value_string(kv.second));
    }

  return retval;
}
