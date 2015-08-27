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

#include "o3d3xx/err.h"
#include <cstring>

const int O3D3XX_NO_ERRORS = 0;
const int O3D3XX_XMLRPC_FAILURE = -9000;
const int O3D3XX_THREAD_INTERRUPTED = -9001;
const int O3D3XX_IMG_CHUNK_NOT_FOUND = -9002;
const int O3D3XX_XMLRPC_TIMEOUT = -9003;
const int O3D3XX_XMLRPC_FINFAIL = -9004;
const int O3D3XX_XMLRPC_OBJ_NOT_FOUND = -9005;
const int O3D3XX_XMLRPC_INVALID_PARAM = -9006;
const int O3D3XX_XMLRPC_INVALID_APPLICATION = -9007;
const int O3D3XX_XMLRPC_APPLICATION_IN_EDIT_MODE = -9008;
const int O3D3XX_XMLRPC_TOO_MANY_APPLICATIONS = -9009;
const int O3D3XX_XMLRPC_NOT_EDITING_APPLICATION = -9010;
const int O3D3XX_XMLRPC_EDIT_SESSION_ALREADY_ACTIVE = -9011;
const int O3D3XX_XMLRPC_METHOD_NOT_FOUND = -9012;
const int O3D3XX_EXPOSURE_TIME_NOT_ACCESSIBLE = -9013;
const int O3D3XX_XMLRPC_VALUE_OUT_OF_RANGE = -9014;
const int O3D3XX_IO_ERROR = -9015;
const int O3D3XX_VALUE_OUT_OF_RANGE = -9016;
const int O3D3XX_INVALID_ARGUMENT = -9017;
const int O3D3XX_XMLRPC_EIP = -9018;
const int O3D3XX_PCIC_BAD_REPLY = -9019;

const char *o3d3xx::strerror(int errnum)
{
  switch (errnum) {
  case O3D3XX_NO_ERRORS:
    return "OK";
  case O3D3XX_XMLRPC_FAILURE:
    return "Unknown XMLRPC failure";
  case O3D3XX_THREAD_INTERRUPTED:
    return "Thread interrupted";
  case O3D3XX_IMG_CHUNK_NOT_FOUND:
    return "Image chunk not found";
  case O3D3XX_XMLRPC_TIMEOUT:
    return "XMLRPC call timed out";
  case O3D3XX_XMLRPC_FINFAIL:
    return "XMLRPC finished but the called failed";
  case O3D3XX_XMLRPC_OBJ_NOT_FOUND:
    return "XMLRPC server object not found";
  case O3D3XX_XMLRPC_INVALID_PARAM:
    return "XMLRPC requested parameter is invalid";
  case O3D3XX_XMLRPC_INVALID_APPLICATION:
    return "XMLRPC invalid application index";
  case O3D3XX_XMLRPC_APPLICATION_IN_EDIT_MODE:
    return "XMLRPC application is in edit-mode";
  case O3D3XX_XMLRPC_TOO_MANY_APPLICATIONS:
    return "XMLRPC maximum number of applications exceeded";
  case O3D3XX_XMLRPC_NOT_EDITING_APPLICATION:
    return "XMLRPC no application in edit status";
  case O3D3XX_XMLRPC_EDIT_SESSION_ALREADY_ACTIVE:
    return "XMLRPC device already has an edit session active";
  case O3D3XX_XMLRPC_METHOD_NOT_FOUND:
    return "XMLRPC method not found";
  case O3D3XX_EXPOSURE_TIME_NOT_ACCESSIBLE:
    return "Exposure time not accessible for imager type";
  case O3D3XX_XMLRPC_VALUE_OUT_OF_RANGE:
    return "XMLRPC value out of range";
  case O3D3XX_IO_ERROR:
    return "I/O error";
  case O3D3XX_VALUE_OUT_OF_RANGE:
    return "Value out of range";
  case O3D3XX_INVALID_ARGUMENT:
    return "Invalid argument(s)";
  case O3D3XX_XMLRPC_EIP:
    return "XMLRPC Unable to enable/disable PCIC Ethernet/IP output";
  case O3D3XX_PCIC_BAD_REPLY:
    return "Error response on PCIC socket";
  default:
    return ::strerror(errnum);
  }
}

o3d3xx::error_t::error_t(int errnum)
  : std::exception(), errnum_(errnum) { }

int o3d3xx::error_t::code() const noexcept
{
  return this->errnum_;
}

const char *o3d3xx::error_t::what() const noexcept
{
  return o3d3xx::strerror(this->code());
}
