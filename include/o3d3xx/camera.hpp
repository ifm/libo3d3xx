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
#ifndef __O3D3XX_CAMERA_H__
#define __O3D3XX_CAMERA_H__

#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/algorithm/string/replace.hpp>
#include <glog/logging.h>
#include <xmlrpc-c/client.hpp>
#include "o3d3xx/device_config.h"
#include "o3d3xx/net_config.h"
#include "o3d3xx/err.h"

namespace o3d3xx
{
  extern const std::string DEFAULT_PASSWORD;
  extern const std::string DEFAULT_IP;
  extern const std::string DEFAULT_SUBNET;
  extern const std::string DEFAULT_GW;
  extern const std::uint32_t DEFAULT_XMLRPC_PORT;
  extern const int MAX_HEARTBEAT;
  extern const int NET_WAIT;

  extern const std::string XMLRPC_MAIN;
  extern const std::string XMLRPC_SESSION;
  extern const std::string XMLRPC_EDIT;
  extern const std::string XMLRPC_DEVICE;
  extern const std::string XMLRPC_NET;
  extern const std::string XMLRPC_APP;
  extern const std::string XMLRPC_IMAGER;

  /**
   * Software interface to the IFM O3D3XX camera.
   */
  class Camera
  {
  public:
    using Ptr = std::shared_ptr<Camera>;

    /**
     * Represents basic information about an application stored on the sensor.
     */
    using app_entry_t = struct {
      int index;
      int id;
      std::string name;
      std::string description;
    };

    /**
     * Camera boot-up modes
     */
    enum class boot_mode : int { PRODUCTIVE = 0, RECOVERY = 1 };

    /**
     * Camera operating modes
     */
    enum class operating_mode : int { RUN = 0, EDIT = 1 };

    /**
     * Allowed values for I/O logic of digital pins
     */
    enum class io_logic_type : int { NPN = 0, PNP = 1 };

    /**
     * Allowed values for I/O external application switch
     */
    enum class io_extern_app_switch : int
    { OFF = 0, STATIC_IO = 1, PULSE_IO = 2, PULSE_TRIGGER = 3 };

    /**
     * Ways in which the camera can obtain an IP address
     */
    enum class ip_address_config : int
    { STATIC = 0, DHCP = 1, LINK_LOCAL = 2, DISCOVERY = 3 };

    /**
     * Initializes the camera interface utilizing library defaults for
     * password, ip address, and xmlrpc port unless explictly passed in.
     *
     * @param[in] ip The ip address of the camera
     * @param[in] xmlrpc_port The tcp port the sensor's XMLRPC server is
     *                        listening on
     * @param[in] password Password required for establishing an "edit session"
     *                     with the sensor. Edit sessions allow for mutating
     *                     camera parameters.
     */
    Camera(const std::string& ip = o3d3xx::DEFAULT_IP,
	   const std::uint32_t xmlrpc_port = o3d3xx::DEFAULT_XMLRPC_PORT,
	   const std::string& password = o3d3xx::DEFAULT_PASSWORD);

    /**
     * Dtor will cancel any open sessions with the camera hardware.
     */
    virtual ~Camera();

    // copy and move semantics
    Camera(Camera&&) = delete;
    Camera& operator=(Camera&&) = delete;
    Camera(Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    // Accessor/Mutators
    std::string GetIP();
    void SetIP(const std::string& ip);

    std::uint32_t GetXMLRPCPort();
    void SetXMLRPCPort(const std::uint32_t& port);

    std::string GetPassword();
    void SetPassword(const std::string& password);

    std::string GetXMLRPCURLPrefix();
    void SetXMLRPCURLPrefix(const std::string& ip_, const std::uint32_t& port);

    std::string GetSessionID();
    void SetSessionID(const std::string& id);

    // XMLRPC Calls to the hardware -- these go over the network
    std::string GetParameter(const std::string& param);
    std::unordered_map<std::string, std::string> GetAllParameters();
    std::unordered_map<std::string, std::string> GetSWVersion();
    std::unordered_map<std::string, std::string> GetHWInfo();
    std::vector<app_entry_t> GetApplicationList();
    void Reboot(const boot_mode& mode);
    std::string RequestSession();
    bool CancelSession();
    int Heartbeat(int hb);
    bool SetOperatingMode(const operating_mode& mode);

    // XMLRPC: DeviceConfig object
    o3d3xx::DeviceConfig::Ptr GetDeviceConfig();
    bool ActivatePassword();
    bool DisablePassword();
    void SetDeviceConfig(const o3d3xx::DeviceConfig* config);
    bool SaveDevice();

    // XMLRPC: NetConfig object
    std::unordered_map<std::string, std::string> GetNetParameters();
    o3d3xx::NetConfig::Ptr GetNetConfig();
    void SetNetConfig(const o3d3xx::NetConfig* config);
    bool SaveNet();

  protected:
    /** Password for mutating camera parameters */
    std::string password_;

    /** Protects password_ */
    std::mutex password_mutex_;

    /** The ip address of the sensor */
    std::string ip_;

    /** Protects ip_ */
    std::mutex ip_mutex_;

    /** The tcp port of the xml-rpc server of the sensor */
    std::uint32_t xmlrpc_port_;

    /** Protects xmlrpc_port_ */
    std::mutex xmlrpc_port_mutex_;

    /** The xmlrpc URL prefix */
    std::string xmlrpc_url_prefix_;

    /** Protects xmlrpc_url_prefix_ */
    std::mutex xmlrpc_url_prefix_mutex_;

    /** XMLRPC client */
    xmlrpc_c::clientPtr xclient_;

    /** Protects xmlrpc_client_ */
    std::mutex xclient_mutex_;

    /** Session ID for mutating camera parameters */
    std::string session_;

    /** Protects session_ */
    std::mutex session_mutex_;

  private:
    /** Terminates iteration over the parameter pack in _XSetParams */
    void _XSetParams(xmlrpc_c::paramList& params) { }

    /**
     * Recursively processes the parameter pack `args' as a list and sets those
     * values into the `params' reference.
     */
    template <typename T, typename... Args>
    void _XSetParams(xmlrpc_c::paramList& params, T value, Args... args)
    {
      params.addc(value);
      this->_XSetParams(params, args...);
    }

    /**
     * Encapsulates XMLRPC calls to the sensor and unifies the trapping of
     * communication errors with the sensor.
     *
     * @param[in] url The URL of the XMLPRC object to invoke
     *
     * @param[in] sensor_method_name The name of the XMLRPC method as defined
     *                               by the sensor
     *
     * @param[in] args A variable list of arguments that should be passed to
     *                 the XMLRPC call.
     *
     * @return The XMLRPC-encoded return value from the sensor.
     *
     * @throws o3d3xx::error_t If an error occurs in the XMLRPC call
     */
    template <typename... Args>
    xmlrpc_c::value const
    _XCall(std::string& url, const std::string& sensor_method_name,
	   Args... args)
    {
      xmlrpc_c::paramList params;
      this->_XSetParams(params, args...);
      xmlrpc_c::rpcPtr rpc(sensor_method_name, params);

      boost::algorithm::replace_all(url, "XXX", this->GetSessionID());
      xmlrpc_c::carriageParm_curl0 cparam(url);

      std::lock_guard<std::mutex> lock(this->xclient_mutex_);
      try
	{
	  rpc->call(this->xclient_.get(), &cparam);
	  return rpc->getResult();
	}
      catch (const std::exception& ex)
	{
	  LOG(ERROR) << url << " -> "
	  	     << sensor_method_name << " : "
	  	     << ex.what();

	  if (! rpc->isFinished())
	    {
	      throw(o3d3xx::error_t(O3D3XX_XMLRPC_TIMEOUT));
	    }
	  else if (! rpc->isSuccessful())
	    {
	      xmlrpc_c::fault f = rpc->getFault();
	      int ifm_error = f.getCode();

	      switch (ifm_error)
		{
		case 101000:
		  throw(o3d3xx::error_t(O3D3XX_XMLRPC_INVALID_PARAM));
		  break;

		case 100000:
		  LOG(WARNING) << "Invalid session object? "
			       << this->GetSessionID();
		  throw(o3d3xx::error_t(O3D3XX_XMLRPC_OBJ_NOT_FOUND));
		  break;

		default:
		  LOG(ERROR) << "IFM error code: " << ifm_error;
		  throw(o3d3xx::error_t(O3D3XX_XMLRPC_FINFAIL));
		  break;
		}
	    }
	  else
	    {
	      throw(o3d3xx::error_t(O3D3XX_XMLRPC_FAILURE));
	    }
	}
    }

    /** _XCall wrapper for XMLRPC calls to the "Main" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallMain(const std::string& sensor_method_name, Args... args)
    {
      std::string url = this->GetXMLRPCURLPrefix() + o3d3xx::XMLRPC_MAIN;
      return this->_XCall(url, sensor_method_name, args...);
    }

    /** _XCall wrapper for XMLRPC calls to the "Session" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallSession(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION;

      return this->_XCall(url, sensor_method_name, args...);
    }

    /** _XCall wrapper for XMLRPC calls to the "DeviceConfig" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallDevice(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT +
	o3d3xx::XMLRPC_DEVICE;

      return this->_XCall(url, sensor_method_name, args...);
    }

    /** _XCall wrapper for XMLRPC calls to the "NetworkConfig" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallNet(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT +
	o3d3xx::XMLRPC_DEVICE +
	o3d3xx::XMLRPC_NET;

      return this->_XCall(url, sensor_method_name, args...);
    }

  }; // end: class Camera

} // end: namespace o3d3xx

#endif // __O3D3XX_CAMERA_H__
