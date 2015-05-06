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
#include "o3d3xx/app_config.h"
#include "o3d3xx/imager_config.h"
#include "o3d3xx/spatial_filter_config.h"
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
  extern const std::string XMLRPC_SPATIALFILTER;
  extern const std::string XMLRPC_TEMPORALFILTER;

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
     * Image acquisition trigger modes
     */
    enum class trigger_mode : int
    { FREE_RUN = 1, PROCESS_INTERFACE = 2 };

    /**
     * Spatial filter types
     */
    enum class spatial_filter : int
    { OFF = 0, MEDIAN_FILTER = 1, MEAN_FILTER = 2, BILATERAL_FILTER = 3 };

    /**
     * Temporal filter types
     */
    enum class temporal_filter : int
    { OFF = 0, TEMPORAL_MEAN_FILTER = 1, ADAPTIVE_EXPONENTIAL_FILTER = 2 };

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

    /**
     * Stringifies the state of the camera to JSON
     *
     * This function can produce a string that can effectively function as a
     * backup of the camera configuration. The generated JSON shall be
     * importable by the `FromJSON' method.
     *
     * NOTE: This method will automatically open an "edit session" with the
     * camera if necessary so a call to `RequestSession()' is not necessary
     * prior to calling this method.
     *
     * @return A JSON string representing the current state of the hardware.
     *
     * @throw o3d3xx::error_t upon error
     */
    std::string ToJSON();

    /**
     * Configures the camera (the physical hardware, not just this software
     * object) from the parameters outlined in the passed in `json' file.
     *
     * We assume the input `json' file conforms to the structure produced by
     *`ToJSON'. However, not all keys in each substructure are required -- only
     * those that are present will be processed.
     *
     * Here are the basic rules for how this function works:
     *
     * 1. The "Device" section is processed and saved on the camera.
     * 2. The "Apps" section is processed. For each app:
     *    2.a. If the "Index" key is present, a current app at that `index' is
     *         looked up. If present, it is edited to reflect the data in the
     *         JSON file. If an app at that `index' is not present, a new app
     *         is created with the parameters from the JSON file. It is not
     *         guaranteed that the new app will have the specified `index'.
     *    2.b. If the "Index" key is not present, a new app is created with the
     *         parameters as specified in the JSON file.
     *    2.c. The active application is set by consulting the desired index
     *         of the `ActiveApplication" from the "Device" section of the
     *         JSON. If the specified `index' does not exist, the active
     *         application is not set.
     * 3. The "Net" section is processed. A reboot of the camera may be
     *    necessary after changing the camera's network parameters.
     *
     * NOTE: This method will automatically open an "edit session" with the
     * camera if necessary so a call to `RequestSession()' is not necessary
     * prior to calling this method.
     *
     * @param[in] json A json string describing the desired camera
     *                 configuration.
     *
     * @throw o3d3xx::error_t upon error
     */
    void FromJSON(const std::string& json);

    //---------------------------------------------
    // XMLRPC: Main object
    //---------------------------------------------

    /**
     * Accessor for device global parameters
     *
     * @param[in] param Name of device parameter
     * @return Value of the requested parameter
     *
     * @throw o3d3xx::error_t upon error
     */
    std::string GetParameter(const std::string& param);

    /**
     * Accessor for all device global parameters. This allows for reading
     * device parameters outside of an edit session.
     *
     * @return A hash table mapping parameter key to parameter value.
     *
     * @throw o3d3xx::error_t upon error
     */
    std::unordered_map<std::string, std::string> GetAllParameters();

    /**
     * Returns version information for all software components
     *
     * @return A hash table mapping software component to version string.
     *
     * @throw o3d3xx::error_t upon error
     */
    std::unordered_map<std::string, std::string> GetSWVersion();

    /**
     * Return hardware information for all components
     *
     * @return A hash table mapping hardware component to version string.
     *
     * @throw o3d3xx::error_t upon error
     */
    std::unordered_map<std::string, std::string> GetHWInfo();

    /**
     * Delivers basic information of all applications stored on the device.
     * A session is not required to call this function.
     *
     * @return A `vector' of `app_entry_t' structures describing basic
     * information about the applications stored on the camera.
     *
     * @throw o3d3xx::error_t upon error
     */
    std::vector<app_entry_t> GetApplicationList();

    /**
     * Reboot the sensor
     *
     * @param[in] mode The system mode that should be booted into after
     * shutdown.
     *
     * @throw o3d3xx::error_t upon error
     */
    void Reboot(const boot_mode& mode = o3d3xx::Camera::boot_mode::PRODUCTIVE);

    /**
     * Request a session-object for access to the configuration and changing of
     * device operating parameters.
     *
     * @return The session id as constructed by the sensor
     *
     * @throw o3d3xx::error_t upon error (e.g., unable to create a session)
     */
    std::string RequestSession();

    /**
     * Explicitly stops the current session with the sensor. If an application
     * is still in edit-mode, it implicitly has the same as effect as calling
     * `stopEditingApplication'
     *
     * NOTE: This function returns a boolean indicating the success/failure of
     * cancelling the session. The reason we return a bool and explicitly
     * supress exceptions is because camera dtors will cancel any open
     * sessions with the camera and we do not want the dtor to throw.
     *
     * @return true if the session was cancelled properly, false if an
     * exception was caught while trying to cancel the session.
     */
    bool CancelSession();

    /**
     * Heartbeat messages are used to keep a session with the sensor
     * alive. This function sends a heartbeat message to the sensor and sets when
     * the next heartbeat message is required.
     *
     * @param[in] hb The time (seconds) of when the next heartbeat message will
     * be required.
     *
     * @return The current timeout-interval in seconds for heartbeat messages.
     *
     * @throw o3d3xx::error_t upon error
     */
    int Heartbeat(int hb);

    /**
     * Changes the operating mode of the device. This is essentially a way to
     * toggle between editing parameters on the device and streaming data from
     * the device.
     *
     * @param[in] mode The mode to set the camera into.
     *
     * @throw o3d3xx::error_t upon error
     */
    void SetOperatingMode(const operating_mode& mode);

    //---------------------------------------------
    // XMLRPC: EditMode object
    //---------------------------------------------

    /**
     * Copies the application currently stored at `idx' to a new application
     * whose index is the return value of this function.
     *
     * @param[in] idx The application index to copy
     * @return The index of the new application.
     *
     * @throw o3d3xx::error_t upon error
     */
    int CopyApplication(int idx);

    /**
     * Deletes the application currently stored at `idx'.
     *
     * @param[in] idx The application index to delete
     *
     * @throw o3d3xx::error_t upon error
     */
    void DeleteApplication(int idx);

    /**
     * Creates a new (empty) application on the camera.
     *
     * @return The integer index of the new application
     *
     * @throw o3d3xx::error_t
     */
    int CreateApplication();

    /**
     * Changes the name and description of the application stored at index
     * `idx'.
     *
     * @param[in] idx The index of the application to edit
     * @param[in] name The new name for the application
     * @param[in] descr The new description for the application
     *
     * @throw o3d3xx::error_t upon error
     */
    void ChangeAppNameAndDescription(int idx,
				     const std::string& name,
				     const std::string& descr);

    /**
     * Puts the specified application into edit-status. This will attach an
     * application-object to the RPC interface. The name of the object will be
     * application independent. This does not change the active application.
     *
     * @param[in] idx The index of the application to set into edit-mode.
     *
     * @throw o3d3xx::error_t upon error
     */
    void EditApplication(int idx);

    /**
     * Detaches the application-object from RPC and discards all unsaved
     * changes on the current application being edited.
     *
     * @throw o3d3xx::error_t upon error
     */
    void StopEditingApplication();

    /**
     * Sets all configuration back to factory defaults. I.e., all applications
     * that are persistently stored on the camera will be deleted.
     *
     * @throw o3d3xx::error_t upon error
     */
    void FactoryReset();

    //---------------------------------------------
    // XMLRPC: ApplicationConfig object
    //---------------------------------------------

    /**
     * Returns a mapping of key/value pairs for the application object
     * currently attached to the XMLRPC server -- this is not necessary the
     * "active application" but rather the application specified by index to
     * `EditApplication(index)'.
     */
    std::unordered_map<std::string, std::string> GetAppParameters();

    /**
     * Saves the application that is currently attached to the XMLRPC server.
     */
    void SaveApp();

    /**
     * Returns an `AppConfig' instance for the application currently attached
     * to the XMLRPC server.
     */
    o3d3xx::AppConfig::Ptr GetAppConfig();

    /**
     * Sets parameters on the XMLRPC-attached application based on the passed
     * in `AppConfig' pointer.
     */
    void SetAppConfig(const o3d3xx::AppConfig* config);

    //---------------------------------------------
    // XMLRPC: ImagerConfig object
    //---------------------------------------------

    /**
     * Lists the available imager types that can be attached to an
     * application.
     */
    std::vector<std::string> GetAvailableImagerTypes();

    /**
     * Changes the imager type for the currently attached application.
     *
     * @param[in] type The identifying string for the new imager type. This
     * string should be one of the strings returned from
     * `GetAvailableImagerTypes()'.
     *
     * @throw o3d3xx::error_t upon error
     */
    void ChangeImagerType(const std::string& type);

    /**
     * Returns a mapping of key/value pairs for the imager of the application
     * object currently attached to the XMLRPC server -- this is not
     * necessarily the "active application" but rather the application
     * specified by index to `EditApplication(index)'.
     */
    std::unordered_map<std::string, std::string> GetImagerParameters();

    /**
     * Returns a mapping of imager parameters to their min/max values. This is
     * based on the imager connected to the currently being edited
     * application.
     */
    std::unordered_map<std::string,
    		       std::unordered_map<std::string, std::string> >
    GetImagerParameterLimits();

    /**
     * Returns an `ImagerConfig' instance for the imager configuration of the
     * application currently attached to the XMLRPC server.
     */
    o3d3xx::ImagerConfig::Ptr GetImagerConfig();

    /**
     * Sets parameters on the XMLRPC-attached application's imager based on the
     * passed in `ImagerConfig' pointer.
     */
    void SetImagerConfig(const o3d3xx::ImagerConfig* config);

    //---------------------------------------------
    // XMLRPC: SpatialFilter object
    //---------------------------------------------

    /**
     * Returns a mapping of key/value pairs for the enabled spatial filter for
     * the application that is currently being edited.
     */
    std::unordered_map<std::string, std::string> GetSpatialFilterParameters();

    /**
     * Returns a mapping of spatial filter parameters to their min/max
     * allowable values. This is based on the spatial filter applied to the
     * imager connected to the currently being edited application.
     */
    std::unordered_map<std::string,
		       std::unordered_map<std::string, std::string> >
    GetSpatialFilterParameterLimits();

    /**
     * Returns a `SpatialFilterConfig' instance for the imager configuration of
     * the application currently attached to the XMLRPC server.
     */
    o3d3xx::SpatialFilterConfig::Ptr GetSpatialFilterConfig();

    /**
     * Sets parameters on the XMLRPC-attached imager's spatial filter based on
     * the passed in `SpatialFilterConfig' pointer.
     */
    void SetSpatialFilterConfig(const o3d3xx::SpatialFilterConfig* config);

    //---------------------------------------------
    // XMLRPC: TemporalFilter object
    //---------------------------------------------


    //---------------------------------------------
    // XMLRPC: DeviceConfig object
    //---------------------------------------------

    /**
     * Returns a device config object populated with the current device
     * configuration. This device config object can be used to mutate
     * parameters on the device by calling its mutator functions and then
     * `SetDeviceConfig' and `SaveDevice'. on the camera.
     *
     * @return A shared pointer to a DeviceConfig.
     *
     * @throw o3d3xx::error_t upon error
     */
    o3d3xx::DeviceConfig::Ptr GetDeviceConfig();

    /**
     * Set a password and activate it for the next edit session. This takes the
     * password that is currently set on the camera instnace either by setting
     * it in the ctor or via `SetPassword'.
     *
     * @throw o3d3xx::error_t
     */
    void ActivatePassword();

    /**
     * Disables password protection on the sensor. In order to make this
     * persistent, `SaveDevice' needs to be called.
     *
     * @throw o3d3xx::error_t
     */
    void DisablePassword();

    /**
     * For each mutable parameter in the device configuration, this funtion
     * will set that parameter on the camera. In order to make the changes
     * persistent `SaveDevice' needs to be called.
     *
     * @param[in] config a Pointer to an `o3d3xx::DeviceConfig'
     *
     * @throw o3d3xx::error_t
     */
    void SetDeviceConfig(const o3d3xx::DeviceConfig* config);

    /**
     * Persists any device changes set on the camera during the current edit
     * session.
     *
     * @throw o3d3xx::error_t
     */
    void SaveDevice();

    //---------------------------------------------
    // XMLRPC: NetConfig object
    //---------------------------------------------

    /**
     * Return a key/value hash table of the sensor's current network settings.
     *
     * @return Hash table of network settings
     *
     * @throw o3d3xx::error_t
     */
    std::unordered_map<std::string, std::string> GetNetParameters();

    /**
     * Returns the current network configuration parameters of the sensor
     * wrapped in an `o3d3xx::NetConfig' object. This object can be mutated and
     * then passed to `SetNetConfig' to enabled changes to the hardware.
     *
     * @return A new `NetConfig' object populated with the sensor's current
     * network configuration information.
     *
     * @throw o3d3xx::error_t
     */
    o3d3xx::NetConfig::Ptr GetNetConfig();

    /**
     * Mutates the sensor's network settings based on the passed in `NetConfig'
     * instance.
     *
     * @param[in] config A `NetConfig' pointer whose settings should be
     * sent to the hardware.
     *
     * @throw o3d3xx::error_t
     */
    void SetNetConfig(const o3d3xx::NetConfig* config);

    /**
     * Persists any network settings that were set on the hardware.
     *
     * @throw o3d3xx::error_t
     */
    void SaveNet();

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
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_INVALID_PARAM);
		  break;

		case 100000:
		  LOG(WARNING) << "Invalid session object? "
			       << this->GetSessionID();
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_OBJ_NOT_FOUND);
		  break;

		case 100001:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_METHOD_NOT_FOUND);
		  break;

		case 101002:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_VALUE_OUT_OF_RANGE);
		  break;

		case 101004:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_EDIT_SESSION_ALREADY_ACTIVE);
		  break;

		case 101013:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_INVALID_APPLICATION);
		  break;

		case 101014:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_APPLICATION_IN_EDIT_MODE);
		  break;

		case 101015:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_TOO_MANY_APPLICATIONS);
		  break;

		case 101016:
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_NOT_EDITING_APPLICATION);
		  break;

		default:
		  LOG(ERROR) << "IFM error code: " << ifm_error;
		  throw o3d3xx::error_t(O3D3XX_XMLRPC_FINFAIL);
		  break;
		}
	    }
	  else
	    {
	      throw o3d3xx::error_t(O3D3XX_XMLRPC_FAILURE);
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

    /** _XCall wrapper for XMLRPC calls to the "EditMode" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallEdit(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT;

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

    /** _XCall wrapper for XMLRPC calls to the "Application" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallApp(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT +
	o3d3xx::XMLRPC_APP;

      return this->_XCall(url, sensor_method_name, args...);
    }

    /** _XCall wrapper for XMLRPC calls to the "ImagerConfig" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallImager(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT +
	o3d3xx::XMLRPC_APP +
	o3d3xx::XMLRPC_IMAGER;

      return this->_XCall(url, sensor_method_name, args...);
    }

    /** _XCall wrapper for XMLRPC calls to the "SpatialFilter" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallSpatialFilter(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT +
	o3d3xx::XMLRPC_APP +
	o3d3xx::XMLRPC_IMAGER +
	o3d3xx::XMLRPC_SPATIALFILTER;

      return this->_XCall(url, sensor_method_name, args...);
    }

    /** _XCall wrapper for XMLRPC calls to the "TemporalFilter" object */
    template <typename... Args>
    xmlrpc_c::value const
    _XCallTemporalFilter(const std::string& sensor_method_name, Args... args)
    {
      std::string url =
	this->GetXMLRPCURLPrefix() +
	o3d3xx::XMLRPC_MAIN +
	o3d3xx::XMLRPC_SESSION +
	o3d3xx::XMLRPC_EDIT +
	o3d3xx::XMLRPC_APP +
	o3d3xx::XMLRPC_IMAGER +
	o3d3xx::XMLRPC_TEMPORALFILTER;

      return this->_XCall(url, sensor_method_name, args...);
    }

  }; // end: class Camera

} // end: namespace o3d3xx

#endif // __O3D3XX_CAMERA_H__
