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

#include "o3d3xx/net_config.h"
#include <exception>
#include <functional>
#include <string>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <glog/logging.h>
#include "o3d3xx/camera.hpp"
#include "o3d3xx/util.hpp"

o3d3xx::NetConfig::NetConfig()
  : mac_address_(""),
    network_speed_(0),
    ip_(o3d3xx::DEFAULT_IP),
    gw_(o3d3xx::DEFAULT_GW),
    subnet_(o3d3xx::DEFAULT_SUBNET),
    use_dhcp_(false)
{ }

o3d3xx::NetConfig::NetConfig(
  const std::unordered_map<std::string, std::string>& params)
  : NetConfig()
{
  for (auto& kv : params)
    {
      try
        {
          auto func = o3d3xx::NetConfig::mutator_map.at(kv.first);
          func(this, kv.second);
        }
      catch (const std::out_of_range& ex)
        {
          // we expect this for the read-only params
        }
      catch (const std::invalid_argument& ia)
        {
          LOG(ERROR) << "Invalid arg for: "
                     << kv.first << "=" << kv.second;
        }
    }
}

std::string
o3d3xx::NetConfig::MACAddress() const noexcept
{
  return this->mac_address_;
}

int
o3d3xx::NetConfig::NetworkSpeed() const noexcept
{
  return this->network_speed_;
}

std::string
o3d3xx::NetConfig::StaticIPv4Address() const noexcept
{
  return this->ip_;
}

void
o3d3xx::NetConfig::SetStaticIPv4Address(const std::string& ip) noexcept
{
  this->ip_ = ip;
}

std::string
o3d3xx::NetConfig::StaticIPv4Gateway() const noexcept
{
  return this->gw_;
}

void
o3d3xx::NetConfig::SetStaticIPv4Gateway(const std::string& gw) noexcept
{
  this->gw_ = gw;
}

std::string
o3d3xx::NetConfig::StaticIPv4SubNetMask() const noexcept
{
  return this->subnet_;
}

void
o3d3xx::NetConfig::SetStaticIPv4SubNetMask(const std::string& mask) noexcept
{
  this->subnet_ = mask;
}

bool
o3d3xx::NetConfig::UseDHCP() const noexcept
{
  return this->use_dhcp_;
}

void
o3d3xx::NetConfig::SetUseDHCP(bool on) noexcept
{
  this->use_dhcp_ = on;
}

const std::unordered_map<std::string,
                         std::function<void(o3d3xx::NetConfig*,
                                            const std::string&)> >
o3d3xx::NetConfig::mutator_map =
  {
    {"StaticIPv4Address",
     [](o3d3xx::NetConfig* net, const std::string& val)
     { net->SetStaticIPv4Address(val); } },

    {"StaticIPv4Gateway",
     [](o3d3xx::NetConfig* net, const std::string& val)
     { net->SetStaticIPv4Gateway(val); } },

    {"StaticIPv4SubNetMask",
     [](o3d3xx::NetConfig* net, const std::string& val)
     { net->SetStaticIPv4SubNetMask(val); } },

    {"UseDHCP",
     [](o3d3xx::NetConfig* net, const std::string& val)
     { net->SetUseDHCP(o3d3xx::stob(val)); } },
  };

std::string
o3d3xx::NetConfig::ToJSON() const
{
  boost::property_tree::ptree pt;

  pt.put("MACAddress", this->MACAddress());
  pt.put("NetworkSpeed", this->NetworkSpeed());
  pt.put("StaticIPv4Address", this->StaticIPv4Address());
  pt.put("StaticIPv4SubNetMask", this->StaticIPv4SubNetMask());
  pt.put("StaticIPv4Gateway", this->StaticIPv4Gateway());
  pt.put("UseDHCP", this->UseDHCP());

  std::ostringstream buf;
  boost::property_tree::write_json(buf, pt);
  return buf.str();
}

o3d3xx::NetConfig::Ptr
o3d3xx::NetConfig::FromJSON(const std::string& json)
{
  o3d3xx::NetConfig::Ptr net =
    o3d3xx::NetConfig::Ptr(new o3d3xx::NetConfig());

  boost::property_tree::ptree pt;
  std::istringstream is(json);
  boost::property_tree::read_json(is, pt);

  for (auto& kv : pt)
    {
      try
        {
          auto func = o3d3xx::NetConfig::mutator_map.at(kv.first);
          func(net.get(), kv.second.data());
        }
      catch (const std::out_of_range& ex)
        {
          DLOG(WARNING) << "In FromJSON: "
                        << kv.first << "=" << kv.second.data()
                        << ": " << ex.what();
        }
    }

  return net;
}
