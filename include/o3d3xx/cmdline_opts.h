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
#ifndef __O3D3XX_CMDLINE_OPTS_H__
#define __O3D3XX_CMDLINE_OPTS_H__

#include <string>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace o3d3xx
{
  /**
   * Thin wrapper on top of boost::program_options that packages up the command
   * line arguments used by the o3d3xx library command line tools.
   */
  class CmdLineOpts
  {
  public:
    CmdLineOpts(const std::string& description);
    virtual ~CmdLineOpts() = default;

    // copy and move semantics
    CmdLineOpts(CmdLineOpts&&) = delete;
    CmdLineOpts& operator=(CmdLineOpts&&) = delete;
    CmdLineOpts(CmdLineOpts&) = delete;
    CmdLineOpts& operator=(const CmdLineOpts&) = delete;

    void Parse(int argc, const char **argv);

    // provides raw acess to boost::program_options structures
    po::variables_map vm;
    po::options_description visible;

  }; // end: class CmdLineOpts

} // end: namespace o3d3xx

#endif // __O3D3XX_CMDLINE_OPTS_H__
