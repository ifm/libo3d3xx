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

#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <o3d3xx/util.hpp>

namespace po = boost::program_options;

namespace o3d3xx
{
  /**
   * Thin wrapper on top of boost::program_options that packages up the command
   * line arguments used by the o3d3xx library command line tools. This class
   * is desgined for convenience.
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

    /**
     * Parses the defined command line options and also processes common
     * options like `--help' and `--version'. In the case of processing common
     * options it will write messages to the `out' argument.
     *
     * @param[in] argc Argument count (should come right from `main')
     * @param[in] argv Argument vector (should come right from `main')
     * @param[out] ip Pointer to a string to fill with the `--ip' arg
     * @param[out] xmlrpc_port Pointer to a uint32_t to fill with the
     *             `--xmlrpc-port' arg
     * @param[in] fn A function that should be run after parsing the common
     *               arguments but prior to returning. It should be noted that
     *               this function will only run if the return value of this
     *               function will be non-zero. Additionally, exceptions from
     *               this passed in function will be propogated.
     * @param[in] out Output stream to write messages to, typically `std::cout'
     *
     * @return zero if the program should exit upon return (typically becuase
     * the user passed in `--help' or `--version' or `> 0' if the program
     * should continue processing.
     */
    int Parse(int argc, const char **argv,
	      std::string *ip = nullptr,
	      std::uint32_t *xmlrpc_port = nullptr,
	      std::function<void()> fn = [](){ o3d3xx::Logging::Init(); },
	      std::ostream& out = std::cout);

    // provides raw acess to boost::program_options structures
    po::variables_map vm;
    po::options_description visible;

  }; // end: class CmdLineOpts

} // end: namespace o3d3xx

#endif // __O3D3XX_CMDLINE_OPTS_H__
