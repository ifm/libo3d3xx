
libo3d3xx
=========

Library and utilities for working with IFM Efector O3D3xx Cameras.

![3dimg](doc/figures/3d.png)

libo3d3xx provides facilities for interfacing with O3D3xx cameras built and
sold by IFM Efector. The O3D3xx cameras are 3D cameras based on the PMD
Tech Photonic Mixer Device time-of-flight imager. This toolbox bridges the IFM
hardware to the open-source computer vision packages:
[OpenCV](http://opencv.org) and [PCL](http://pointclouds.org).

At its core, libo3d3xx provides a way to stream images from an O3D3xx camera in
real-time and access the 2D data as OpenCV images (applies to the Amplitude,
Depth, and Confidence images) and the 3D data as PCL point clouds (i.e., the
`CARTESIAN_X`, `CARTESIAN_Y`, and `CARTESIAN_Z` are fused to create a point
cloud). We note that the PCL point cloud constructed by this library has point
type `pcl::PointXYZI` (referred to as `o3d3xx::PointT`). For the intensity
channel, we use the Amplitude image data registered to each point. We do this
because, unlike in earlier PMD-based IFM cameras, the Intensity image is not
currently available. We expect the Amplitude image can act as a proxy for the
Intensity image as it relates to PCL algorithms that may rely on that data.

The code has been developed on 64-bit Ubuntu Linux 14.04 LTS. This is currently
the only platform that the software has been tested on. It is expected that
some tweaks will need to be made to use the software on other platforms. You
can contact [Love Park Robotics](http://loveparkrobotics.com) for assistance in
porting or file an [issue request](https://github.com/lovepark/libo3d3xx/issues)

Software Compatibility Matrix
-----------------------------
<table>
  <tr>
    <th>libo3d3xx version</th>
    <th>IFM Firmware Version</th>
    <th>Supported Cameras</th>
  </tr>
  <tr>
    <td>0.1.7</td>
    <td>0.06.13</td>
    <td>O3D303</td>
  </tr>
  <tr>
    <td>0.1.8</td>
    <td>0.06.39</td>
    <td>O3D303</td>
  </tr>
</table>

Features
--------

High-level features of this library include:

* The code is written in modern C++11.
* The library employs PCL and OpenCV native image formats.
* Easily scriptable command line utilities are provided for performing common
  tasks associated with configuring, backing up, restoring, and introspecting,
  the camera settings. This scriptability lends itself to managing fleets of
  cameras for large-scale installations.
* A simple viewer application is provided for concurrently inspecting the point
  cloud, depth, amplitude, and confidence image.
* A business-friendly (Apache 2.0) License is employed.
* The code is being actively developed and maintained at
  [Love Park Robotics](http://loveparkrobotics.com). Pull requests are welcome
  from those who wish to contribute code!

[ROS](http://ros.org) bindings are available [here](https://github.com/lovepark/o3d3xx-ros)

Prerequisites
-------------

* [Boost](http://www.boost.org) (>= 1.54)
* [Gtest](https://code.google.com/p/googletest/) (unit testing)
* [Glog](https://code.google.com/p/google-glog/) (logging infrastructure)
* [libxmlrpc](http://xmlrpc-c.sourceforge.net/)
* [OpenCV](http://opencv.org) (>= 2.4)
* [PCL](http://pointclouds.org) (>= 1.7.1)
* [CMake](http://www.cmake.org) (>= 2.8.11)

Additionally, your compiler must support C++11. We are using g++ 4.8.2 on
Ubuntu Linux 14.04 LTS.

Installation
------------

(Assumes Linux)

To build and install `libo3d3xx` you can issue the following commands
from the top-level directory of this source distribution:

	$ mkdir build
	$ cd build
	$ cmake ..
	$ make
	$ make check
	$ sudo make install

NOTE: Running `make check` will run unit tests. It assumes that the hardware is
available and running with factory default settings. If your camera is using an
IP address different from the factory default of 192.168.0.69, you can set the
`O3D3XX_IP` environment variable to point to the proper IP. Please note, the unit
tests will wipe out your current camera settings and applications. You should
back up your camera prior to running the unit tests.

There is also a `make package` target that will build the binary debian
package. You can then install this with the usual Debian/Ubuntu `dpkg`
tool.

After installation, you may find it useful to add the following to your
`~/.bash_profile`:

	if [ -f /opt/libo3d3xx/etc/setup.bash ]; then
		source /opt/libo3d3xx/etc/setup.bash
	fi

This will modify your `LD_LIBRARY_PATH` and `PATH` to ensure the library is
available to your environment.

Running
-------

* [Using the library](doc/using.md)
* [Command-line utilities](doc/utils.md)

TODO
----

Please see the [Github Issues](https://github.com/lovepark/libo3d3xx/issues).

LICENSE
-------

Please see the file called LICENSE.

AUTHORS
-------

Tom Panzarella <tom@loveparkrobotics.com>
