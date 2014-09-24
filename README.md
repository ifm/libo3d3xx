
o3d3xx-toolbox
==============

Driver for IFM Efector O3D3xx Cameras

Driver and utilities for usage with O3D3xx cameras built and sold
by IFM Efector. The O3D3xx cameras are 3D cameras based on the PMD
Tech Photonic Mixer Device time-of-flight imager. This toolbox has been built
with mobile robotics applications in mind. To that end, the toolbox supplies
a library for high-speed streaming of images from the camera written in modern
C++, point cloud constructions, plugability with other frameworks commonly
used by roboticists (e.g., ROS, PCL, Matlab, Python), and simple
parameterization of the camera operating characteristics.


Installation
------------

To build and install o3d3xx you can issue the following commands
from the top-level directory of this source distribution:

	$ mkdir build
	$ cd build
	$ cmake ..
	$ make
	$ make check
	$ sudo make install

There is also a `make package` target that will build the binary deb. After
installation, you may find it useful to add the following to your
`~/.bash_profile`:

	if [ -f /opt/o3d3xx-toolbox/etc/setup.bash ]; then
		source /opt/o3d3xx-toolbox/etc/setup.bash
	fi


Prerequisites
-------------

To build the software, you will need the following third-party software
(expressed as Ubuntu software packages):

* build-essential
* cmake
* libgtest-dev
* libgoogle-glog-dev
* libboost-system-dev (>= 1.46)
* libboost-thread-dev (>= 1.46)
* libboost-program-options-dev (>= 1.46)

Runtime dependencies are encoded into the deb built as a result of
`make package`. Typically, those are a subset of the build dependencies.

TODO
----

Please see the [Github Issues](https://github.com/lovepark/o3d3xx-toolbox/issues).

LICENSE
-------

Please see the file called LICENSE.

AUTHORS
-------

Tom Panzarella <tom@loveparkrobotics.com>

