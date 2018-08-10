## Changes between libo3d3xx 0.7.5 and 0.7.6

* Updated cmake/packaging scripts to support the Ubuntu 18.04 toolchain

## Changes between libo3d3xx 0.7.4 and 0.7.5

* Added timestamp support for images grabbed with the oem module

## Changes between libo3d3xx 0.7.3 and 0.7.4

* Updated XMLRPC protocol for 1.20.x firmwares
* Made framegrabber robust to exhausted PCIC connections and async messages.

## Changes between libo3d3xx 0.7.2 and 0.7.3

* Merged @graugans PR120: o3d3xx-trace

## Changes between libo3d3xx 0.7.1 and 0.7.2

* Merged @graugans PR117: Add support for the result data time stamp

## Changes between libo3d3xx 0.6.0 and 0.7.1

* Merged @stefan-kuhn PR101: Deadlock bug fix

## Changes between libo3d3xx 0.6.0 and 0.7.0

* Merged @stefan-kuhn PR95: Pcicclient async messages integration
* Merged @stefan-kuhn PR96: Integrate illu temp

## Changes between libo3d3xx 0.5.0 and 0.6.0

* Merged @cfreundl PR89: Integration of resultsync 0.2
* Merged @stefan-kuhn PR91: Implementation of PCIC client

## Changes between libo3d3xx 0.4.9 and 0.5.0

* Introduced the `oem` module to enable optimized functionality for OEM's
  developing on-camera algorithms.
* Integrated ifm's Results Synchronization Library with the `oem` module to
  allow for (near) zero-copy framegrabbing. There is still a single copy that
  needs to be made to pull data out of the ifm buffers and populate the OpenCV
  data structures.
* Added a new command-line program (as part of the oem module) called
  `o3d3xx-oem-jitter`. Can be used to collect data on the resultsync-based
  framegrabber's latency jitter.
* Brought library compatibility up-to-date with ifm 1.6.2114 firmware. NOTE:
  0.4.9 and 0.5.0 are ahead of the public ifm firmwares, so, I am disabling the
  unit tests for setting temporary application parameters and registering the
  exposure times to the image buffer. These will be re-enabled with a 1.8.xxx
  version of the ifm firmware.

## Changes between libo3d3xx 0.4.8 and 0.4.9

* Added new device config parameter `EthernetFieldBusEndianness`
* Added new imager config parameter `UseSimpleBinning`
* Added new program `ex-cartesian_compare.cpp` in the *examples* module. This
  is to allow for checking, visually, an off-board computed cloud vs. an
  on-board computed cloud.
* Fixed memory access [bug](https://github.com/lovepark/libo3d3xx/issues/67)
* Updated to support (in a backward compatible fashion) image chunk header
  version 2.
* Framegrabber now supports streaming back exposure times in-line with the
  pixel data. See `o3d3xx-schema` for new schema mask and the JSON sent over
  PCIC to the camera.
* ImageBuffer allows for accessing exposure times registered to the current
  image frame (assuming the schema mask set on the `FrameGrabber` specifies the
  streaming of exposure times).
* `Camera` now provides an interface for setting `ExposureTime` and
  `ExposureTimeRatio` on the fly while streaming in pixel data.
* A new program `ex-exposure_times.cpp` in the `examples` module has been added
  to illustrate setting on-the-fly exposure times.

## Changes between libo3d3xx 0.4.6 and 0.4.8

* NOTE: 0.4.7 was a testing branch and never made it to a release.
* FrameGrabber ctor will not throw exceptions explicitly. This is achived by,
  1) not introspecting the current trigger mode, 2) not explicitly setting
  the camera into `RUN` mode, 3) Not throwing an error if we cannot discern the
  PCIC TCP port. The intent is to make the software more stable to volatility
  in the hardware being present or not (be it at start up or during
  operation).
* Software Triggering while in `FREE_RUN` mode will not cause the frame grabber
  to throw and exception.

## Changes between libo3d3xx 0.4.5 and 0.4.6

* Updates to XML-RPC protocol for FW 1.6.x
* Provided mutators on the DeviceConfig value object so that when fetching data
  from the camera (e.g., in o3d3xx-dump), it will reflect the true state of the
  camera. See: https://github.com/lovepark/libo3d3xx/issues/63
* Removed tests against the 100k pixel imager -- this firmware seems to
  deprecate this feature.

## Changes between libo3d3xx 0.4.4 and 0.4.5

* Added support for multiple PCIC connections under software triggering. See
  also: https://github.com/lovepark/libo3d3xx/issues/58
* Adapted build scripts to be able to cross-compile to the camera

## Changes between libo3d3xx 0.4.3 and 0.4.4

* Firmware files moved to new `o3d3xx-firmware` repository
* Deb files have been renamed to include ubuntu version string in filename
* FrameGrabber has been refactored and now supports S/W triggering
* o3d3xx-hz now supports a `--sw` argument indicating that you want it to
  do a S/W trigger prior to fetching the image data -- if you set up your
  applications correctly on the camera, you can benchmark a free-running camera
  vs. a software triggering setup.

## Changes between libo3d3xx 0.4.2 and 0.4.3

* Updated build scripts to support Ubuntu 16.04 LTS
* Updated build scripts to deal with parallel installations of
  OpenCV 3 and OpenCV 2.4. This is important for our ROS users.

## Changes between libo3d3xx 0.4.1 and 0.4.2

* Updated unit tests to properly validate Extrinsics

## Changes between libo3d3xx 0.4.0 and 0.4.1

* Updates to XML-RPC protocol for FW 1.4.x. See JSON schema diff in
  `docs/json`.

## Changes between libo3d3xx 0.3.0 and 0.4.0

### Modularization of libo3d3xx

* The project has been broken up into three separate sub-projects or
  *modules*. They are:
  * libo3d3xx-camera: Implements the XML-RPC protocol
  * libo3d3xx-framegrabber: Streams image data from camera
  * libo3d3xx-image: Organizes the image data utilizing OpenCV and PCL data
    structures.

* The main README page contains updated build instructions for the new code
  structure.

### ByteBuffer interface

* Introduced a new ByteBuffer interface. This is a class intended to be
  subclassed and would allow developers to use only the `camera` and
  `framegrabber` modules and supply their own image/cloud data structures. This
  effectively makes the usage of OpenCV and PCL optional. ByteBuffer is bundled
  as part of the `framegrabber` module.

### Examples

* Added a pseudo-module called `examples` as a place to collect simple programs
  that demonstrate a particular concept. Initial concrete examples include:
  ex-100k, ex-cartesian, and ex-file_io. The README in the `examples` directory
  for more information.

### Deprecations

* Removed `hist1` function from `util.hpp`. This has already been taken out of
  usage by `o3d3xx-ros` which to the best of my knowledge, was the only
  consumer of this interface. The function is simple enough to implement for
  users that need it.

* Removed some old documentation that no longer applies: `custom_builds` and
  `using`. They have been moved to `doc/attic`.

## Changes between libo3d3xx 0.2.0 and 0.3.0

### XML-RPC Protocol

* Compatible to the 1.3.x IFM firmware release.

### CMAKE_INSTALL_PREFIX

* The default install location is now `/usr`. This removes the need for setting
  `LD_LIBRARY_PATH` and having a `setup.bash` file.

### Custom/Pluggable PCIC Schemas

* The `FrameGrabber` now supports parsing return data from custom schemas based
  on a _mask_. This feature has been implemented so that user of low bandwidth
  or noisy (e.g., WiFi) networks can configure the camera to return only the
  data they need and to not consume unnecessary bandwith. The aforementioned
  goal is only partially achieved with this feature. See the addition of the
  `Extrinsics()` and `UnitVectors()` calls below to complete this feature.

* The introduction of this feature also brought about the introduction of a new
  environment variable called `O3D3XX_MASK` which can be used to set the schema
  mask manually. For example: `$ O3D3XX_MASK=15 o3d3xx-viewer` The value of the
  variable is interrpeted as a `uint16_t`. If the passed in value causes an error
  during conversion, the default mask is used -- i.e., the mask returned by
  running `o3d3xx-schema` with no arguments.

### ImageBuffer

* Has a new method `Extrinsics()` to return the camera extrinsics as reported
  by the O3D. This is provided to assist in reconstructing the Cartesian
  data from the radial distance image and the unit vectors.

* Has a new method `UnitVectors()` to return the rotated unit vectors that can
  be used, together with the radial distance image and extrinsics, to compute
  the cartesian data off-board the camera. An example of effectively using this
  can be found in the `ImageBuffers_Tests.ComputeCartesian` unit test in
  `o3d3xx-image-tests.cpp`.

### o3d3xx-schema

* New tool used for testing and building custom PCIC schemas

### o3d3xx-reboot

* Added the `-r` flag to allow for rebooting the sensor into recovery
  mode. This is useful for updating the sensor firmware w/o having to boot
  Windows.

### ImagerConfig

* Support for `EnableFastFrequency`, `Resolution`, and `ClippingCuboid`
  parameters

### DeviceConfig

* Changed default for the ServiceReportFailedBuffer and
  ServiceReportPassedBuffer default values from 5 to 15. This is an un-doing of
  the change from 0.1.11 to 0.2.0 (see below), but the new 1.2.x firmware
  restores these old defaults so we will do the same here.

* Support for `EvaluationFinishedMinHoldTime` and
  `SaveRestoreStatsOnApplSwitch` parameters.

### 100K Imager

* As of the 1.2.x firmware, 100K images are officially supported by IFM. So, we
  have implemented that support into this library and have removed the backdoor
  methods for doing it.

## Changes between libo3d3xx 0.1.11 and 0.2.0

### Added cross-compilation support

A new directory called `cmake/toolchains` has been added to allow for
setting up a cross-compilation toolchain. Additonally, a convenience `deploy`
target has been added that assumes, you are deploying the deb files created
from the `package` target and that you will deploy via `scp`. These deployment
assumptions will likely need to be revisited to make them more generally
useful, however, in the near-term, lets consider them a proof-of-concept for
automating the deployment of cross-compiled code. The following cmake
variables can be set on the command line to control how the deploy occurs:

* TARGET\_IP    (ip address of your embedded system)
* TARGET\_USER  (user to login to the embedded system as)
* TARGET\_DIR   (directory on the embedded system to copy the debs to)

Currently cross compiling makes the assumption you are deploying to an embedded
Linux system (i.e., not cross compiling for Windows or Mac) and to that end,
turns off the building of `o3d3xx-viewer`.

It should also be noted that when cross compiling, unit tests are still built
(by default) but you cannot run `make check`. You can deploy the code and run
the unit tests as an executeable on the embedded system.

Initial cross-compilers tested include Linaro and ELDK from Denx.

### Refactored and Modularized Build Process

Options to conditionally build parts of the code have been added:

* BUILD_TESTS
* BUILD_VIEWER
* BUILD\_SHARED\_LIBS
* BUILD\_STATIC\_LIBS
* BUILD\_EXE\_VIEWER
* BUILD\_EXE\_VERSION
* BUILD\_EXE\_RESET
* BUILD\_EXE\_LS
* BUILD\_EXE\_DUMP
* BUILD\_EXE\_CONFIG
* BUILD\_EXE\_RM
* BUILD\_EXE\_REBOOT
* BUILD\_EXE\_HZ
* BUILD\_EXE\_IMAGER\_TYPES
* BUILD\_EXE\_IFM\_IMPORT
* BUILD\_EXE\_IFM\_EXPORT

These are all turned ON by default but can be switched off on the cmake command
line. For example: `$ cmake -DBUILD_TESTS=OFF ..`

Static libraries (in addition to the shared libraries) are being built by
default. Provisions have been made to compile the library code once then link
twice (shared and static).

See also:

* https://github.com/lovepark/libo3d3xx/issues/13

### Raw Amplitude

The raw amplitude data is now available on the `ImageBuffer` object via the
`RawAmplitudeImage()` accessor. This is the non-normalized (wrt the exposure
time) amplitude data.

### o3d3xx-viewer

The bottom right image on the 2D image window has the raw amplitude image
replace what was historically the amplitude histogram.

### FromJSON()

The `o3d3xx::Camera::FromJSON(...)` function is fundamental to how tools like
`o3d3xx-config` work. This function, including the ones on which it depends
(namely the `FromJSON()` calls on the `XXXConfig` value objects), has been
overhauled in this release. The idea was to allow for better "one-liners"
(i.e., partial JSON configurations) to be specified on the command line.

See also:

* https://github.com/lovepark/libo3d3xx/issues/22#issuecomment-155788906

### Device config default values changed

Changed default for the ServiceReportFailedBuffer and ServiceReportPassedBuffer
default values from 15 to 5. For reasoning, please see:

* https://github.com/lovepark/libo3d3xx/issues/20

### Setting NetConfig tries to avoid unnecessary reboots

The handling of setting the camera's network configuration has been modified to
try to avoid reboots where possible. For example, please see:

* https://github.com/lovepark/libo3d3xx/issues/23

## Changes between libo3d3xx 0.1.10 and 0.1.11

### ImageBuffer

Fixed a bug in how the OpenCV image encoding was specified for the `XYZImage`.

## Changes between libo3d3xx 0.1.9 and 0.1.10

### ImageBuffer now makes point cloud data available as cv::Mat

A new `XYZImage` is avialable on the `ImageBuffer` class via the `XYZImage()`
accessor. This image is a 3-channel OpenCV image encoding of the point cloud
where the three channels are spatial planes (x, y, z). It should be noted that
while this encoding of the point cloud contains the same data as the PCL encoded
point cloud the data are kept in mm (as opposed to meters) and the data
type is int16\_t as opposed to float. This was motivated by efficiency needs on
embedded systems. However, the coord frame for this point cloud data is
consistent with the coord frame on the PCL point cloud.

See also:

* https://github.com/lovepark/libo3d3xx/issues/17
* https://github.com/lovepark/libo3d3xx/issues/14#issuecomment-144391800
* https://github.com/lovepark/libo3d3xx/issues/14#issuecomment-144548199

## Changes between libo3d3xx 0.1.8 and 0.1.9

### Updates to schema-aware FrameGrabber

The 0.1.8 frame grabber had issues when run in connection with the ROS code in
`o3d3xx-ros`. Retrieving, mutating, and restoring the TCP result schema via the
XML-RPC interface proved to be fragile. Per suggestion by @graugans (IFM) we
now use the `c` command on the PCIC interface.

See comments/discussion at the end of [this commit](https://github.com/lovepark/libo3d3xx/commit/ba9407a67adc71d85e53aca3072601dd2bc64385)

### o3d3xx::Camera::FromJSON(..)

The implementation of FromJSON was very fragile due to how 0.1.8 made changes
to JSON comparisons. This fragility caused problems with heavily used
command line tools like `o3d3xx-config`. This release has better error trapping
while maintaining the network efficiency intentions of the 0.1.8 FromJSON(..)
implementation.


## Changes between libo3d3xx 0.1.7 and 0.1.8

### Schema-aware FrameGrabber

https://github.com/lovepark/libo3d3xx/issues/4

Per suggestion by @graugans (IFM), the `FrameGrabber` will now apply a known
schema at runtime in the event the configured schema for the active application
is different than what is expected. Additionally, the configured schema gets
restored once the FrameGrabber is destroyed. This allows for compatibility with
other computing environments that may expect the configured schema to be
returned.

### Interoperability with IFM's Vision Assistant

IFM's software provides the means to import/export applications to/from the
camera where the files have an IFM-specific serialization (zip-compressed JSON
as of this writing). `o3d3xx::Camera` now has an `ImportIFMApp` and
`ExportIFMApp` method to enable passing data to/from the camera in a way that
is compatible with Vision Assistant. Additionally, two new command line tools
have been provided (`o3d3xx-ifm-import` and `o3d3xx-ifm-export`) that exploits
the new camera methods to move data to/from files employing the IFM
serialization scheme.

### Initial support for 100K pixel images

Unlocking the 100K pixel support on the O3D303 is not yet (as of this writing)
officially supported by IFM and you should only use this feature at your own
risk. However, if you like to live on the edge, you can play with this feature
now. We have provided an application, `test/data/100k.o3d3xxapp`, that unlocks
this feature into a new application on your camera. You can import this
application using `o3d3xx-ifm-import` and then mark it active using
`o3d3xx-config`.


## The initial release of libo3d3xx was 0.1.7
