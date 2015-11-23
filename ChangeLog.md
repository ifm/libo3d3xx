## Changes between libo3d3xx 0.2.0 and 0.3.0

**IMPORTANT:** This release breaks both API and ABI compatibility with 0.2.0.

### Custom/Pluggable PCIC Schemas

The `FrameGrabber` now supports parsing return data from custom schemas based
on a _mask_. This (currently experimental) feature has been implemented so that
user of low bandwidth or noisy (e.g., WiFi) networks can configure the camera
to return only the data they need and to not consume unnecessary bandwith. As
of this writing, the aforementioned goal is only partially achieved with the
current code. This feature is still under active development.

The introduction of this feature also brought about the introduction of a new
environment variable called `O3D3XX_MASK` which can be used to set the schema
mask manually. For example: `$ O3D3XX_MASK=15 o3d3xx-viewer` The value of the
variable is interrpeted as a `uint16_t`. If the passed in value causes an error
during conversion, the default mask is used -- i.e., the mask returned by
running `o3d3xx-schema` with no arguments.

### Raw Amplitude

Now with the support of pluggable schemas, the raw amplitude is no longer part
of the default schema. This can be turned on by users on a case-by-case basis.

### o3d3xx-viewer

DOCUMENT THE CHANGES HERE

### o3d3xx-schema

* New tool used for testing and building custom PCIC schemas

### o3d3xx-reboot

* Added the `-r` flag to allow for rebooting the sensor into recovery
  mode. This is useful for updating the sensor firmware w/o having to boot
  Windows.

### ImagerConfig

* Support for `EnableFastFrequency` and `Output100K` parameters

### DeviceConfig

* Changed default for the ServiceReportFailedBuffer and ServiceReportPassedBuffer
  default values from 5 to 15. This is an un-doing of the change from 0.1.11
  to 0.2.0 (see below), but the new 1.2.x firmware restores these old defaults
  so we will do the same here.

### 100K Imager

As of the 1.2.x firmware, 100K images are officially supported by IFM. So, we
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
