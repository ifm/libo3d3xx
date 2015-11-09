## Changes between libo3d3xx 0.1.11 and 0.2.0

### Added cross-compilation support

A new directory called `cmake/toolchains` has been added to allow for
setting up a cross-compilation toolchain. Additonally, a convenience `deploy`
target has been added that assumes, you are deploying the deb files created
from the `package` target and that you will deploy via `scp`. These deployment
assumptions will likely need to be revisited to make them more generally
useful, however, in the near-term, lets consider them a proof-of-concept for
automating the deployment of cross-compiled code.

### Device config default values changed

Changed default for the ServiceReportFailedBuffer and ServiceReportPassedBuffer
default values from 15 to 5. For reasoning, please see:

https://github.com/lovepark/libo3d3xx/issues/20

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

See also:

* https://github.com/lovepark/libo3d3xx/issues/13

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
