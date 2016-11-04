
libo3d3xx Examples
==================

This directory contains example programs that utilize `libo3d3xx`. The
intention is to create standalone programs that illustrate one very specific
concept in order to serve the purpose of letting developers ramp up quickly
with using the library. The build infrastructure in this directory is minimal
and the programs are intended to be run in place. Additonally, unless
specifically stated otherwise, things like performance and robust error
handling are not demonstrated. The purpose is to clearly illustrate the task
without clouding it with the details of real-world software engineering --
unless, of course, that was the point of the example.

It is expected that this library of examples will grow over time in response to
common themes we see on the issue tracker.

Prerequisites
-------------

It is assumed you have installed the camera, framegrabber, and image modules
from this project before you proceed.

Building the examples
----------------------

Assuming you are starting from the top-level directory of this source
distribution:

    $ cd modules/examples
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

What is included?
-----------------

* [Image Module](../image) If you want to write a custom implementation of the
  framegrabber's `ByteBuffer` interface, i.e., to use an image or point
  cloud data structure other than OpenCV or PCL, you should look at this
  project's `image` module. It implements the `ByteBuffer` interface and makes
  the data available to the user employing OpenCV and PCL data structures. You
  would follow this as a guide but, of course, populate your chosen data
  structures with the parsed out bytes from the camera.

* [ex-100k](ex-100k.cpp) Shows how to use the API to create a new application,
  set its resolution to 100K pixels, and stream data in that mode. We note that
  you really should use `o3d3xx-dump` and `o3d3xx-config` to create an
  application that employs the 100k imager, set it as active, and simply stream
  data from it rather than using the API to manipulate the applications on the
  camera. However, we have gotten enough requests for this, that we are going
  to show how to do it.

* [ex-cartesian](ex-cartesian.cpp) Shows how to compute the Cartesian data from
  the unit vectors, extrinsics, and radial distance image.

* [ex-cartesian_compare](ex-cartesian_compare.cpp) Just like above, except we
  write out the point cloud as computed by the camera to a PCD file and the one
  we computed to a PCD file so that we can compare them visually using
  `pcl_viewer`.

* [ex-file_io](ex-file_io.cpp) Shows how to capture data from the camera and
  write the images to disk. In this example, the amplitude and radial distance
  image are written out as PNG files and the point cloud is written as a PCD.

* [ex-framegrabber_recycling](ex-framegrabber_recycling.cpp) Shows how to make
  your framegrabbing robust to the camera "disappearing" (e.g., due to power
  cycling, lost network, etc.).

* [ex-exposure_times](ex-exposure_times.cpp) Shows how to change imager
  exposure times on the fly while streaming in pixel data and validating the
  setting of the exposure times registered to the frame data.
