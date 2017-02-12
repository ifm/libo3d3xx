
libo3d3xx: oem module
=====================

The `oem` module provides specialized code to be used by OEM's whose intention
is to deploy an algorithm/code to be hosted directly upon the O3D303
camera. Almost all users, including those OEM's who will be deploying code to
the camera, have little/no reason to build the software in this module. That is
because the headers/libraries will be included in the Poky SDK rootfs and the
runtime libraries will already be installed on the camera. To that end, these
instructions below are for developers who are working on the `oem` module code
itself.

Features
--------

* Integrates directly with ifm's `libresultsync` to enable lower-latency
  framegrabbing when running locally on the camera

Building and Installing the Software
------------------------------------

Before starting, we assume you will be building this code using the
ifm-supplied Poky SDK that has the core `libo3d3xx` modules installed on the
rootfs. Namely `camera`, `framegrabber`, and `image`. We also asume your Poky
SDK is located at: `/opt/poky/1.8.1/`.

Next, we assume you are running in development mode and will be deploying your
development snapshots of the `oem` module to an oem partition on the device. To
that end, our build scripts provide some convenience Makefile targets for
deploying the code to the oem partition on the camera and also removing
it. Those will be explained (along with their caveats) in-line below.

Assuming you are located in `${LIBO3D3XX_SRC_DIR}/modules/oem`, to build the
software:

```
    $ source /opt/poky/1.8.1/environment-setup-armv7ahf-vfp-neon-poky-linux-gnueabi
    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_INSTALL_PREFIX=/opt/oem/usr \
            -DCPACK_INSTALL_PREFIX=/opt/oem/usr \
            -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchains/o3d303.cmake ..
    $ make
```

At this point the software is built. We note that using our `cmake` line above,
when we build our deployment package (below) this will include the unit tests
but not include development headers. If you would like to *exclude* the unit
tests you can specify `-DBUILD_TESTS=OFF`. If you would like to *include* the
development headers, you can specify `-DINSTALL_HEADERS=ON`.

Now that the software is built, we want to create a deployment package and
install the software on the camera. This can be done with the following:

```
    $ make package
    $ make deploy
```

The `deploy` target above assumes that your camera is at `192.168.0.69` (the
default for the device), you are logging into the device as the user `oem`, and
your SSH private key is located at `~/.ssh/id_rsa.oem`. To change these you can
specify `-DTARGET_IP=<ip>`, `-DTARGET_USER=<user>`, and `-DSSH_KEY=</path/key>`
respectively.

At this point, the code has been installed on the camera. You can now log in to
the camera and run the unit tests. Using the same assumed credentials as above,
that can be done like:

```
    $ ssh -i ~/.ssh/id_rsa.oem oem@192.168.0.69
```

The next set of commands assume you now have a shell on the camera

```
    $ . ../etc/o3d3xx_oem/setup.bash
    $ o3d3xx-oem-tests
```

If you would like to clean up your OEM partition, you can execute the following
(from the `build` subdirectory on your development machine):

```
    $ make uninstall
```


Using the alternate framegrabber
--------------------------------

The OEM framegrabber uses the same idioms as the tcp framegrabber. To utilize
this alternate implementation and access the pixel data, a typical pattern will
look like:

```
    auto cam = std::make_shared<o3d3xx::Camera>();
    auto fg = std::make_shared<o3d3xx::oem::FrameGrabber>(cam);
    auto im = std::make_shared<o3d3xx::oem::ImageBuffer>();

    if(! fg->WaitForFrame(im.get(), 1000))
      {
        // timeout
      }

    cv::Mat cloud = im->XYZImage();
    // do something with the pixel data
```

A more complete example is available by reviewing the source code to the
provided [o3d3xx-oem-jitter](src/bin/o3d3xx-oem-jitter.cpp) command line
utility.
