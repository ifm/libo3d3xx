
Cross-compiling libo3d3xx
=========================

As of the `0.2.0` release, `libo3d3xx` has support for cross-compiling to
different architectures (e.g., ARM). However, the current cross-compiliation
support makes some general assumptions that the `target` is an embedded Linux
device and provides some further conveniences if that embedded Linux target is
a Debian or Ubuntu system. A future version of the software may generalize this
approach for cross-compiliation to targets like Mac OS X or Windows.

This document will describe, in reasonable detail, the process of cross
compiling `libo3d3xx` for an embedded Linux system running Ubuntu 14.04 on an
ARM hard-float system. At a very high level, the basic idea is that you will be
building the software from an x86 Linux laptop (for example) but deploying the
built binaries to the embedded target computer where it is intended to run.

Target Dependencies
-------------------

Before starting the build process, you will need to ensure that `libo3d3xx`
dependencies for the target architecture are available to your build process on
your laptop. The most reliable way of doing this is to have access to the
target device's root filesystem available to your compiler and linker. One way
to handle this is to create a _raw image_ file of the target's root filesystem
and copy that file to your laptop (if it is large, it could be located on an
external drive). This can be done by booting your device via a _live disk_
(e.g., on a USB drive, sdcard, or some other way) and then using the Linux `dd`
tool to create the image file. E.g., `dd if=/dev/sda of=/mnt/disk/ssd.dd
bs=4096`. Creating this image file will depend on the specifics of your target
device. The remainder of this document will assume we have an image file called
`ssd.dd` that contains the target's root filesystem.

With the `ssd.dd` image available, we begin our step-by-step process to
cross-compile `libo3d3xx`. The first step is to mount the target's root
filesystem on to your laptop. We will mount the root filesystem at
`/mnt/rootfs` as that is the default expected location defined in the supplied
_toolchain files_ (more on that later). To do this, we will use the `kpartx`
tool available via `apt-get` on Debian or Ubuntu.

Let's first inspect our raw image file:

    $ sudo kpartx -l ssd_2015-10-29.dd
    loop0p1 : 0 31811584 /dev/loop0 2048
    loop0p2 : 0 20480000 /dev/loop0 31813632
    loop0p3 : 0 8192000 /dev/loop0 52293632
    loop0p4 : 0 2045952 /dev/loop0 60485632
    loop deleted : /dev/loop0

By examining this output, we see that my raw image file has four partitions
defined. Let's make them all available for mounting to my laptop:

    $ sudo kpartx -a -v ssd_2015-10-29.dd
    add map loop0p1 (252:0): 0 31811584 linear /dev/loop0 2048
    add map loop0p2 (252:1): 0 20480000 linear /dev/loop0 31813632
    add map loop0p3 (252:2): 0 8192000 linear /dev/loop0 52293632
    add map loop0p4 (252:3): 0 2045952 linear /dev/loop0 60485632

    $ ls -l /dev/mapper
    total 0
    crw------- 1 root        root         10, 236 Nov 16 08:38 control
    brw------- 1 tpanzarella tpanzarella 252,   0 Nov 16 12:03 loop0p1
    brw------- 1 tpanzarella tpanzarella 252,   1 Nov 16 12:03 loop0p2
    brw------- 1 tpanzarella tpanzarella 252,   2 Nov 16 12:03 loop0p3
    brw------- 1 tpanzarella tpanzarella 252,   3 Nov 16 12:03 loop0p4

Now, I happen to know that the root filesystem is located on the first
partition `loop0p1`. So let's mount that:

    $ sudo mount /dev/mapper/loop0p1 /mnt/rootfs/

Now, we sanity check ourselves:

    $ ls /mnt/rootfs/
    bin   dev  home  lost+found  mnt  proc  run   srv  tmp  var
    boot  etc  lib   media       opt  root  sbin  sys  usr

This next part is not necessary, but I will use it to illustrate that this root
filessystem is precisely what we need for building and linking our desired
`armhf` version of `libo3d3xx`:

    $ cd /mnt/rootfs/usr/lib/
    $ $ file libpcl_common.so.1.7.1
    libpcl_common.so.1.7.1: ELF 32-bit LSB  shared object, ARM, EABI5 version 1
    (SYSV), dynamically linked,
    BuildID[sha1]=4936e768eafef85ca6bebbc80f543e10e1a22d3c, stripped

We can even double check that it is in fact _hard float_ with:

    $ $ readelf -a libpcl_common.so.1.7.1 | grep -i 'FP'
    (... some output omitted ...)
    Tag_FP_arch: VFPv3-D16
    Tag_ABI_FP_denormal: Needed
    Tag_ABI_FP_exceptions: Needed
    Tag_ABI_FP_number_model: IEEE 754
    Tag_ABI_HardFP_use: SP and DP
    Tag_ABI_VFP_args: VFP registers

The `Tag_ABI_VFP_args: VFP registers` indicates that this is a hard-float
binary (soft float will not have this indicator).

Patching the Target Files
-------------------------

**NOTE:** This step may not be strictly necessary for you.

On Ubuntu 14.04 LTS armhf I ran into linker issues with `libc.so` and
`libpthread.so` Those files needed to be patched in the mounted rootfs prior to
cross-compiling. Just to be clear, do not patch these files on the embedded
system -- they are fine. Said patches are provided with the `libo3d3xx` source
distribution in the `cmake/toolchains/patches_arm-linux-gnueabihf`
directory. To apply:

    $ cd /mnt/rootfs/usr/lib/arm-linux-gnueabihf
    $ sudo patch -b libc.so libc.so.patch
    $ sudo patch -b libpthread.so libpthread.so.patch

Setting up the toolchain
------------------------

So far, so good. We have our `armhf` dependencies available to us on our
laptop. We now turn our attention to building the code. In order to do this, a
cross-compiler needs to be installed on your laptop and with that
cross-compiler installed, a _toolchain file_ needs to be specified to `cmake`
specifying that the specific cross-compilation toolchain is to be
used. Currently, the `libo3d3xx` project ships two toolchain files to be used
either directly or as an example for making your own for your specific
cross-compiler. The two provided assume that you are building for Linux ARM
hard-float. They are:

<table>
        <tr>
              <th>Vendor</th>
              <th>Version</th>
              <th>Link</th>
              <th>Toolchain File</th>
        </tr>

        <tr>
              <th>Linaro</th>
              <th>4.8-2014.04</th>
              <th>http://releases.linaro.org/14.04/components/toolchain/binaries/</th>
              <th>cmake/toolchains/linaro-arm-linux-gnueabihf.cmake</th>
        </tr>

        <tr>
              <th>Denx</th>
              <th>ELDK 5.5.3 armv7a-hf</th>
              <th>ftp://ftp.denx.de/pub/eldk/5.5.3/targets/armv7a-hf/eldk-eglibc-i686-arm-toolchain-qte-5.5.3.sh</th>
              <th>cmake/toolchains/eldk-armv7a-hf.cmake</th>
        </tr>
</table>

You can choose whichever cross-compilier you would like. For purposes of this
document, we will assume the Linaro one is being used. To install it, from the
link above, download the file
`gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.bz2`. Then:

    $ sudo cp gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.bz2 /opt/
    $ sudo tar xvjf gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.bz2

Thats it. Now you have your cross-compiler installed at:
`/opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux` which is the default
location that the `cmake/toolchains/linaro-arm-linux-gnueabihf.cmake`
assumes.

Building and deploying the code
-------------------------------

Building the code is done in the standard way it is done for a native build
except you need to tell `cmake` which cross-compiler to use. This is done by
specifying the toolchain file. Assuming you are in the top-level of this source
distribution:

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/linaro-arm-linux-gnueabihf.cmake ..
    $ make
    $ make package

**NOTE:** Beyond specifying the `-DCMAKE_TOOLCHAIN_FILE` directive, you can
  also specify any of the other options discussed [here](custom_builds.md).

A few things to note at this point. First, by default, the GUI viewer
application is not built as it is assumed you are deploying code to a headless
embedded Linux system. Second, the unit tests have been built but you will not
have a `make check` target available to you as the built code is for the ARM
architecture and you are running on x86. However, the tests will be bundled in
the deb file which you created via: `make package`.

Installing and testing the build
--------------------------------

Once you have built the code as discussed above, the only thing that is left is
to copy the binaries to the target and install them. Assuming your target is
Debian or Ubuntu, you built the deb as shown above, and your target is running
an SSH daemon, you can simply:

    $ make deploy

The deployment parameters can be customized as described
[here](custom_builds.md) and for the sake of brevity will not be
repeated. Assuming you accepted all of the defaults (which you likely will NOT
want to do), you can now:

    $ ssh lovepark@192.168.0.68

Now assuming you are on the remote machine:

    $ cd debs
    $ sudo dpkg -i libo3d3xx_0.2.0_armhf.deb

**NOTE:** The version string in the deb file may be different based upon the
  version of libo3d3xx that you are building.

Now the code is installed on the remote target. At this point, it is highly
advised that you copy the following snippet to your _login script_ (e.g.,
`~/.bashrc`):

    if [ -f /opt/libo3d3xx/etc/setup.bash ]; then
        source /opt/libo3d3xx/etc/setup.bash
    fi

With that in place, you should log out then log back in via SSH. Now, assuming
you are back on to the target machine and you want to run the unit tests, you
can do the following:

    $ cd /opt/libo3d3xx/test/
    $ . env.sh
    $ ./o3d3xx-tests

Assuming all of the tests passed, you are ready to use `libo3d3xx` on your
embedded system.
