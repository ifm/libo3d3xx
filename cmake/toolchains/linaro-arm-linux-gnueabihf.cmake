#
# Toolchain file for Linux on ARM hard float (armhf) architecture
#
# Tested with the following cross-compilers:
#
# - Linaro 4.8-2014.04:
#   http://releases.linaro.org/14.04/components/toolchain/binaries/
#
# ---------------------------------------------------------------
# ---------------------------------------------------------------
#
# IMPORTANT NOTE:
#
# On Ubuntu 14.04 LTS armhf I ran into linker issues with libc.so
# and libpthread.so. Those files need to be patched in your mounted
# rootfs prior to cross-compiling. Just to be clear, do not patch
# the files on your embedded system -- they are fine. Said patches
# are provided (as unified diffs) in `patches_arm-linux-gnueabihf`.
# To apply:
#
# $ cd /mnt/rootfs/usr/lib/arm-linux-gnueabihf
# $ sudo patch -b libc.so libc.so.patch
# $ sudo patch -b libpthread.so libpthread.so.patch
#
# Backup files are made as `.orig` in the same directory.
#
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSROOT /mnt/rootfs)

set(CROSSTOOL_ROOT /opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux)
set(CMAKE_C_COMPILER "${CROSSTOOL_ROOT}/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${CROSSTOOL_ROOT}/bin/arm-linux-gnueabihf-g++")

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CROSSTOOL_EXE_LINKER_FLAGS
    "-Wl,-rpath,${CMAKE_SYSROOT}/lib:${CMAKE_SYSROOT}/lib/arm-linux-gnueabihf")

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
