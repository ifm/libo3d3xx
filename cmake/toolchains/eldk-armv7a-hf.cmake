#
# Toolchain file for Linux on ARMv7a hard float (armhf) architecture
#
# Tested with the following cross-compilers:
#
# - Denx ELDK 5.5.3 armv7a-hf
#   ftp://ftp.denx.de/pub/eldk/5.5.3/targets/armv7a-hf
#
#   From the above link, download this file:
#     eldk-eglibc-i686-arm-toolchain-qte-5.5.3.sh
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

set(CROSSTOOL_ROOT /opt/eldk-5.5.3/armv7a-hf/sysroots/i686-eldk-linux)
set(CMAKE_C_COMPILER
    "${CROSSTOOL_ROOT}/usr/bin/arm-linux-gnueabi/arm-linux-gnueabi-gcc")
set(CMAKE_CXX_COMPILER
    "${CROSSTOOL_ROOT}/usr/bin/arm-linux-gnueabi/arm-linux-gnueabi-g++")

set(CMAKE_FIND_ROOT_PATH
    ${CMAKE_SYSROOT}
    ${CROSSTOOL_ROOT}/../armv7ahf-vfp-neon-linux-gnueabi
   )
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_INCLUDE_PATH /include /usr/include)
set(CMAKE_LIBRARY_PATH /lib
                       /lib/arm-linux-gnueabihf
                       /lib/arm-linux-gnueabi
                       /usr/lib
                       /usr/lib/arm-linux-gnueabihf
                       /usr/lib/arm-linux-gnueabi)

set(CROSSTOOL_COMPILE_FLAGS
    "-march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon")

set(CROSSTOOL_EXE_LINKER_FLAGS
    "-Wl,-rpath,${CMAKE_SYSROOT}/lib:${CMAKE_SYSROOT}/lib/arm-linux-gnueabihf")

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
