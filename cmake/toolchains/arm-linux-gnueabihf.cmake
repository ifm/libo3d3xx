#
# CMake cross-compile toolchain file for ARM EABI Hard Float Linux systems
#
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSROOT /mnt/rootfs)

set(CROSSTOOL_ROOT /opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux)
set(CMAKE_C_COMPILER
    "${CROSSTOOL_ROOT}/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER
    "${CROSSTOOL_ROOT}/bin/arm-linux-gnueabihf-g++")

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

#set(CMAKE_CXX_FLAGS "--sysroot=${CMAKE_SYSROOT}")

set(CMAKE_LIBRARY_PATH
    "${CMAKE_SYSROOT}/lib/arm-linux-gnueabihf"
    "${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf")

set(CMAKE_FIND_LIBRARY_PREFIXES lib)
set(CMAKE_FIND_LIBRARY_SUFFIXES .so)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
