
Custom Builds of libo3d3xx
==========================

As of the `0.2.0` release, `libo3d3xx` can be customized (by
design). Specifically, the following options exist:

Standard Build Options
----------------------

<table>
  <tr>
        <th>Option</th>
        <th>Default</th>
        <th>Description</th>

  </tr>

  <tr>
       <td>CMAKE_INSTALL_PREFIX</td>

       <td>/opt/libo3d3xx</td>

       <td>
       The root level directory of where to install the software.
       </td>
  </tr>

  <tr>
       <td>BUILD_TESTS</td>

       <td>ON</td>

       <td>
       Build the unit tests. This gives you a `make check` target that can be
       used to test the software prior to installing it. It should be noted
       that in the case of cross-compiling the code, specifying `BUILD_TESTS`
       will build the units tests but will not provide the `make check`
       target. Instead the tests will be bundled into the `deb` file produced
       by `make package` so that the unit tests can be run on the target
       embedded system as a standard binary.
       </td>
  </tr>

  <tr>
       <td>BUILD_SHARED_LIBS</td>

       <td>ON</td>

       <td>
       Builds the libo3d3xx.so shared library. It should be noted that at
       least one of BUILD_SHARED_LIBS or BUILD_STATIC_LIBS must be turned
       ON. When BUILD_SHARED_LIBS is ON, the binaries that get built will also
       be dynamically linked to this built shared libary (even if
       BUILD_STATIC_LIBS is specified).
       </td>
  </tr>

  <tr>
       <td>BUILD_STATIC_LIBS</td>

       <td>ON</td>

       <td>
       Builds the libo3d3xx_static.a static library. It should be noted that
       when linking to this static library, dependent libraries may be linked
       dynamically.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_VIEWER</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-viewer application. It should be noted that in the
       case of cross-compiling, this option is implicitly turned OFF because
       the current cross-compiliation support assumes the target is an embedded
       linkux system as opposed to something like Mac OS X or Windows. This may
       be changed in the future, however, for now, this is the behavior.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_VERSION</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-version application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_RESET</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-reset application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_LS</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-ls application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_DUMP</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-dump application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_CONFIG</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-config application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_RM</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-rm application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_REBOOT</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-reboot application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_HZ</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-hz application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_IMAGER_TYPES</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-imager-types application.
       </td>
  </tr>


  <tr>
       <td>BUILD_EXE_IFM_IMPORT</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-ifm-import application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_IFM_EXPORT</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-ifm-export application.
       </td>
  </tr>

  <tr>
       <td>BUILD_EXE_SCHEMA</td>

       <td>ON</td>

       <td>
       Builds the o3d3xx-schema application.
       </td>
  </tr>
</table>


Cross-compilation Build Options
-------------------------------
Currently, cross-compiling `libo3d3xx` assumes the target device is an embedded
Linux system. Further conveniences are available if the system is an embedded
Debian or Ubuntu variant. Specifically, options are available for deploying the
built code to the target via `scp`. These options are described below. A
complete tutorial on cross-compiling `libo3d3xx` is available
[here](cross_compiling.md).

<table>
  <tr>
        <th>Option</th>
        <th>Default</th>
        <th>Description</th>
  </tr>

  <tr>
        <td>CMAKE_TOOLCHAIN_FILE</td>
        <td>(empty)</td>
        <td>
        When this option is specified, the build assumes that it is
        cross-compiling the code, typically for a different target architecture
        (e.g., ARM).
        </td>
  </tr>

  <tr>
        <td>TARGET_IP</td>
        <td>192.168.0.68</td>
        <td>
        The IP address of the embedded system to deploy the deb file to.
        </td>
  </tr>

  <tr>
        <td>TARGET_USER</td>
        <td>lovepark</td>
        <td>
        The username to specify to the scp executable when attempting to login
        to the remote target device.
        </td>
  </tr>

  <tr>
        <td>TARGET_DIR</td>
        <td>/home/lovepark/debs/</td>
        <td>
        The full path to the directory on the remote machine to copy the deb
        file to.
        </td>
  </tr>

</table>


Examples
--------
The preferred way to specify these build options is on the command line while
running `cmake`. So, for example, if you wanted to build the project but you
did not want to build the GUI viewer application or the static libraries, your
build and installation process would look like:

	$ mkdir build
	$ cd build
	$ cmake -DBUILD_EXE_VIEWER=OFF -DBUILD_STATIC_LIBS=OFF ..
	$ make
	$ make check
    $ make package
    $ sudo dpkg -i libo3d3xx_0.2.0_amd64.deb
