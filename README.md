                          __   _ __      _  __    _  ___________  _  __
                         / /  (_) /____ | |/_/___| |/_/_  __/ _ \| |/_/
                        / /__/ / __/ -_)>  </___/>  <  / / / , _/>  <
                       /____/_/\__/\__/_/|_|   /_/|_| /_/ /_/|_/_/|_|
                      XTRX LiteX/LitePCIe based alternative FPGA design
                          Copyright (c) 2021-2024 Enjoy-Digital.
                          Copyright (c) 2021-2022 Julia Computing.

[> Intro
--------

This project aims to recreate a FPGA design for the XTRX board with
LiteX/LitePCIe:

![](https://user-images.githubusercontent.com/1450143/147348139-503834af-76d5-4172-8ca0-e323b719fa17.png)

[> Getting started
------------------

### [> Installing LiteX:

LiteX can be installed by following the installation instructions from the LiteX
Wiki: https://github.com/enjoy-digital/litex/wiki/Installation

Be sure to use the latest master branch of the LiteX sources, as none of the
currently-available tags include some of the changes we require.

### [> Installing the RISC-V toolchain for the Soft-CPU:

To get and install a RISC-V toolchain, please install it manually of follow the
LiteX's wiki: https://github.com/enjoy-digital/litex/wiki/Installation:

```
./litex_setup.py --gcc=riscv
```


[> Build and Test the design
----------------------------

Build the design and flash it to the board:

```
./fairwaves_xtrx.py --build --flash
```

Build the Linux kernel driver and load it.
Note that by default, the current live kernel will be built against, but you can cross-compile for a target kernel version by setting `USE_LIVE_KERNEL=false`.

```
make -C software litepcie/kernel
sudo software/litepcie/kernel/init.sh
```

Note that if a thunderbolt carrier is in use, it may be necessary rescan the pci bus:

```
sudo bash -c 'echo "1" > /sys/bus/pci/rescan'
```

Build the Linux user-space utilities and test them:

```
make -C software litepcie/user -j$(nproc)
cd build/litepcie/user
./litepcie_util info
./litepcie_util scratch_test
./litepcie_util dma_test
```

If anything goes wrong, reset the device with:

```
sudo bash -c 'echo "1" > /sys/bus/pci/devices/0000\:02\:00.0/reset'
```


[> User-space software
----------------------

To interface with the LMS7002M chip, we provide a SoapySDR driver. This driver
is in turn built on top of [MyriadRF's LMS7002M driver
library](https://github.com/myriadrf/LMS7002M-driver), which is downloaded and
installed automatically when you compile the SoapySDR driver:

```
make -C software soapysdr-xtrx -j$(nproc)
export SOAPY_SDR_PLUGIN_PATH="$(make -sC software print-soapysdr-plugin-path)"
```

The above snippet sets `SOAPY_SDR_PLUGIN_PATH` so that any SoapySDR application
can find the XTRX driver.

There is also a modified version of LimeSuite available that makes it possible
to interactively configure the LMS7002M:

```
make -C software limesuite -j$(nproc)
```

You can then run it out of the `build/soapysdr/bin` directory.  Note that we
install to the `soapysdr` directory to simplify the path manipulation needed
for SoapySDR module autodetection.

[> Development
--------------

Both using LimeSuite and SoapySDR it's possible to load LMS7002M register dumps,
e.g., to test specific functionality:

- TX-RX FPGA internal loopback:

  ```
  LimeSuiteGUI (and open/load xtrx_dlb.ini)
  cd software/app
  make
  ./litex_xtrx_util lms_set_tx_rx_loopback 1
  ./litex_xtrx_util dma_test -e -w 12
  ```

- TX Pattern + LMS7002M loopback test:

  ```
  LimeSuiteGUI (and open/load xtrx_dlb.ini)
  cd software/app
  make
  ./litex_xtrx_util lms_set_tx_rx_loopback 0
  ./litex_xtrx_util lms_set_tx_pattern 1
  ../user/litepcie_test record dump.bin 0x100
  ```

- DMA+LMS7002 loopback test:

  ```
  LimeSuiteGUI (and open/load xtrx_dlb.ini)
  cd software/app
  make
  ./litex_xtrx_util lms_set_tx_rx_loopback 0
  ./litex_xtrx_util lms_set_tx_pattern 0
  ./litex_xtrx_util dma_test -e -w 12
  ```

To work with the embedded RISC-V CPU, the firmware (which is normally
automatically compiled and integrated in the SoC during build) can be recompiled
and reloaded with:

```
cd firmware
make
sudo litex_term /dev/ttyLXU0 --kernel=firmware.bin --safe
```

LiteScope:

```
litex_server --jtag --jtag-config=openocd_xc7_ft232.cfg
litescope_cli
```

GLScopeClient (WIP):
https://github.com/juliacomputing/scopehal-apps/tree/sjk/xtrx1


[> Contact
----------

E-mail: florent@enjoy-digital.fr
