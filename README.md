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

LiteX can be installed by following the installation instructions from the LiteX Wiki: https://github.com/enjoy-digital/litex/wiki/Installation

### [> Installing the RISC-V toolchain for the Soft-CPU:

To get and install a RISC-V toolchain, please install it manually of follow the LiteX's wiki: https://github.com/enjoy-digital/litex/wiki/Installation:
```
./litex_setup.py --gcc=riscv
```

[> Build and Test the design(s)
---------------------------------

Build the design and flash it to the board:
```
./fairwaves_xtrx.py --build --flash --driver
```

Build the Linux kernel and load it:
```
cd software/kernel
make
sudo ./init.sh
```

Build the Linux user-space utilities and test them:
```
cd software/user
make
./litepcie_util info
./litepcie_util scratch_test
./litepcie_util dma_test
```

The firmware is automatically integrated in the SoC during the build and can be executed with:
```
sudo litex_term /dev/ttyLXU0
```

For development, firmware can be recompiled and reloaded with:
```
cd firmware
make
sudo litex_term /dev/ttyLXU0 --kernel=firmware.bin --safe
```

Get/Use modified LimeSuite:
```
git clone https://github.com/JuliaComputing/LimeSuite
cd LimeSuite
git checkout tb/xtrx_litepcie
mkdir builddir
cd builddir
LITEPCIE_ROOT=/path/to/xtrx_julia/software cmake -DENABLE_XTRX=yes -DCMAKE_BUILD_TYPE=Debug ../
make
sudo make install
```

Get/Use modified LMS7002M-driver:
```
git clone https://github.com/JuliaComputing/LMS7002M-driver
cd LMS7002M-driver
git checkout tb/xtrx
mkdir build
cd build
LITEPCIE_ROOT=/path/to/xtrx_julia/software cmake -DCMAKE_BUILD_TYPE=Debug ../
make
sudo make install
```

TX-RX FPGA internal loopback test:
```
LimeSuiteGUI (and open/load xtrx_dlb.ini)
cd software/app
make
./litex_xtrx_util lms_set_tx_rx_loopback 1
./litex_xtrx_util dma_test -e -w 12
```

TX Pattern + LMS7002M loopback test:
```
LimeSuiteGUI (and open/load xtrx_dlb.ini)
cd software/app
make
./litex_xtrx_util lms_set_tx_rx_loopback 0
./litex_xtrx_util lms_set_tx_pattern 1
../user/litepcie_test record dump.bin 0x100
```

DMA+LMS7002 loopback test:
```
LimeSuiteGUI (and open/load xtrx_dlb.ini)
cd software/app
make
./litex_xtrx_util lms_set_tx_rx_loopback 0
./litex_xtrx_util lms_set_tx_pattern 0
./litex_xtrx_util dma_test -e -w 12
```

LiteScope:
```
litex_server --jtag --jtag-config=openocd_xc7_ft232.cfg
litescope_cli
```

[> Contact
-------------
E-mail: florent@enjoy-digital.fr
