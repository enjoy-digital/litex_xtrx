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
#### [> Installing LiteX:

LiteX can be installed by following the installation instructions from the LiteX Wiki: https://github.com/enjoy-digital/litex/wiki/Installation

#### [> Installing the RISC-V toolchain for the Soft-CPU:

To get and install a RISC-V toolchain, please install it manually of follow the LiteX's wiki: https://github.com/enjoy-digital/litex/wiki/Installation:
```bash
./litex_setup.py --gcc=riscv
```
#### [> Clone repository:

```bash
git clone --recursive https://github.com/enjoy-digital/litex_xtrx
```

#### [> Software Prerequisites

```bash
apt install gnuradio gnuradio-dev soapysdr-tools libsoapysdr0.8 libsoapysdr-dev libgnuradio-soapy3.10.1
```

[> Build and Test the design
----------------------------

#### [> Build and Flash the design

Build the design and flash it to the board:
```bash
./fairwaves_xtrx.py --build --flash [--driver]
```

>Notes: python script may be modified for test purpose
- `with_analyzer` to enable/disable *litescope* support
- `with_rx_pattern` to replace *lms7002* rx stream by a pattern generator + a converter 32 -> 64
- `with_tx_pattern` to cut *DMA* -> *lms7002* TX stream and to replace by a converter 64->32 always ready
- `with_rx_scope` (must be enabled at the same time at `with_rx_pattern` and `with_analyzer` to have access to rx signals (pattern generator, dma sink and writer sink);
- `with_tx_scope` (must be enabled at the same time at `with_tx_pattern` and `with_analyzer` to have access to tx signals dma source and reader source and converter source);

#### [> Check Board Presence
Reboot or Rescan PCIe Bus (Optional):
```bash
echo 1 | sudo tee /sys/bus/pci/devices/0000\:0X\:00.0/remove (replace X with actual value)
echo 1 | sudo tee /sys/bus/pci/rescan
```

`0X:00.0 Memory controller: Xilinx Corporation Device 7022` should then be seen with lspci.


#### [> Build and load Linux Kernel Module
Build the Linux kernel and load it:
```bash
cd software/litepcie/kernel
make
sudo ./init.sh
```

#### [> Test Firmware

The firmware is automatically integrated in the SoC during the build and can be executed with:
```bash
sudo litex_term /dev/ttyLXU0
```

For development, firmware can be recompiled and reloaded with:
```bash
cd firmware
make
sudo litex_term /dev/ttyLXU0 --kernel=firmware.bin --safe
```

#### [> Install LMS7002M Driver
```bash
cd software/LMS7002M-driver
mkdir build
cd build
cmake ../
make
sudo make install
```

#### [> Install SoapySDR Driver
```bash
cd software/soapysdr-xtrx
mkdir build
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
```

Test driver with:

`SoapySDRUtil --info`

It must contains:
- *Module found: /usr/lib/x86_64-linux-gnu/SoapySDR/modules0.8/libSoapyLiteXXTRX.so*
- LiteXXTRX in *Available factories* list

And following commands should display similar results:
```bash
SoapySDRUtil --find="driver=LiteXXTRX"
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Found device 0
  driver = LiteXXTRX
  identification = LiteX SoC on Fairwaves XTRX 300a43e-dirty 2024-03-12 09:19:59
  path = /dev/litepcie0
  serial = 13245867a084854
```

[> Use Apps
-----------

#### [> LitePCIe Utils

Identification

```bash
./litepcie_util info
[> FPGA/SoC Information:
------------------------
FPGA Identifier:  LiteX SoC on Fairwaves XTRX 300a43e-dirty 2024-03-12 11:15:36.
FPGA DNA:         0x013245867a084854
FPGA Temperature: 71.4 °C
FPGA VCC-INT:     0.91 V
FPGA VCC-AUX:     1.77 V
FPGA VCC-BRAM:    0.91 V
```

DMA test

```bash
./litepcie_util dma_test
[> DMA loopback test:
---------------------
DMA_SPEED(Gbps) TX_BUFFERS      RX_BUFFERS      DIFF    ERRORS
          7.01        2689            2561       128         0
          6.86        5345            5217       128         0
          6.84        7969            7841       128         0
          6.84       10593           10465       128         0
          6.84       13216           13088       128         0
          6.84       15840           15712       128         0
          6.86       18496           18368       128         0
          6.84       21120           20992       128         0
          6.84       23744           23616       128         0
          6.86       26400           26272       128         0
```

#### [> LimeSuite

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

#### [> LiteScope

LiteScope:
```
litex_server --jtag --jtag-config=openocd_xc7_ft232.cfg
litescope_cli
```

#### [> GNU Radio

Directory *app/gnuradio* provides some examples to test/use XTRX with soapysdr module: see *gnuradio/README.md*

[> Contact
----------
E-mail: florent@enjoy-digital.fr
