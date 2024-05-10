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

![](https://github.com/enjoy-digital/litex_xtrx/assets/1450143/23a10c1f-0971-413a-a607-3439b13941eb.png)

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
./litex_xtrx.py --build --flash [--driver]
```

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
cd software/kernel
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

#### [> Install LMS7002MNG Driver (extracted from LimeSuiteNG)
```bash
cd software/lms7002mNG
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

#### [> PCIe Rescan

```bash
echo 1 | sudo tee /sys/bus/pci/devices/0000\:0x\:00.0/remove (get x with lspci)
echo 1 | sudo tee /sys/bus/pci/rescan
```bash

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
