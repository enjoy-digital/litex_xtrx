[> Intro
--------

[> Setup
--------

```bash
git clone --recursive https://github.com/enjoy-digital/litex_xtrx
```

### [> Gateware

```
./fairwaves_xtrx.py --build [--load] [--flash]
```

**NOTE:** python script may be modified for test purpose
- `with_analyzer` to enable/disable *litescope* support
- `with_rx_pattern` to replace *lms7002* rx stream by a pattern generator + a converter 32 -> 64
- `with_tx_pattern` to cut *DMA* -> *lms7002* TX stream and to replace by a converter 64->32 always ready
- `with_rx_scope` (must be enabled at the same time at `with_rx_pattern` and `with_analyzer` to have access to rx signals (pattern generator, dma sink and writer sink);
- `with_tx_scope` (must be enabled at the same time at `with_tx_pattern` and `with_analyzer` to have access to tx signals dma source and reader source and converter source);

### [> Software

#### Prerequisites

```bash
apt install gnuradio gnuradio-dev soapysdr-tools libsoapysdr0.8 libsoapysdr-dev libgnuradio-soapy3.10.9
```

#### kernel

located in *software/litepcie-kernel-module*

```bash
make clean
make
sudo ./init.sh
```

#### software

*LMS7002M-driver*
-----------------

located in *software/LMS7002M-driver*

```bash
mkdir build
cd build
cmake ../
make
sudo make install
```

*litepcie-user-library*:
------------------------

located in *software/litepcie-user-library*

```bash
make clean
make
```

Detail: **TBD**

*soapysdr-xtrx*
---------------

located in *software/soapysdr-xtrx*

```bash
mkdir build
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
```

### [> Check if xtrx and soapy module are ready


#### With litepcie user application.
------------------------------------

**Identification**

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

**DMA test**

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

#### SoapySDR
-------------

`SoapySDRUtil --info`

must contains:
- *Module found: /usr/lib/x86_64-linux-gnu/SoapySDR/modules0.8/libSoapyLiteXXTRX.so*
- LiteXXTRX in *Available factories* list

And command:
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

[> GNU Radio
------------

Directory *gnuradio* provides some examples to test/use XTRX with soapysdr module: see *gnuradio/README.md*