[> GNU Radio tests/examples.
----------------------------

This directory contains some GNU Radio examples primarily intended to validate Pcie/DMA behavior

- *xtrx_testRX.grc*: Receives a flow from XTRX in CF32 (Complex float) format. When gateware is built with `with_rx_pattern` the stream is a ramp from 0 to (2^11)-1. I and Q have the same value. An additional *GNU/Octave* (**xtrx_testRX.m** script is provided to display data stored.
- *xtrx_testTX.grc*: TX generator (signal_source or constant source connected to the SoapySDR Custom block). With a gateware configured with `with_tx_test` samples may be seen using *litescope*