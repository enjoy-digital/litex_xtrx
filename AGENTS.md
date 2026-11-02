# AGENTS.md: LiteX-XTRX

LiteX/LitePCIe alternative gateware and software for the Fairwaves XTRX SDR (Artix-7 + LMS7002M RFIC):
PCIe DMA, LMS7002M digital interface, GPS/time sync, RF switches, SoapySDR support.

## Where things are

- `litex_xtrx.py` (SoC/target); `gateware/` (`lms7002m.py`, `synchro.py`, `vctcxo.py`, `gps.py`, `rf_switches.py`, `gpio.py`, `aux.py`).
- `software/`: kernel driver, `user/` tools, `lms7002m/`, `soapysdr/`, `build_driver.py`; `firmware/`; `app/`; `doc/` (LMS7002M, XTRX, xsync).
- `test/`: hardware test scripts (`test_lms7002m_digital_interface.py`, `test_prbs.py`), run against a board.

## Verify a change

```sh
./litex_xtrx.py --help                                              # SoC options (checks it imports with the installed LiteX)
./litex_xtrx.py --build                                             # gateware build (Vivado)
```

- LMS7002M interface changes: run the PRBS/digital interface test scripts on hardware, at several
  sample rates.

## LiteX

This project is built with LiteX: read https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions,
how to verify a change, contribute back the generic fixes to LiteX and its cores).

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and the SoC identity first, registers
before traces, narrow LiteScope captures, read the generated Verilog, turn findings into tests).
`litex_mcp` gives an agent the same loop as MCP tools through `litex_server` (CSRs by name, LiteScope
captures as data, console, builds with parsed timing, simulation; writes and `--load`/`--flash`
only with `--allow-write`): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.

## Rules

- LiteX style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md.
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, generated Verilog): change the Python.
- Never flash a board or write its SPI flash unless the user asked; `--load` (volatile) for experiments.
- Never commit flash dumps, stock firmware or vendor bitstreams.
- Pull requests: one topic each, with what was tested and on which hardware; say that an agent wrote
  it and who supervised.
