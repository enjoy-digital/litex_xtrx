#!/usr/bin/env python3

#
# This file is part of LiteX-XTRX.
#
# Copyright (c) 2021-2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import argparse

from migen import *
from migen.fhdl.specials import Tristate
from migen.genlib.cdc    import MultiReg

from litex.gen import *

from litex_boards.platforms import fairwaves_xtrx
from litex_boards.platforms import limesdr_xtrx

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock     import *
from litex.soc.cores.led       import LedChaser
from litex.soc.cores.icap      import ICAP
from litex.soc.cores.gpio      import GPIOOut
from litex.soc.cores.spi_flash import S7SPIFlash
from litex.soc.cores.bitbang   import I2CMaster
from litex.soc.cores.spi       import SPIMaster
from litex.soc.cores.xadc      import XADC
from litex.soc.cores.dna       import DNA

from litepcie.phy.s7pciephy import S7PCIEPHY

from litescope import LiteScopeAnalyzer

from gateware.gpio        import GPIO
from gateware.aux         import AUX
from gateware.gps         import GPS
from gateware.vctcxo      import VCTCXO
from gateware.synchro     import Synchro
from gateware.rf_switches import RFSwitches
from gateware.lms7002m    import LMS7002M

from software import generate_litepcie_software

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_idelay = ClockDomain()

        # # #

        # Clk/Rst.
        assert sys_clk_freq == int(125e6)
        self.comb += self.cd_sys.clk.eq(ClockSignal("pcie"))
        self.comb += self.cd_sys.rst.eq(ResetSignal("pcie"))

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(ResetSignal("pcie"))
        pll.register_clkin(ClockSignal("pcie"), 125e6)
        pll.create_clkout(self.cd_idelay, 200e6)

        # IDelayCtrl.
        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    SoCCore.csr_map = {
        # SoC.
        "uart"        : 0,
        "icap"        : 1,
        "flash"       : 2,
        "xadc"        : 3,
        "dna"         : 4,

        # PCIe.
        "pcie_phy"    : 10,
        "pcie_msi"    : 11,
        "pcie_dma0"   : 12,

        # XTRX.
        "i2c0"        : 20,
        "i2c1"        : 21,
        "gpio"        : 22,
        "gps"         : 23,
        "vctcxo"      : 24,
        "rf_switches" : 25,
        "lms7002m"    : 26,
        "xsync_spi"   : 27,
        "synchro"     : 28,

        # Analyzer.
        "analyzer"    : 30,
    }

    def __init__(self, board="fairwaves", sys_clk_freq=int(125e6),
        with_cpu      =True, cpu_firmware=None,
        with_jtagbone = True,
        with_analyzer = True,
    ):
        # Platform ---------------------------------------------------------------------------------
        platform = {
            "fairwaves_cs"  : fairwaves_xtrx.Platform(variant="xc7a35t"),
            "fairwaves_pro" : fairwaves_xtrx.Platform(variant="xc7a50t"),
            "limesdr"       : limesdr_xtrx.Platform()
        }[board]

        # Configuration: See if useful in longterm and integrate properly.
        with_rx_pattern = False
        with_tx_test    = False
        with_rxtx_loop  = False
        with_analyzer   = True
        if with_analyzer:
            with_rx_scope   = True
            with_tx_scope   = False
            with_rxtx_scope = True

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = f"LiteX SoC on {board.capitalize()} XTRX ",
            ident_version            = True,
            cpu_type                 = "vexriscv" if with_cpu else None,
            cpu_variant              = "minimal",
            integrated_rom_size      = 0x8000 if with_cpu else 0,
            integrated_sram_ram_size = 0x1000 if with_cpu else 0,
            integrated_main_ram_size = 0x4000 if with_cpu else 0,
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little"),
            uart_name                = "crossover",
        )
        # Automatically jump to pre-initialized firmware.
        self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["main_ram"])
        # Avoid stalling CPU at startup.
        self.uart.add_auto_tx_flush(sys_clk_freq=sys_clk_freq, timeout=1, interval=128)

        # Clocking ---------------------------------------------------------------------------------
        self.crg = CRG(platform, sys_clk_freq)

        # JTAGBone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()
            platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
            platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # Leds -------------------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq
        )
        self.leds2 = LedChaser(
            pads         = platform.request_all("user_led2"),
            sys_clk_freq = sys_clk_freq
        )

        # ICAP -------------------------------------------------------------------------------------
        self.icap = ICAP()
        self.icap.add_reload()
        self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # SPIFlash ---------------------------------------------------------------------------------
        self.flash_cs_n = GPIOOut(platform.request("flash_cs_n"))
        self.flash      = S7SPIFlash(platform.request("flash"), sys_clk_freq, 25e6)

        # XADC -------------------------------------------------------------------------------------
        self.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # PCIe -------------------------------------------------------------------------------------
        self.pcie_phy = S7PCIEPHY(platform, platform.request(f"pcie_x2"),
            data_width = 64,
            bar0_size  = 0x20000,
            cd         = "pcie"
            #cd         = "sys"
        )
        self.add_pcie(phy=self.pcie_phy, address_width=32, ndmas=1,
            with_dma_buffering    = True, dma_buffering_depth=8192,
            with_dma_loopback     = True,
            with_dma_synchronizer = True,
            with_msi              = True
        )

        # I2C Bus0:
        # - Temperature Sensor (TMP108  @ 0x4a) Lime: (TMP1075 @ 0x4b).
        # - PMIC-LMS           (LP8758  @ 0x60).
        # - VCTCXO DAC         Rev4: (MCP4725 @ 0x62) Rev5: (DAC60501 @ 0x4b) Lime: (AD5693 @ 0x4c).
        self.i2c0 = I2CMaster(pads=platform.request("i2c", 0))

        # I2C Bus1:
        # PMIC-FPGA (LP8758 @ 0x60).
        self.i2c1 = I2CMaster(pads=platform.request("i2c", 1))

        # XSYNC SPI Bus:
        xsync_spi_pads      = platform.request("xsync_spi")
        xsync_spi_pads.miso = Signal() # SPI is 3-wire, add fake MISO. Will only allow writes, not reads.
        self.xsync_spi = SPIMaster(
            pads         = xsync_spi_pads,
            data_width   = 32,
            sys_clk_freq = sys_clk_freq,
            spi_clk_freq = 1e6
        )

        # PMIC-FPGA:
        # Buck0: 1.0V VCCINT + 1.0V MGTAVCC.
        # Buck1: 1.8V/3.3V VCCIO (DIGPRVDD2/DIGPRVDD3/DIGPRPOC + VDD18_TXBUF of LMS + Bank 0/14/16/34/35 of FPGA).
        # Buck2: 1.2V MGTAVTT + 1.2V VDLMS (VDD12_DIG / VDD_SPI_BUF / DVDD_SXR / DVDD_SXT / DVDD_CGEN).
        # Buck3: 1.8V VCCAUX  + 1.8V VDLMS (VDD18_DIG).

        # PMIC-LMS:
        # Buck0: +2.05V (used as input to 1.8V LDO for LMS analog 1.8V).
        # Buck1: +3.3V rail.
        # Buck2: +1.75V (used as input to 1.4V LDO for LMS analog 1.4V).
        # Buck3: +1.5V  (used as input to 1.25V LDO for LMS analog 1.25V).

        # Aux -------------------------------------------------------------------------------------
        self.aux = AUX(platform.request("aux"))

        # GPIO -------------------------------------------------------------------------------------
        #self.gpio = GPIO(platform.request("gpio"))

        # GPS --------------------------------------------------------------------------------------
        self.gps = GPS(platform.request("gps"), sys_clk_freq, baudrate=9600)

        # VCTCXO -----------------------------------------------------------------------------------
        vctcxo_pads = platform.request("vctcxo")
        self.vctcxo = VCTCXO(vctcxo_pads)
        platform.add_period_constraint(vctcxo_pads.clk, 20)

        # Synchro ----------------------------------------------------------------------------------
        self.synchro = Synchro(platform.request("synchro"))
        self.comb += self.synchro.pps_gps.eq(self.gps.pps)
        self.comb += self.pcie_dma0.synchronizer.pps.eq(self.synchro.pps)

        # RF Switches ------------------------------------------------------------------------------
        self.rf_switches = RFSwitches(platform.request("rf_switches"))

        # LMS7002M ---------------------------------------------------------------------------------
        self.lms7002m = LMS7002M(platform, platform.request("lms7002m"), sys_clk_freq,
            tx_delay_init = 16,
            rx_delay_init = 16
        )

        # FIXME: Too complicated to apprehend, simplify and integrate properly.
        if with_tx_test:
            self.tx0_i = Signal(16)
            self.tx0_q = Signal(16)
            self.tx1_i = Signal(16)
            self.tx1_q = Signal(16)
            self.comb += self.pcie_dma0.source.ready.eq(1),
            self.sync += [
                If(self.pcie_dma0.source.valid,
                    self.tx0_i.eq(self.pcie_dma0.source.data[ 0:16]),
                    self.tx0_q.eq(self.pcie_dma0.source.data[16:32]),
                    self.tx1_i.eq(self.pcie_dma0.source.data[32:48]),
                    self.tx1_q.eq(self.pcie_dma0.source.data[48:64]),
                )
            ]
        elif not with_rxtx_loop:
            self.comb += self.pcie_dma0.source.connect(self.lms7002m.sink)

        if with_rxtx_loop:
            self.comb += self.pcie_dma0.source.connect(self.pcie_dma0.sink)
        elif with_rx_pattern:
            class MYTXPatternGenerator(LiteXModule): # FIXME: Integrate Pattern properly or remove.
                def __init__(self):
                    self.source  = stream.Endpoint([("data", 32)])

                    # # #

                    # Control-Path.
                    # -------------
                    self.comb += self.source.valid.eq(1)
                    self.sync += self.source.last.eq(0)

                    # Generator.
                    # ----------

                    # Counter.
                    count = Signal(12)
                    self.sync += [
                        count.eq(count + 1),
                    ]

                    # Data-Path.
                    # ----------
                    self.sync += [
                        self.source.data.eq(0),
                        self.source.data[ 0:12].eq(count[ 0:11]), # FIXME: sign extension
                        self.source.data[16:28].eq(count[ 0:11]), # FIXME: sign extension
                    ]
            test_tx_pat = MYTXPatternGenerator()
            self.test_tx_pat = test_tx_pat
            self.rx_conv    = rx_conv    = stream.Converter(32, 64)
            self.comb += self.test_tx_pat.source.connect(self.rx_conv.sink, omit={"data"})
            self.comb += self.rx_conv.sink.data.eq(self.test_tx_pat.source.data & 0x0fff0fff)
            self.comb += self.rx_conv.source.connect(self.pcie_dma0.sink)
        else:
            self.comb += self.lms7002m.source.connect(self.pcie_dma0.sink, omit={"last"})

        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.lms7002m.cd_rfic.clk)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            #analyzer_signals = [platform.lookup_request("lms7002m")]
            analyzer_signals = [
                #self.lms7002m.sink,
                #self.lms7002m.source,
                #self.lms7002m.tx_frame,
                #self.lms7002m.tx_data,
                #self.lms7002m.rx_frame,
                #self.lms7002m.rx_aligned,
                #self.lms7002m.rx_data,
            ]

            if with_rx_scope:
                analyzer_signals += [
                    #self.pcie_dma0.sink,
                    self.pcie_dma0.writer.sink,
                ]

                if with_rx_pattern:
                    analyzer_signals += [
                        self.test_tx_pat.source
                    ]

            if with_rxtx_scope:
                analyzer_signals += [
                    self.pcie_dma0.reader.source,
                ]
            if with_tx_scope:
                analyzer_signals += [
                    #self.pcie_dma0.reader.source,
                    self.pcie_dma0.source.data,
                    self.pcie_dma0.source.ready,
                    self.pcie_dma0.source.valid,
                    self.pcie_dma0.source.last,
                    self.pcie_dma0.source.first,
                    self.tx0_i,
                    self.tx0_q,
                    self.tx1_i,
                    self.tx1_q,
                ]

            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 128,
                clock_domain = "sys",
                register     = True,
                csr_csv      = "analyzer.csv"
            )

        # Timing Constraints/False Paths -----------------------------------------------------------
        for i in range(4):
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks        dna_clk] -asynchronous")
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks       jtag_clk] -asynchronous")
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks       icap_clk] -asynchronous")
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks     vctcxo_clk] -asynchronous")
            platform.toolchain.pre_placement_commands.append(f"set_clock_groups -group [get_clocks {{{{*s7pciephy_clkout{i}}}}}] -group [get_clocks lms7002m_mclk1] -asynchronous")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Fairwaves/LimeSDR XTRX")
    parser.add_argument("--board",   default="fairwaves_pro", help="Select XTRX board.", choices=["fairwaves_cs", "fairwaves_pro", "limesdr"])
    parser.add_argument("--build",   action="store_true", help="Build bitstream.")
    parser.add_argument("--load",    action="store_true", help="Load bitstream.")
    parser.add_argument("--flash",   action="store_true", help="Flash bitstream.")
    parser.add_argument("--driver",  action="store_true", help="Generate PCIe driver from LitePCIe (override local version).")
    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        soc = BaseSoC(
            board        = args.board,
            cpu_firmware = None if prepare else "firmware/firmware.bin",
        )
        builder = Builder(soc, csr_csv="csr.csv")
        builder.build(run=build)
        if prepare:
            os.system(f"cd firmware && make BUILD_DIR={builder.output_dir} clean all")

    # Generate LitePCIe Driver.
    generate_litepcie_software(soc, "software", use_litepcie_software=args.driver)

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    # Flash Bitstream.
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, os.path.join(builder.gateware_dir, soc.build_name + ".bin"))

if __name__ == "__main__":
    main()
