#
# This file is part of LiteX-XTRX.
#
# Copyright (c) 2021-2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.cores.uart import UARTPHY, UART

# GPS ----------------------------------------------------------------------------------------------

class GPS(LiteXModule):
    def __init__(self, pads, sys_clk_freq, baudrate=9600):
        self.control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, values=[
                ("``0b0``", "Disable GPS."),
                ("``0b1``", "Enable GPS.")
            ], reset=0),
        ])
        self.pps = Signal()

        # # #

        # Drive Control Pin.
        self.comb += pads.rst.eq(~self.control.fields.enable)

        # PPS Resynchronization.
        self.specials += MultiReg(pads.pps, self.pps)

        # UART.
        self.uart_phy = UARTPHY(pads, sys_clk_freq, baudrate=baudrate)
        self.uart     = UART(self.uart_phy, rx_fifo_rx_we=True)
