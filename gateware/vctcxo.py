#
# This file is part of LiteX-XTRX.
#
# Copyright (c) 2021-2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

# VCTCXO --------------------------------------------------------------------------------------------

class VCTCXO(LiteXModule):
    def __init__(self, pads):
        self.control = CSRStorage(fields=[
            CSRField("sel", size=1, offset=0, values=[
                ("``0b0``", "Use VCTCXO Clk."),
                ("``0b1``", "Use External Clk.")
            ], reset=0),
            CSRField("en", size=1, offset=1, values=[
                ("``0b0``", "Disable VCTCXO"),
                ("``0b1``", "Enable VCTCXO")
            ], reset=1),
        ])
        self.cycles_latch = CSR()
        self.cycles       = CSRStatus(32, reset=0)

        # # #

        # Drive Control Pins.
        self.comb += [
            pads.sel.eq(self.control.fields.sel),
            pads.en.eq(1), # FIXME: Add dynamic control and check original firmware.
        ]

        # Clock Input.
        self.cd_txco = ClockDomain("vctcxo")
        self.comb += self.cd_txco.clk.eq(pads.clk)

        # Cycles Count.
        self.cycles_count = Signal(32)
        self.sync.vctcxo += self.cycles_count.eq(self.cycles_count + 1)
        self.sync += If(self.cycles_latch.re, self.cycles.status.eq(self.cycles_count))
