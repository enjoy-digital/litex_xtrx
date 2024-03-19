#
# This file is part of LiteX-XTRX.
#
# Copyright (c) 2021-2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.spi import SPIMaster

# GPIO ---------------------------------------------------------------------------------------------

class GPIO(LiteXModule):
    def __init__(self, pads):
        # CSRs.
        self.control = CSRStorage(fields=[
            CSRField("gpio",  size=len(pads), offset=0, reset=0)
        ])

        # # #

        # Drive Control Pins.
        self.comb += [
            pads.eq(self.control.fields.gpio),
        ]
