#
# This file is part of LiteX.
#
# Copyright (c) 2022 Ilia Sergachev <ilia@sergachev.ch>
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV32
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import axi
from litex.soc.interconnect import wishbone
from litex.soc.integration.soc import SoCRegion

class Open(Signal): pass

# VexRiscv Timer -----------------------------------------------------------------------------------

class VexRiscvTimer(Module, AutoCSR):
    def __init__(self):
        self._latch    = CSR()
        self._time     = CSRStatus(64)
        self._time_cmp = CSRStorage(64, reset=2**64-1)
        self.interrupt = Signal()

        # # #

        time = Signal(64)
        self.sync += time.eq(time + 1)
        self.sync += If(self._latch.re, self._time.status.eq(time))

        time_cmp = Signal(64, reset=2**64-1)
        self.sync += If(self._latch.re, time_cmp.eq(self._time_cmp.storage))

        self.comb += self.interrupt.eq(time >= time_cmp)


# VexRiscv on AXI ----------------------------------------------------------------------------------------

class VexRiscvAxi(CPU):
    category             = "softcore"
    family               = "riscv"
    name                 = "vexriscv_axi"
    human_name           = "VexRiscv AXI"
    data_width           = 32
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV32
    linker_output_format = "elf32-littleriscv"
    nop                  = "nop"
    variants             = ["standard"]
    io_regions           = {
        # Origin, Length.
        0x4000_0000 : 0x2000_0000,
        0xa000_0000 : 0x6000_0000,
    }

    # Memory Mapping.
    @property
    def mem_map(self):
        return {
            "rom"      : 0x6000_0000,
            "sram"     : 0x2000_0000,
            "main_ram" : 0x6100_0000,
            "csr"      : 0xa000_0000,
        }

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags = "-march=rv32i2p0_mac   -mabi=ilp32"
        flags += " -D__vexriscv__"
        return flags

    def __init__(self, platform, variant="standard-debug", with_timer=False):
        self.platform     = platform
        self.variant          = variant
        self.human_name       = "VexRiscvAxi4"
        self.external_variant = None
        self.reset            = Signal()
        self.interrupt        = Signal(32)

        # Create AXI-Full Interfaces.
        self.ibus_axi   =  ibus = axi.AXIInterface(data_width=64, address_width=32, id_width = 1)
        self.dbus_axi   =  dbus = axi.AXIInterface(data_width=32, address_width=32, id_width = 1)

        # Create AXI-Lite Interfaces.
        self.dbus             = dbus_lite = axi.AXILiteInterface(data_width=32, address_width=32)

        # Adapt AXI interfaces to AXILite.
        self.submodules += axi.AXI2AXILite(dbus, dbus_lite)

        # Expose AXI-Lite Interfaces.
        self.periph_buses     = [dbus_lite] # Peripheral buses (Connected to main SoC's bus).
        self.memory_buses     = [['ibus', ibus], ['dbus', dbus]]   # Memory buses (Connected directly to LiteDRAM).

        # CPU Instance.
        self.cpu_params = dict(
            i_clk                    = ClockSignal("sys"),
            i_reset                  = ResetSignal("sys") | self.reset,

            i_externalInterruptArray = self.interrupt,
            i_timerInterrupt         = 0,
            i_softwareInterrupt      = 0,

            # Debug.
            # i_debugReset = ResetSignal() | self.reset, # handled in debugReset

            # Instruction Bus (AXI).
            #o_AWVALIDC = ibus.aw.valid,
            #i_AWREADYC = ibus.aw.ready,
            #o_AWADDRC  = ibus.aw.addr,
            #o_AWBURSTC = ibus.aw.burst,
            #o_AWCACHEC = ibus.aw.cache,
            #o_AWLENC   = ibus.aw.len,
            #o_AWLOCKC  = ibus.aw.lock,
            #o_AWPROTC  = ibus.aw.prot,
            #o_AWSIZEC  = ibus.aw.size,

            #o_WVALIDC  = ibus.w.valid,
            #i_WREADYC  = ibus.w.ready,
            #o_WLASTC   = ibus.w.last,
            #o_WSTRBC   = ibus.w.strb,
            #o_HWDATAC  = ibus.w.data,

            #i_BVALIDC  = ibus.b.valid,
            #o_BREADYC  = ibus.b.ready,
            #i_BRESPC   = ibus.b.resp,

            o_iBusAxi_ar_valid         = ibus.ar.valid,
            i_iBusAxi_ar_ready         = ibus.ar.ready,
            o_iBusAxi_ar_payload_addr  = ibus.ar.addr,
            o_iBusAxi_ar_payload_burst = ibus.ar.burst,
            o_iBusAxi_ar_payload_cache = ibus.ar.cache,
            o_iBusAxi_ar_payload_len   = ibus.ar.len,
            o_iBusAxi_ar_payload_lock  = ibus.ar.lock,
            o_iBusAxi_ar_payload_prot  = ibus.ar.prot,
            o_iBusAxi_ar_payload_size  = ibus.ar.size,
            o_iBusAxi_ar_payload_id    = ibus.ar.id, # not on M3
            o_iBusAxi_ar_payload_qos   = ibus.ar.qos, # not on M3
            o_iBusAxi_ar_payload_region = Open(), # ibus.ar.region, # not on M3

            i_iBusAxi_r_valid          = ibus.r.valid,
            o_iBusAxi_r_ready          = ibus.r.ready,
            i_iBusAxi_r_payload_last   = ibus.r.last,
            i_iBusAxi_r_payload_resp   = ibus.r.resp,
            i_iBusAxi_r_payload_data   = ibus.r.data,
            i_iBusAxi_r_payload_id     = ibus.r.id, # not on M3

            # Data Bus (AXI).
            o_dBusAxi_aw_valid         = dbus.aw.valid,
            i_dBusAxi_aw_ready         = dbus.aw.ready,
            o_dBusAxi_aw_payload_addr  = dbus.aw.addr,
            o_dBusAxi_aw_payload_burst = dbus.aw.burst,
            o_dBusAxi_aw_payload_cache = dbus.aw.cache,
            o_dBusAxi_aw_payload_len   = dbus.aw.len,
            o_dBusAxi_aw_payload_lock  = dbus.aw.lock,
            o_dBusAxi_aw_payload_prot  = dbus.aw.prot,
            o_dBusAxi_aw_payload_size  = dbus.aw.size,
            o_dBusAxi_aw_payload_id    = dbus.aw.id, # not on M3
            o_dBusAxi_aw_payload_region = Open(), # dbus.aw.region, # not on M3
            o_dBusAxi_aw_payload_qos   = dbus.aw.qos, # not on M3

            o_dBusAxi_w_valid          = dbus.w.valid,
            i_dBusAxi_w_ready          = dbus.w.ready,
            o_dBusAxi_w_payload_last   = dbus.w.last,
            o_dBusAxi_w_payload_strb   = dbus.w.strb,
            o_dBusAxi_w_payload_data   = dbus.w.data,

            i_dBusAxi_b_valid          = dbus.b.valid,
            o_dBusAxi_b_ready          = dbus.b.ready,
            i_dBusAxi_b_payload_resp   = dbus.b.resp,
            i_dBusAxi_b_payload_id     = dbus.b.id, # not on M3

            o_dBusAxi_ar_valid         = dbus.ar.valid,
            i_dBusAxi_ar_ready         = dbus.ar.ready,
            o_dBusAxi_ar_payload_addr  = dbus.ar.addr,
            o_dBusAxi_ar_payload_burst = dbus.ar.burst,
            o_dBusAxi_ar_payload_cache = dbus.ar.cache,
            o_dBusAxi_ar_payload_len   = dbus.ar.len,
            o_dBusAxi_ar_payload_lock  = dbus.ar.lock,
            o_dBusAxi_ar_payload_prot  = dbus.ar.prot,
            o_dBusAxi_ar_payload_size  = dbus.ar.size,
            o_dBusAxi_ar_payload_id    = dbus.ar.id, # not on M3
            o_dBusAxi_ar_payload_region = Open(), # dbus.ar.region, # not on M3
            o_dBusAxi_ar_payload_qos   = dbus.ar.qos, # not oon M3

            i_dBusAxi_r_valid          = dbus.r.valid,
            o_dBusAxi_r_ready          = dbus.r.ready,
            i_dBusAxi_r_payload_last   = dbus.r.last,
            i_dBusAxi_r_payload_resp   = dbus.r.resp,
            i_dBusAxi_r_payload_data   = dbus.r.data,
            i_dBusAxi_r_payload_id     = dbus.r.id, # not on M3
        )
        platform.add_source_dir("deps/pythondata-cpu-vexriscv/pythondata_cpu_vexriscv/verilog/VexRiscv_CranSoC.v")

        # Add Timer (Optional).
        if with_timer:
            self.add_timer()

        # Add Debug (Optional).
        if "debug" in variant:
            self.add_debug()

    def add_jtag(self, pads):
        self.cpu_params.update(
            i_jtag_tdi      = pads.tdi,
            o_jtag_tdo      = pads.tdo,
            i_jtag_tms      = pads.tms,
            i_jtag_tck      = pads.tck,
            i_debugReset    = pads.trst,
        )

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        self.cpu_params.update(i_externalResetVector=Signal(32, reset=reset_address))

    def add_timer(self):
        self.submodules.timer = VexRiscvTimer()
        self.cpu_params.update(i_timerInterrupt=self.timer.interrupt)

    def add_debug(self):
        debug_reset = Signal()

        # ibus_err = Signal()
        # dbus_err = Signal()

        # self.i_cmd_valid           = Signal()
        # self.i_cmd_payload_wr      = Signal()
        # self.i_cmd_payload_address = Signal(8)
        # self.i_cmd_payload_data    = Signal(32)
        # self.o_cmd_ready           = Signal()
        # self.o_rsp_data            = Signal(32)
        self.o_resetOut            = Signal()

        reset_debug_logic = Signal()

        # self.transfer_complete     = Signal()
        # self.transfer_in_progress  = Signal()
        # self.transfer_wait_for_ack = Signal()

        # self.debug_bus = wishbone.Interface()

        # self.sync += self.debug_bus.dat_r.eq(self.o_rsp_data)
        self.sync += debug_reset.eq(reset_debug_logic | ResetSignal())

        self.sync += [
            # # CYC is held high for the duration of the transfer.
            # # STB is kept high when the transfer finishes (write)
            # # or the master is waiting for data (read), and stays
            # # there until ACK, ERR, or RTY are asserted.
            # If((self.debug_bus.stb & self.debug_bus.cyc)
            # & (~self.transfer_in_progress)
            # & (~self.transfer_complete)
            # & (~self.transfer_wait_for_ack),
            #     self.i_cmd_payload_data.eq(self.debug_bus.dat_w),
            #     self.i_cmd_payload_address.eq((self.debug_bus.adr[0:6] << 2) | 0),
            #     self.i_cmd_payload_wr.eq(self.debug_bus.we),
            #     self.i_cmd_valid.eq(1),
            #     self.transfer_in_progress.eq(1),
            #     self.transfer_complete.eq(0),
            #     self.debug_bus.ack.eq(0)
            # ).Elif(self.transfer_in_progress,
            #     If(self.o_cmd_ready,
            #         self.i_cmd_valid.eq(0),
            #         self.i_cmd_payload_wr.eq(0),
            #         self.transfer_complete.eq(1),
            #         self.transfer_in_progress.eq(0)
            #     )
            # ).Elif(self.transfer_complete,
            #     self.transfer_complete.eq(0),
            #     self.debug_bus.ack.eq(1),
            #     self.transfer_wait_for_ack.eq(1)
            # ).Elif(self.transfer_wait_for_ack & ~(self.debug_bus.stb & self.debug_bus.cyc),
            #     self.transfer_wait_for_ack.eq(0),
            #     self.debug_bus.ack.eq(0)
            # ),
            # Force a Wishbone error if transferring during a reset sequence.
            # Because o_resetOut is multiple cycles and i.stb/d.stb should
            # deassert one cycle after i_err/i_ack/d_err/d_ack are asserted,
            # this will give i_err and o_err enough time to be reset to 0
            # once the reset cycle finishes.
            If(self.o_resetOut,
                # If(self.ibus.cyc & self.ibus.stb, ibus_err.eq(1)).Else(ibus_err.eq(0)),
                # If(self.dbus.cyc & self.dbus.stb, dbus_err.eq(1)).Else(dbus_err.eq(0)),
                reset_debug_logic.eq(1))
            .Else(
                reset_debug_logic.eq(0)
            )
        ]

        self.cpu_params.update(
            i_reset = ResetSignal() | self.reset | debug_reset,
            # i_iBusWishbone_ERR              = self.ibus.err | ibus_err,
            # i_dBusWishbone_ERR              = self.dbus.err | dbus_err,
            # i_debugReset                    = ResetSignal(), # this is provided by JTAG
            # i_debug_bus_cmd_valid           = self.i_cmd_valid,
            # i_debug_bus_cmd_payload_wr      = self.i_cmd_payload_wr,
            # i_debug_bus_cmd_payload_address = self.i_cmd_payload_address,
            # i_debug_bus_cmd_payload_data    = self.i_cmd_payload_data,
            # o_debug_bus_cmd_ready           = self.o_cmd_ready,
            # o_debug_bus_rsp_data            = self.o_rsp_data,
            o_debug_resetOut                = self.o_resetOut
        )

    @staticmethod
    def add_sources(platform, variant="standard"):
        platform.add_source("VexRiscv.v")

    def add_soc_components(self, soc, soc_region_cls):
        # Connect Debug interface to SoC.
        if "debug" in self.variant:
            soc.bus.add_slave("vexriscv_debug", self.debug_bus, region=
                soc_region_cls(
                    origin = soc.mem_map.get("vexriscv_debug"),
                    size   = 0x100,
                    cached = False
                )
            )

        # Pass I/D Caches info to software.
        base_variant = str(self.variant.split('+')[0])
        # DCACHE is present on all variants except minimal and lite.
        if not base_variant in ["minimal", "lite"]:
            soc.add_config("CPU_HAS_DCACHE")
        # ICACHE is present on all variants except minimal.
        if not base_variant in ["minimal"]:
            soc.add_config("CPU_HAS_ICACHE")

    def use_external_variant(self, variant_filename):
        self.external_variant = True
        self.platform.add_source(variant_filename)

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        if not self.external_variant:
            self.add_sources(self.platform, self.variant)
        self.specials += Instance("VexRiscvAxi4", **self.cpu_params)
        if hasattr(self, "cfu_params"):
            self.specials += Instance("Cfu", **self.cfu_params)

"""
module VexRiscvAxi4 (
  input      [31:0]   externalResetVector,//*
  input               timerInterrupt,//
  input               softwareInterrupt,//
  input      [31:0]   externalInterruptArray,//
  output              debug_resetOut,//*
  output              iBusAxi_ar_valid,//
  input               iBusAxi_ar_ready,//
  output     [31:0]   iBusAxi_ar_payload_addr,//
  output     [0:0]    iBusAxi_ar_payload_id, <---
  output     [3:0]    iBusAxi_ar_payload_region,
  output     [7:0]    iBusAxi_ar_payload_len,//
  output     [2:0]    iBusAxi_ar_payload_size,//
  output     [1:0]    iBusAxi_ar_payload_burst,//
  output     [0:0]    iBusAxi_ar_payload_lock,//
  output     [3:0]    iBusAxi_ar_payload_cache,//
  output     [3:0]    iBusAxi_ar_payload_qos, <---
  output     [2:0]    iBusAxi_ar_payload_prot,//
  input               iBusAxi_r_valid,//
  output              iBusAxi_r_ready,//
  input      [63:0]   iBusAxi_r_payload_data,//
  input      [0:0]    iBusAxi_r_payload_id, <---
  input      [1:0]    iBusAxi_r_payload_resp,//
  input               iBusAxi_r_payload_last,//
  output              dBusAxi_aw_valid,//
  input               dBusAxi_aw_ready,//
  output     [31:0]   dBusAxi_aw_payload_addr,//
  output     [0:0]    dBusAxi_aw_payload_id, <---
  output     [3:0]    dBusAxi_aw_payload_region, <---
  output     [7:0]    dBusAxi_aw_payload_len,//
  output     [2:0]    dBusAxi_aw_payload_size,//
  output     [1:0]    dBusAxi_aw_payload_burst,//
  output     [0:0]    dBusAxi_aw_payload_lock,//
  output     [3:0]    dBusAxi_aw_payload_cache,//
  output     [3:0]    dBusAxi_aw_payload_qos, <---
  output     [2:0]    dBusAxi_aw_payload_prot,//
  output              dBusAxi_w_valid,//
  input               dBusAxi_w_ready,//
  output     [31:0]   dBusAxi_w_payload_data,//
  output     [3:0]    dBusAxi_w_payload_strb,//
  output              dBusAxi_w_payload_last,//
  input               dBusAxi_b_valid,//
  output              dBusAxi_b_ready,//
  input      [0:0]    dBusAxi_b_payload_id, <---
  input      [1:0]    dBusAxi_b_payload_resp,//
  output              dBusAxi_ar_valid,//
  input               dBusAxi_ar_ready,//
  output     [31:0]   dBusAxi_ar_payload_addr,//
  output     [0:0]    dBusAxi_ar_payload_id, <---
  output     [3:0]    dBusAxi_ar_payload_region, <---
  output     [7:0]    dBusAxi_ar_payload_len,//
  output     [2:0]    dBusAxi_ar_payload_size,//
  output     [1:0]    dBusAxi_ar_payload_burst,//
  output     [0:0]    dBusAxi_ar_payload_lock,//
  output     [3:0]    dBusAxi_ar_payload_cache,//
  output     [3:0]    dBusAxi_ar_payload_qos, <---
  output     [2:0]    dBusAxi_ar_payload_prot,//
  input               dBusAxi_r_valid,//
  output              dBusAxi_r_ready,//
  input      [31:0]   dBusAxi_r_payload_data,//
  input      [0:0]    dBusAxi_r_payload_id, <---
  input      [1:0]    dBusAxi_r_payload_resp,//
  input               dBusAxi_r_payload_last,//
  input               jtag_tms,//*
  input               jtag_tdi,//*
  output              jtag_tdo,//*
  input               jtag_tck,//*
  input               clk,//
  input               reset,//
  input               debugReset//
);
"""