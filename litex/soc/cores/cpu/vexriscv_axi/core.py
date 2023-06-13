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
from litex.soc.integration.soc import SoCRegion, SoCIORegion
from litex.soc.integration.soc import SoCBusHandler

from axi_axil_adapter import AXI2AXILiteAdapter
from axi_crossbar import AXICrossbar
from axi_common import *
from litex.soc.integration.doc import AutoDoc,ModuleDoc

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

class VexRiscvAxi(CPU, AutoDoc):
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
        0xE000_0000 : 0x1000_0000,
    }

    # Memory Mapping.
    @property
    def mem_map(self):
        return {
            "periph"   : 0x4000_0000, # for Daric peripherals + Litex peripherals (need to divide up the space more finely)
            "memory"   : 0x6000_0000, # 0x2000_0000
            "norflash" : 0x7000_0000, # this is included inside the "memory" range; we can address up to 0x8000_0000
            "csr"      : 0xE000_0000, # put RISCV-specific CSRs in the upper region of Daric I/O
        }

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags = "-march=rv32i2p0_mac   -mabi=ilp32"
        flags += " -D__vexriscv__"
        return flags

    def __init__(self, platform, variant="standard-debug", with_timer=False, link_docs=True):
        self.platform         = platform
        self.variant          = variant
        self.human_name       = "VexRiscvAxi4"
        self.external_variant = None
        self.interrupt        = Signal(32)

        MEMORY_LEN = 0x2000_0000 # this addresses [0x6000_0000 : 0x7FFF_FFFF], so it can hit the lower 128MiB of `norflash` region.
        # terrible pseudo-parser to extract I/O ranges from the source file used to generate the verilog definition of the CPU
        non_cached_prefix = []
        SOURCE=os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../../VexRiscv/GenCramSoC.scala"))
        with open(SOURCE, "r") as s:
            lines = s.readlines()
            inside_mmu_indent = None
            for line in lines:
                if inside_mmu_indent is not None:
                    cheesyparse = line.rstrip().split('===')
                    if len(cheesyparse) > 1:
                        assert '31 downto 28' in cheesyparse[0], "Rewrite Scala source parsing to handle a wider range of ioRange decoder cases!"
                        if cheesyparse[0].lstrip(' ').startswith('//'): # filter commented lines
                            continue
                        non_cached_prefix += [int(cheesyparse[1].split('//')[0], 16)] # remove trailing comments
                    indent = len(line) - len(line.lstrip(' '))
                    if indent <= inside_mmu_indent:
                        inside_mmu_indent = None
                if 'new MmuPlugin' in line:
                    inside_mmu_indent = len(line) - len(line.lstrip(' '))

            non_cached_prettyprint = ""
            start_region = None
            end_region = None
            for region in non_cached_prefix:
                if start_region is None:
                    start_region = region
                    end_region = region
                else:
                    if region - end_region == 1:
                        end_region = region
                    else:
                        non_cached_prettyprint += "   - 0x{:X}0000000 - {:X}FFFFFFF\n".format(start_region, end_region )
                        start_region = region
                        end_region = region
            non_cached_prettyprint += "   - 0x{:X}0000000 - {:X}FFFFFFF\n".format(start_region, end_region)

        self.intro = ModuleDoc("""
This `VexRiscv <https://github.com/SpinalHDL/VexRiscv#vexriscv-architecture>`_ core provides the following bus interfaces:

- 64-bit AXI-4 instruction cache bus (read-only cached) with full access to all locations in memory.
- Data bus crossbar

  - 0x{:X}-{:X}: 32-bit AXI-4 data cache bus (r/w cached)
  - 0x{:X}-{:X}: 32-bit AXI-lite peripheral bus (r/w uncached)
  - 0x{:x}-{:x}: 32-bit internal bus (r/w uncached) local to RV32
  - All other locations in memory are not accessible by RV32 LD/ST unit
- All busses run at ACLK speed
- WFI signal is broken out to `wfi_active`
- `satp` signals broken out for `coreuser` interpretation

The core itself contains the following features:

- VexRiscv CPU (simple, in-order RV32-IMAC with pipelining)
- Static branch prediction
- 4k, 4-way D-cache
- 4k, 4-way I-cache
- MMU and 8-entry TLB
- AES instruction extensions
- Non-cached regions (used for I/O):

{}
   - Any non-cached regions not routed through peripheral bus are internal to the core block

        """.format(
            self.mem_map["memory"], self.mem_map["memory"] + MEMORY_LEN - 1,
            self.mem_map["periph"], self.io_regions[self.mem_map["periph"]] + self.mem_map["periph"] - 1,
            self.mem_map["csr"], self.io_regions[self.mem_map["csr"]] + self.mem_map["csr"] - 1,
            non_cached_prettyprint,
        ))

        # SoC-specific signals -------------------------------------------------------
        # Trimming bits come from the ReRAM fuse bank. They are set at wafer sort.
        # Reset vector is trimmable because there are two cores in the system with conflicting
        # binary types, and we need to coordinate their start points in a flexible manner.
        self.trimming_reset   = Signal(32)
        self.trimming_reset_ena = Signal()

        # SATP breakout. This is used to generate the "coreuser" signal, which indicates
        # that the system is currently executing code in a trusted process space.
        self.satp_mode = Signal()
        self.satp_asid = Signal(9)
        self.satp_ppn = Signal(22)

        # WFI active signal
        self.wfi_active = Signal()
        # Privilege state
        self.privilege = Signal(2)

        # self-test signals
        self.cmbist = Signal()
        self.cmatpg = Signal()

        # Create AXI-Full Interfaces, attached to the CPU
        self.ibus_axi   =  ibus = axi.AXIInterface(data_width=64, address_width=32, id_width = 1, bursting=True)
        self.dbus_axi   = axi.AXIInterface(data_width=32, address_width=32, id_width = 1, bursting=True)

        # adapt AXI->AXIlite with a verilog module
        dbus_peri = axi.AXIInterface(data_width=32, address_width=32, id_width=1, bursting=False)
        peripherals = axi.AXILiteInterface(data_width=32, address_width=32)
        self.submodules += AXI2AXILiteAdapter(platform, dbus_peri, peripherals)

        # adapt AXI->AXIlite with a verilog module
        axi_csr = axi.AXIInterface(data_width=32, address_width=32, id_width=1, bursting=True)
        corecsr = axi.AXILiteInterface(data_width=32, address_width=32)
        self.submodules += AXI2AXILiteAdapter(platform, axi_csr, corecsr)

        # This is the dbus downstream of the crossbar, in contrast to dbus_axi, which is connected upstream to the CPU.
        dbus = axi.AXIInterface(data_width=32, address_width=32, id_width=1, bursting=True)

        # Create a crossbar to split out the AXI-full dbus to an axi-lite p-bus and an AXI-full dbus
        d_xbar = AXICrossbar(platform=platform)
        self.submodules += d_xbar
        d_xbar.add_slave(name = "cpu", s_axi=self.dbus_axi,
            aw_reg = AXIRegister.BYPASS,
            w_reg  = AXIRegister.BYPASS,
            b_reg  = AXIRegister.BYPASS,
            ar_reg = AXIRegister.BYPASS,
            r_reg  = AXIRegister.BYPASS,
        )
        d_xbar.add_master(name = "peripherals", m_axi=dbus_peri, origin=self.mem_map["periph"],
            size=self.io_regions[self.mem_map["periph"]])
        d_xbar.add_master(name = "corecsr", m_axi=axi_csr, origin=self.mem_map["csr"],
            size=self.io_regions[self.mem_map["csr"]])
        d_xbar.add_master(name = "memory", m_axi=dbus, origin=self.mem_map["memory"], size=MEMORY_LEN)

        # Expose AXI-Lite Interfaces.
        self.periph_buses     = [corecsr] # Peripheral buses (Connected to main SoC's bus). Leave blank because we don't want any bus to me inferred for the core generation.
        self.memory_buses     = [
            ['ibus', ibus],
            ['dbus', dbus],
            ['pbus', peripherals],
        ]   # Memory buses (Connected directly to crossbar to main memory).

        # CPU Instance.
        self.cpu_params = dict(
            i_clk                    = ClockSignal("sys"),
            i_reset                  = ResetSignal("sys"),

            i_externalInterruptArray = self.interrupt,
            i_timerInterrupt         = 0,
            i_softwareInterrupt      = 0,

            o_MmuPlugin_satp_mode = self.satp_mode,
            o_MmuPlugin_satp_asid = self.satp_asid,
            o_MmuPlugin_satp_ppn = self.satp_ppn,

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
            o_iBusAxi_ar_payload_region = ibus.ar.region, # not on M3

            i_iBusAxi_r_valid          = ibus.r.valid,
            o_iBusAxi_r_ready          = ibus.r.ready,
            i_iBusAxi_r_payload_last   = ibus.r.last,
            i_iBusAxi_r_payload_resp   = ibus.r.resp,
            i_iBusAxi_r_payload_data   = ibus.r.data,
            i_iBusAxi_r_payload_id     = ibus.r.id, # not on M3

            # Data Bus (AXI).
            o_dBusAxi_aw_valid          = self.dbus_axi.aw.valid,
            i_dBusAxi_aw_ready          = self.dbus_axi.aw.ready,
            o_dBusAxi_aw_payload_addr   = self.dbus_axi.aw.addr,
            o_dBusAxi_aw_payload_burst  = self.dbus_axi.aw.burst,
            o_dBusAxi_aw_payload_cache  = self.dbus_axi.aw.cache,
            o_dBusAxi_aw_payload_len    = self.dbus_axi.aw.len,
            o_dBusAxi_aw_payload_lock   = self.dbus_axi.aw.lock,
            o_dBusAxi_aw_payload_prot   = self.dbus_axi.aw.prot,
            o_dBusAxi_aw_payload_size   = self.dbus_axi.aw.size,
            o_dBusAxi_aw_payload_id     = self.dbus_axi.aw.id, # not on M3
            o_dBusAxi_aw_payload_region = self.dbus_axi.aw.region, # not on M3
            o_dBusAxi_aw_payload_qos    = self.dbus_axi.aw.qos, # not on M3

            o_dBusAxi_w_valid           = self.dbus_axi.w.valid,
            i_dBusAxi_w_ready           = self.dbus_axi.w.ready,
            o_dBusAxi_w_payload_last    = self.dbus_axi.w.last,
            o_dBusAxi_w_payload_strb    = self.dbus_axi.w.strb,
            o_dBusAxi_w_payload_data    = self.dbus_axi.w.data,

            i_dBusAxi_b_valid           = self.dbus_axi.b.valid,
            o_dBusAxi_b_ready           = self.dbus_axi.b.ready,
            i_dBusAxi_b_payload_resp    = self.dbus_axi.b.resp,
            i_dBusAxi_b_payload_id      = self.dbus_axi.b.id, # not on M3

            o_dBusAxi_ar_valid          = self.dbus_axi.ar.valid,
            i_dBusAxi_ar_ready          = self.dbus_axi.ar.ready,
            o_dBusAxi_ar_payload_addr   = self.dbus_axi.ar.addr,
            o_dBusAxi_ar_payload_burst  = self.dbus_axi.ar.burst,
            o_dBusAxi_ar_payload_cache  = self.dbus_axi.ar.cache,
            o_dBusAxi_ar_payload_len    = self.dbus_axi.ar.len,
            o_dBusAxi_ar_payload_lock   = self.dbus_axi.ar.lock,
            o_dBusAxi_ar_payload_prot   = self.dbus_axi.ar.prot,
            o_dBusAxi_ar_payload_size   = self.dbus_axi.ar.size,
            o_dBusAxi_ar_payload_id     = self.dbus_axi.ar.id, # not on M3
            o_dBusAxi_ar_payload_region = self.dbus_axi.ar.region, # not on M3
            o_dBusAxi_ar_payload_qos    = self.dbus_axi.ar.qos, # not oon M3

            i_dBusAxi_r_valid           = self.dbus_axi.r.valid,
            o_dBusAxi_r_ready           = self.dbus_axi.r.ready,
            i_dBusAxi_r_payload_last    = self.dbus_axi.r.last,
            i_dBusAxi_r_payload_resp    = self.dbus_axi.r.resp,
            i_dBusAxi_r_payload_data    = self.dbus_axi.r.data,
            i_dBusAxi_r_payload_id      = self.dbus_axi.r.id, # not on M3

            o_CsrPlugin_inWfi           = self.wfi_active,
            o_CsrPlugin_privilege       = self.privilege,

            i_CMBIST                    = self.cmbist,
            i_CMATPG                    = self.cmatpg,
        )
        platform.add_source_dir("VexRiscv/VexRiscv_CramSoC.v")

        # Add Timer (Optional).
        if with_timer:
            self.add_timer()

        # Add Debug (Optional).
        if "debug" in variant:
            self.add_debug()

    def add_jtag(self, pads):
        jtag_disable = Signal()
        self.comb += jtag_disable.eq(pads.disable)
        jtag_tdo = Signal()
        self.comb += pads.tdo.eq(~jtag_disable & jtag_tdo)
        self.cpu_params.update(
            i_jtag_tdi      = ~jtag_disable & pads.tdi,
            o_jtag_tdo      = jtag_tdo,
            i_jtag_tms      = ~jtag_disable & pads.tms,
            i_jtag_tck      = ~jtag_disable & pads.tck,
            i_debugReset    = pads.trst,
        )

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        reset_mux = Signal(32, reset=reset_address)
        self.comb += [
            If(self.trimming_reset_ena,
                reset_mux.eq(self.trimming_reset)
            ).Else(
                reset_mux.eq(Signal(32, reset=reset_address))
            )
        ]
        self.cpu_params.update(i_externalResetVector=reset_mux)

    def add_timer(self):
        self.submodules.timer = VexRiscvTimer()
        self.cpu_params.update(i_timerInterrupt=self.timer.interrupt)

    def add_debug(self):
        self.o_resetOut   = Signal()
        reset_debug_logic = Signal()
        debug_reset       = Signal()
        self.sync += reset_debug_logic.eq(self.o_resetOut)
        self.sync += debug_reset.eq(reset_debug_logic | ResetSignal())

        self.cpu_params.update(
            i_reset            = ResetSignal() | debug_reset,
            o_debug_resetOut   = self.o_resetOut
        )

    @staticmethod
    def add_sources(platform, variant="standard"):
        platform.add_source("VexRiscv.v")

    def add_soc_components(self, soc):
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
